#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point32, PolygonStamped, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import math
import tf2_ros
from tf2_ros import TransformException
import random
import numpy as np

MAP_FRAME = 'map'
SAFE_DIST = 1.2  # ÇOK ARTIRILDI - Daha erken fark etsin
CRITICAL_DIST = 0.7  # Kritik mesafe
EMERGENCY_DIST = 0.4  # ACİL DURUM!
ZIGZAG_STEP = 0.4
POSITION_TOLERANCE = 0.15
ANGLE_TOLERANCE = 0.2

# Oda koordinatları
ROOMS = [
    {'name': 'living_room', 'xmin': 0.1, 'xmax': 5.8, 'ymin': -5.8, 'ymax': -0.1},
    {'name': 'dining_kitchen', 'xmin': 0.1, 'xmax': 5.8, 'ymin': 0.1, 'ymax': 5.8},
    {'name': 'bedroom_1', 'xmin': -5.8, 'xmax': -0.1, 'ymin': 0.1, 'ymax': 5.8},
    {'name': 'bedroom_2', 'xmin': -5.8, 'xmax': -0.1, 'ymin': -0.1, 'ymax': -5.8},
]

DOCK = (0.0, 0.0)


class ZigZagCleaner(Node):

    def __init__(self):
        super().__init__('zigzag_cleaner')

        self.state = 'IDLE'
        self.room_idx = 0
        self.waypoints = []
        self.wp_idx = 0
        self.latest_scan = None
        
        # AGRESIF ENGEL ALGILAMA
        self.obstacle_detected = False
        self.min_distance = 10.0
        self.obstacle_left = False
        self.obstacle_right = False
        self.obstacle_front = False
        
        # KAÇINMA DURUMU
        self.avoiding = False
        self.avoid_phase = 0  # 0=geri, 1=dön, 2=ilerle
        self.avoid_counter = 0
        self.turn_direction = 1
        
        # TAKILMA
        self.stuck_counter = 0
        self.last_position = None
        self.position_history = []
        self.max_history = 30

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, 'planned_path', 10)
        self.robot_path_pub = self.create_publisher(Path, 'robot_path', 10)
        self.boundary_pub = self.create_publisher(PolygonStamped, 'cleaning_boundary', 10)
        self.current_goal_pub = self.create_publisher(Marker, 'current_goal', 10)
        self.waypoint_markers_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)
        
        # Subscribers
        self.create_subscription(PointStamped, '/clicked_point', self.rviz_click_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot_path = Path()
        self.robot_path.header.frame_id = MAP_FRAME
        
        self.planned_path = Path()
        self.planned_path.header.frame_id = MAP_FRAME

        self.last_click_time = None
        self.click_count = 0

        # Timers - HIZLI DÖNGÜ
        self.create_timer(0.03, self.loop)  # 30Hz ana döngü!
        self.create_timer(0.1, self.update_robot_path)
        self.create_timer(1.0, self.publish_boundary)
        self.create_timer(2.0, self.status_update)
        self.create_timer(0.3, self.check_stuck)
        self.create_timer(0.2, self.publish_current_goal)
        self.create_timer(1.0, self.publish_waypoint_markers)

        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('🤖 SÜPER AGRESİF ENGEL KAÇINMA MODU!')
        self.get_logger().info('🛡️ Engel mesafesi: 1.2m (Daha erken fark eder)')
        self.get_logger().info('⚡ 30Hz döngü hızı (Çok hızlı tepki)')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('🎮 KONTROLLER:')
        self.get_logger().info('   ▶️  START  → "2D Pose Estimate"')
        self.get_logger().info('   ⏸️  PAUSE  → "Publish Point" 1x')
        self.get_logger().info('   🛑 STOP   → "Publish Point" 2x hızlı')
        self.get_logger().info('═══════════════════════════════════════')

    # ================= RViz KONTROL =================
    def rviz_goal_cb(self, msg):
        if self.state != 'IDLE':
            self.get_logger().warn('⚠️ Zaten çalışıyor!')
            return
        self.room_idx = 0
        self.robot_path.poses.clear()
        self.position_history.clear()
        self.avoiding = False
        self.start_room()
        self.get_logger().info('🚀 BAŞLADI!')

    def rviz_click_cb(self, msg):
        current_time = self.get_clock().now()
        
        if self.last_click_time:
            time_diff = (current_time - self.last_click_time).nanoseconds / 1e9
            if time_diff < 1.0:
                self.click_count += 1
            else:
                self.click_count = 1
        else:
            self.click_count = 1
        
        self.last_click_time = current_time

        if self.click_count >= 2:
            self.state = 'IDLE'
            self.stop_robot()
            self.click_count = 0
            self.get_logger().error('🛑 DURDURULDU!')
            return

        if self.state == 'CLEANING':
            self.state = 'PAUSED'
            self.stop_robot()
            self.get_logger().warn('⏸️ PAUSE')
        elif self.state == 'PAUSED':
            self.state = 'CLEANING'
            self.get_logger().info('▶️ RESUME')

    # ================= AGRESIF SENSÖR =================
    def scan_cb(self, msg):
        """ÇOK AGRESİF ENGEL ALGILAMA"""
        self.latest_scan = msg
        
        if not msg.ranges or len(msg.ranges) == 0:
            self.obstacle_detected = False
            return

        ranges = np.array(msg.ranges)
        # Geçerli mesafeleri al
        valid_mask = (ranges > 0.05) & (ranges < 10.0)
        valid_ranges = ranges[valid_mask]
        
        if len(valid_ranges) == 0:
            self.obstacle_detected = False
            return

        # GLOBAL MİN MESAFE
        self.min_distance = np.min(valid_ranges)

        mid = len(ranges) // 2
        
        # GENİŞ ALGILAMA ALANLARI
        front_width = 50   # Çok geniş ön alan
        side_width = 60    # Çok geniş yan alanlar
        
        # ÖN bölge - ÇOK GENİŞ
        front_start = max(0, mid - front_width)
        front_end = min(len(ranges), mid + front_width)
        front_ranges = ranges[front_start:front_end]
        front_valid = front_ranges[(front_ranges > 0.05) & (front_ranges < 10.0)]
        
        # SOL bölge
        left_start = max(0, mid + front_width)
        left_end = min(len(ranges), mid + front_width + side_width)
        left_ranges = ranges[left_start:left_end]
        left_valid = left_ranges[(left_ranges > 0.05) & (left_ranges < 10.0)]
        
        # SAĞ bölge
        right_start = max(0, mid - front_width - side_width)
        right_end = mid - front_width
        right_ranges = ranges[right_start:right_end]
        right_valid = right_ranges[(right_ranges > 0.05) & (right_ranges < 10.0)]
        
        # ENGEL TESPİTİ
        self.obstacle_front = len(front_valid) > 0 and np.min(front_valid) < SAFE_DIST
        self.obstacle_left = len(left_valid) > 0 and np.min(left_valid) < SAFE_DIST
        self.obstacle_right = len(right_valid) > 0 and np.min(right_valid) < SAFE_DIST
        
        self.obstacle_detected = self.obstacle_front or self.obstacle_left or self.obstacle_right
        
        # ACİL DURUM!
        if self.min_distance < EMERGENCY_DIST:
            if not self.avoiding:
                self.get_logger().error(f'🚨🚨🚨 ACİL! {self.min_distance:.2f}m')
                self.emergency_response()
        elif self.min_distance < CRITICAL_DIST and self.obstacle_front:
            if not self.avoiding:
                self.get_logger().warn(f'⚠️ KRİTİK! {self.min_distance:.2f}m')
                self.start_avoidance()
        elif self.obstacle_detected:
            if not self.avoiding:
                self.get_logger().info(f'🛡️ ENGEL! {self.min_distance:.2f}m')
                self.start_avoidance()

    def emergency_response(self):
        """ACİL DURUM - HEMEN GERİ VE DÖN"""
        self.stop_robot()
        self.avoiding = True
        self.avoid_phase = 0
        self.avoid_counter = 150  # Çok uzun kaçınma
        
        # EN AÇIK YÖNE DÖN
        if not self.obstacle_left and self.obstacle_right:
            self.turn_direction = 1  # Sol açık
        elif not self.obstacle_right and self.obstacle_left:
            self.turn_direction = -1  # Sağ açık
        else:
            self.turn_direction = random.choice([-1, 1])

    def start_avoidance(self):
        """Normal engel kaçınma başlat"""
        self.avoiding = True
        self.avoid_phase = 0
        self.avoid_counter = 120
        
        # Akıllı yön seçimi
        if not self.obstacle_left and self.obstacle_right:
            self.turn_direction = 1
        elif not self.obstacle_right and self.obstacle_left:
            self.turn_direction = -1
        else:
            # İkisi de engelli - rastgele seç
            self.turn_direction = random.choice([-1, 1])

    # ================= TAKILMA KONTROLÜ =================
    def check_stuck(self):
        if self.state != 'CLEANING':
            return
        
        try:
            t = self.tf_buffer.lookup_transform(MAP_FRAME, 'base_link', rclpy.time.Time())
            pos = (t.transform.translation.x, t.transform.translation.y)
            
            self.position_history.append(pos)
            if len(self.position_history) > self.max_history:
                self.position_history.pop(0)
            
            # Son 15 pozisyon
            if len(self.position_history) >= 15:
                recent = self.position_history[-15:]
                total_dist = sum(
                    math.hypot(recent[i][0] - recent[i-1][0], recent[i][1] - recent[i-1][1])
                    for i in range(1, len(recent))
                )
                
                # Çok az hareket = TAKILDI
                if total_dist < 0.1:
                    self.stuck_counter += 1
                    if self.stuck_counter > 2:
                        self.get_logger().error('🆘 TAKILDI! KURTARMA!')
                        self.emergency_response()
                        self.stuck_counter = 0
                else:
                    self.stuck_counter = 0
            
        except TransformException:
            pass

    # ================= ANA DÖNGÜ =================
    def loop(self):
        if self.state in ['IDLE', 'PAUSED']:
            return

        # ENGEL KAÇINMA ÖNCELİKLİ
        if self.avoiding:
            self.execute_avoidance()
            return

        if self.state == 'CLEANING':
            if self.wp_idx >= len(self.waypoints):
                self.next_room()
                return
            tx, ty = self.waypoints[self.wp_idx]
            self.go_to(tx, ty)

        elif self.state == 'DOCKING':
            self.go_to(DOCK[0], DOCK[1], docking=True)

    # ================= ENGEL KAÇINMA =================
    def execute_avoidance(self):
        """3 AŞAMALI AGRESİF KAÇINMA"""
        cmd = Twist()
        
        # FAZ 0: GERİ GİT (50 adım)
        if self.avoid_phase == 0:
            if self.avoid_counter > 100:
                cmd.linear.x = -0.2  # Hızlı geri
                cmd.angular.z = 0.0
            else:
                self.avoid_phase = 1
                self.get_logger().info('   → DÖN FAZINA GEÇ')
        
        # FAZ 1: DÖN (60 adım)
        elif self.avoid_phase == 1:
            if self.avoid_counter > 40:
                cmd.linear.x = 0.0
                cmd.angular.z = 1.5 * self.turn_direction  # Hızlı dön
            else:
                self.avoid_phase = 2
                self.get_logger().info('   → İLERLE FAZINA GEÇ')
        
        # FAZ 2: İLERLE (40 adım)
        elif self.avoid_phase == 2:
            if self.avoid_counter > 0:
                cmd.linear.x = 0.15
                cmd.angular.z = 0.3 * self.turn_direction  # Hafif dön
            else:
                # Bitti mi kontrol et
                if not self.obstacle_detected:
                    self.avoiding = False
                    self.avoid_phase = 0
                    self.get_logger().info('✅ ENGEL AŞILDI!')
                else:
                    # Hala engel var - tekrar başla
                    self.avoid_phase = 0
                    self.avoid_counter = 120
                    self.turn_direction *= -1  # Ters yöne dene
                    self.get_logger().warn('🔄 HALA ENGEL VAR - TEKRAR!')
        
        self.cmd_pub.publish(cmd)
        self.avoid_counter -= 1

    # ================= ODA YÖNETİMİ =================
    def start_room(self):
        room = ROOMS[self.room_idx]
        self.generate_zigzag(room)
        self.wp_idx = 0
        self.state = 'CLEANING'
        self.get_logger().info(f'🧹 {room["name"]} ({self.room_idx+1}/{len(ROOMS)})')

    def next_room(self):
        self.room_idx += 1
        if self.room_idx >= len(ROOMS):
            self.state = 'DOCKING'
            self.get_logger().info('✨ TAMAMLANDI! 🔋 Şarja gidiyor...')
        else:
            self.start_room()

    # ================= ZİGZAG YOLU - PLANLANAN YOL =================
    def generate_zigzag(self, r):
        """Zigzag yol oluştur ve PLANLANAN YOL olarak yayınla"""
        self.waypoints.clear()
        y = r['ymin'] + 0.3  # Start slightly inside the room
        direction = 1
        
        # Calculate number of rows based on room height and step size
        num_rows = int((r['ymax'] - r['ymin']) / ZIGZAG_STEP)
        
        for i in range(num_rows):
            if direction == 1:
                # Go from left to right with more waypoints for better visibility
                start_x = r['xmin'] + 0.3
                end_x = r['xmax'] - 0.3
                self.waypoints.append((start_x, y))
                # Add intermediate waypoints for smoother path visualization
                mid_x = (start_x + end_x) / 2
                self.waypoints.append((mid_x, y))
                self.waypoints.append((end_x, y))
            else:
                # Go from right to left with more waypoints for better visibility
                start_x = r['xmax'] - 0.3
                end_x = r['xmin'] + 0.3
                self.waypoints.append((start_x, y))
                # Add intermediate waypoints for smoother path visualization
                mid_x = (start_x + end_x) / 2
                self.waypoints.append((mid_x, y))
                self.waypoints.append((end_x, y))
            
            y += ZIGZAG_STEP
            direction *= -1  # Reverse direction for next row

        # Add extra waypoints at room corners to ensure complete coverage
        # This helps with the transition between different areas of the room
        if len(self.waypoints) > 1:
            # Add corner waypoints to make the pattern more visible in RViz
            self.waypoints.append((r['xmin'] + 0.3, r['ymin'] + 0.3))
            self.waypoints.append((r['xmax'] - 0.3, r['ymax'] - 0.3))
            # Add some extra waypoints to make the zigzag pattern more obvious
            center_x = (r['xmin'] + r['xmax']) / 2
            center_y = (r['ymin'] + r['ymax']) / 2
            self.waypoints.extend([
                (center_x, r['ymin'] + 0.3),
                (center_x, r['ymax'] - 0.3),
                (r['xmin'] + 0.3, center_y),
                (r['xmax'] - 0.3, center_y)
            ])

        # PLANLANAN YOL OLUŞTUR - MAVİ ÇİZGİ
        path = Path()
        path.header.frame_id = MAP_FRAME
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.waypoints:
            p = PoseStamped()
            p.header.frame_id = MAP_FRAME
            p.header.stamp = path.header.stamp
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            # Add orientation to make the path more visible
            p.pose.orientation.w = 1.0
            path.poses.append(p)

        self.planned_path = path
        self.plan_pub.publish(self.planned_path)
        self.get_logger().info(f'📊 Planlanan yol: {len(self.waypoints)} waypoint')
        self.get_logger().info(f'🏠 {r["name"]} için zigzag deseni oluşturuldu')

    # ================= HAREKET =================
    def go_to(self, tx, ty, docking=False):
        try:
            t = self.tf_buffer.lookup_transform(MAP_FRAME, 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

            dx = tx - x
            dy = ty - y
            dist = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - yaw)

            cmd = Twist()
            
            if dist < 0.1:  # Reduced threshold for more precise navigation
                if not docking:
                    self.wp_idx += 1
                    self.get_logger().info(f'🎯 Waypoint reached: ({tx:.2f}, {ty:.2f}) - {len(self.waypoints)-self.wp_idx} left')
                else:
                    self.state = 'IDLE'
                    self.stop_robot()
                    self.get_logger().info('═══════════════════════════════════════')
                    self.get_logger().info('🏁 ŞARJ İSTASYONUNA VARILDI!')
                    self.get_logger().info('✅ TÜM GÖREVLER TAMAMLANDI!')
                    self.get_logger().info('═══════════════════════════════════════')
                return

            # More precise angular control for better zigzag pattern
            if abs(angle_error) > 0.15:  # Reduced threshold for smoother turning
                cmd.angular.z = 1.0 if angle_error > 0 else -1.0
                cmd.linear.x = 0.05  # Small forward movement while turning
            else:
                cmd.linear.x = 0.2  # Increased speed for better movement
                cmd.angular.z = angle_error * 2.0  # Adjusted angular correction

            self.cmd_pub.publish(cmd)

        except TransformException:
            # If transform fails, try to continue with previous command
            cmd = Twist()
            cmd.linear.x = 0.1
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

    # ================= VİZ =================
    def publish_boundary(self):
        if self.state not in ['CLEANING', 'PAUSED']:
            return
        r = ROOMS[min(self.room_idx, len(ROOMS)-1)]
        poly = PolygonStamped()
        poly.header.frame_id = MAP_FRAME
        poly.header.stamp = self.get_clock().now().to_msg()
        for x, y in [(r['xmin'], r['ymin']), (r['xmax'], r['ymin']), 
                     (r['xmax'], r['ymax']), (r['xmin'], r['ymax'])]:
            p = Point32()
            p.x, p.y = x, y
            poly.polygon.points.append(p)
        self.boundary_pub.publish(poly)

    def update_robot_path(self):
        if self.state not in ['CLEANING', 'DOCKING']:
            return
        try:
            t = self.tf_buffer.lookup_transform(MAP_FRAME, 'base_link', rclpy.time.Time())
            p = PoseStamped()
            p.header.frame_id = MAP_FRAME
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = t.transform.translation.x
            p.pose.position.y = t.transform.translation.y
            p.pose.orientation = t.transform.rotation
            self.robot_path.poses.append(p)
            self.robot_path.header.stamp = p.header.stamp
            self.robot_path_pub.publish(self.robot_path)
        except TransformException:
            pass

    def publish_current_goal(self):
        if self.state != 'CLEANING' or self.wp_idx >= len(self.waypoints):
            return
        tx, ty = self.waypoints[self.wp_idx]
        m = Marker()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "goal"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = tx, ty, 0.3
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color.r = m.color.g = 1.0
        m.color.a = 0.8
        self.current_goal_pub.publish(m)

    def publish_waypoint_markers(self):
        if self.state != 'CLEANING' or not self.waypoints:
            return
        ma = MarkerArray()
        for i, (x, y) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = MAP_FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "wp"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.1
            m.scale.x = m.scale.y = m.scale.z = 0.15
            if i < self.wp_idx:
                m.color.r = m.color.g = m.color.b = 0.5
                m.color.a = 0.3
            else:
                m.color.r = m.color.g = m.color.b = 1.0
                m.color.a = 0.5
            ma.markers.append(m)
        self.waypoint_markers_pub.publish(ma)

    def status_update(self):
        if self.state == 'CLEANING':
            room = ROOMS[self.room_idx]
            prog = (self.wp_idx / len(self.waypoints)) * 100 if self.waypoints else 0
            st = f'📊 {room["name"]}: {prog:.0f}% | WP:{self.wp_idx}/{len(self.waypoints)} | '
            st += f'Min:{self.min_distance:.2f}m | Yol:{len(self.robot_path.poses)}'
            if self.avoiding:
                st += f' | 🚨 KAÇIYOR (Faz {self.avoid_phase})'
            self.get_logger().info(st)

    def normalize_angle(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = ZigZagCleaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Bye!')
    finally:
        node.stop_robot()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
