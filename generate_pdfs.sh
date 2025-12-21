#!/bin/bash

# Script to generate PDFs from Markdown files

echo "Generating PDFs..."

# Check if pandoc is installed
if ! command -v pandoc &> /dev/null
then
    echo "pandoc could not be found, installing..."
    sudo apt update
    sudo apt install -y pandoc texlive-latex-base texlive-fonts-recommended texlive-fonts-extra
fi

# Generate Project Report PDF
pandoc PROJECT_REPORT.md -o PROJECT_REPORT.pdf \
    --pdf-engine=pdflatex \
    -V geometry:margin=1in \
    -V fontsize=12pt \
    --toc \
    --highlight-style=tango

# Generate Presentation PDF
pandoc PRESENTATION.md -o PRESENTATION.pdf \
    --pdf-engine=pdflatex \
    -V geometry:margin=1in \
    -V fontsize=12pt \
    --highlight-style=tango

echo "PDF generation complete!"
echo "Generated files:"
echo "  - PROJECT_REPORT.pdf"
echo "  - PRESENTATION.pdf"