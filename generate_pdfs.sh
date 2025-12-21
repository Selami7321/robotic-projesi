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

# Try to generate PDFs with LaTeX first
if command -v pdflatex &> /dev/null
then
    echo "Generating PDFs with LaTeX support..."
    # Generate Project Report PDF
    pandoc PROJECT_REPORT.md -o PROJECT_REPORT.pdf \
        --pdf-engine=pdflatex \
        -V geometry:margin=1in \
        -V fontsize=12pt \
        --toc \
        --highlight-style=tango || echo "Failed to generate PROJECT_REPORT.pdf with LaTeX"

    # Generate Presentation PDF
    pandoc PRESENTATION.md -o PRESENTATION.pdf \
        --pdf-engine=pdflatex \
        -V geometry:margin=1in \
        -V fontsize=12pt \
        --highlight-style=tango || echo "Failed to generate PRESENTATION.pdf with LaTeX"
else
    echo "LaTeX not found, generating HTML versions instead..."
    # Generate HTML versions as fallback
    pandoc PROJECT_REPORT.md -o PROJECT_REPORT.html \
        -s --toc \
        --css=https://cdn.jsdelivr.net/npm/water.css@2/out/light.css || echo "Failed to generate PROJECT_REPORT.html"
    
    pandoc PRESENTATION.md -o PRESENTATION.html \
        -s --css=https://cdn.jsdelivr.net/npm/water.css@2/out/light.css || echo "Failed to generate PRESENTATION.html"
fi

echo "PDF/HTML generation complete!"
echo "Generated files:"
ls -la PROJECT_REPORT.* PRESENTATION.* 2>/dev/null || echo "No files generated"