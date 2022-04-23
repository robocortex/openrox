#!/bin/sh

echo "Compiling OPENROX user manual"
pdflatex openrox_user_manual
bibtex   openrox_user_manual
pdflatex openrox_user_manual
pdflatex openrox_user_manual

