#!/bin/bash
echo "Creating doccumentation"
(cd ..; doxygen Doxyfile)

ln -s html/index.html index.html
echo "Documentation generation is complete!"
