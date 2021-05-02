# ImageProcessing

cd build
cmake --build . --config Release

To rename all calibration files:
ls -v | cat -n | while read n f; do mv -n "$f" "$n.jpg"; done 
