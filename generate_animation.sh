#!/bin/bash
rm 'input.txt'; touch input.txt; for x in $(ls iteration_f*.png); do echo "file " $x >> input.txt && echo "duration 0.03333333333333333" >> input.txt; done
ffmpeg -f concat -safe 0 -i input.txt -r 30 -y b_output.mkv
