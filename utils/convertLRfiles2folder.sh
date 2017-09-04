#!/bin/bash
# move L{number}.png files to L/{number}.png
mkdir L
for file in L*.png
do
    if [[ -f $file ]]; then
        newFile="${file/L/}"
        echo moving $file to L/$newFile
        mv $file L/$newFile
    fi
done
# move R{number}.png files to R/{number}.png
mkdir R
for file in R*.png
do
    if [[ -f $file ]]; then
        newFile="${file/R/}"
        echo moving $file to R/$newFile
        mv $file R/$newFile
    fi
done
