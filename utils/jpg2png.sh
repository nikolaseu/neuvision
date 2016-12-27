#!/bin/bash
for jpg; do                                  # use $jpg in place of each filename given, in turn
    png="${jpg%.jpg}.png"                    # construct the PNG version of the filename by replacing .jpg with .png
    echo converting "$jpg" ...               # output status info to the user running the script
    #if convert "$jpg" jpg.to.png ; then      # use the convert program (common in Linux) to create the PNG in a temp file
    if sips -s format png "$jpg" --out jpg.to.png ; then
        mv jpg.to.png "$png"                 # if it worked, rename the temporary PNG image to the correct name
    else                                     # ...otherwise complain and exit from the script
        echo 'jpg2png: error: failed output saved in "jpg.to.png".' >&2
        exit 1
    fi                                       # the end of the "if" test construct
done                                         # the end of the "for" loop
echo all conversions successful              # tell the user the good news
exit 0