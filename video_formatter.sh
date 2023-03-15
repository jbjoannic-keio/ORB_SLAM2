#!/bin/bash

if [ $# -ne 2 ]; then
    echo "Usage: $0 <folder containing videos> <output folder for frames>"
    exit 1
fi

videos_folder="$1"
frames_folder="$2"
output_frame_rate=30

for video_file in "$videos_folder"/*; do


    if [ -d "$video_file" ]; then
    echo "The file is a directory."
    else
    echo "Processing $video_file"



    # Extract video filename without extension
    video_name=$(basename -- "$video_file")
    video_name="${video_name%.*}"
    # Create folder for frames
    mkdir -p "$frames_folder/$video_name/frames"

    # Create time.txt file
    ffmpeg -i "$video_file" -r $output_frame_rate -vf "scale=480:270" "$frames_folder/$video_name/frames/%04d.png"



    frame_count=$(ls -1 "$frames_folder/$video_name/frames" | wc -l)


    for (( i=1; i<=$frame_count; i++ ))
    do
        timestamp=$(echo "scale=5; $i/30" | bc)
        echo "$timestamp" >> "$frames_folder/$video_name/times.txt"
    done

    fi
    
done
