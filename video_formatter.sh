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

    ffprobe_output=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 "$video_file" 2>&1)
    width=$(echo "$ffprobe_output" | cut -d 'x' -f 1)
    height=$(echo "$ffprobe_output" | cut -d 'x' -f 2)
    echo "Video resolution: $width x $height"
    if ( [ "$height" -eq 1080 ] ); then
        newHeight=270
    else
        newHeight=360
    fi
    echo "New resolution: 480 x $newHeight"
    # Create folder for frames
    mkdir -p "$frames_folder/$video_name/frames"

    ffmpeg -i "$video_file" -r $output_frame_rate -start_number 0 -vf "scale=480:$newHeight" "$frames_folder/$video_name/frames/%06d.png"



    frame_count=$(ls -1 "$frames_folder/$video_name/frames" | wc -l)


    for (( i=1; i<=$frame_count; i++ ))
    do
        timestamp=$(echo "scale=5; $i/30" | bc)
        echo "$timestamp" >> "$frames_folder/$video_name/times.txt"
    done

    fi
    
done
