#!/bin/bash

# Define the time intervals in seconds as arrays
interval_starts_1=(28 90 134 225 600 789 816 884 945)
interval_ends_1=(58 99 180 595 740 796 871 911 1056) 

immobile_interval_starts_1=(45 78 86 100 133 186 212 229 259 345 461 495 828 874) #in frames (1350 2340 2580 3000 3990 5580 6360 6870 7770 10350 13830 14850 24840 26220)
immobile_interval_ends_1=(57 84 96 125 167 198 222 239 277 390 475 510 855 885) #in frames (1710 2520 2880 3750 5010 5940 6660 7170 8310 11700 14250 15300 25650 26550 )

interval_starts_2=(334 378 437 685 915 1877 1914 2216) 
interval_ends_2=(367 405 624 763 962 1890 2053 2232) 
sampleNumber="1"

# Input and output file names
input_file="videos/Lab/LapC${sampleNumber}/times.txt"
output_file="videos/Lab/LapC${sampleNumber}/isHorizontal.txt"

# Check if input file exists
if [ ! -f "$input_file" ]; then
    echo "Input file not found."
    exit 1
fi

# Check if output file exists
if [ -f "$output_file" ]; then
    echo "Output file already exists. Overwrite"
    rm "$output_file"
fi

# Function to check if a time falls within any interval
check_interval() {
    local time=$1

    if [[ $sampleNumber == "1" ]]; then

        local start_intervals=("${immobile_interval_starts_1[@]}")
        local end_intervals=("${immobile_interval_ends_1[@]}")
    else
        local start_intervals=("${interval_starts_2[@]}")
        local end_intervals=("${interval_ends_2[@]}")
    fi

    format_time() {
        local decimal_time=$1
        if (( $(awk -v t="$decimal_time" 'BEGIN {print (t < 1) ? 1 : 0}') == 1 )); then
            echo "0$decimal_time"
        else
            echo "$decimal_time"
        fi
    }

    time=$(format_time "$time")

    for ((i = 0; i < ${#start_intervals[@]}; i++)); do
        if (( $(awk -v t="$time" -v start="${start_intervals[i]}" -v end="${end_intervals[i]}" 'BEGIN {print (t >= start && t <= end) ? 0 : 1}') == 0 )); then
            echo "1"
            return
        fi
    done

    echo "0"
}

# Process input file line by line
while IFS= read -r line; do
    echo "Processing time $line..."

    result=$(check_interval "$line")
    echo "Result: $result"
    echo "$result" >> "$output_file"
done < "$input_file"

echo "Processing complete. Results written to $output_file."