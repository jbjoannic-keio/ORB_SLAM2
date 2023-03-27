#!/bin/bash

# Check if the folder argument is provided
if [ -z "$1" ]
then
  echo "Usage: $0 <folder>"
  exit 1
fi

# Loop through all PNG files in the folder
for file in "$1"/*.png
do
  # Check if the file exists and is a regular file
  if [ -f "$file" ]
  then
    # Get the filename without the extension
    filename=$(basename "$file" .png)
    
    # Convert the filename to decimal explicitly
    number=$(printf "%d" "$filename")
    
    # Pad the filename with leading zeros to make it 6 digits
    padded_filename=$(printf "%06d" "$number")
    
    # Rename the file
    mv "$file" "$1/$padded_filename.png"
  fi
done

echo "Done."
