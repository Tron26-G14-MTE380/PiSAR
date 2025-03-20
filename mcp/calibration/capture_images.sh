#!/bin/bash

# Check if resolution is provided
if [ -z "$1" ]; then
    echo "Usage: $0 WIDTHxHEIGHT"
    exit 1
fi

# Extract width and height from argument
resolution="$1"
width=${resolution%x*}
height=${resolution#*x}

# Define output directory
output_dir="captured_images"
mkdir -p "$output_dir"

# Get the next available index
index=$(ls "$output_dir" | grep -oP '(?<=image_)\d+' | sort -nr | head -n1)
index=$((index + 1))

echo "Using resolution: ${width}x${height}"
echo "Press 's' to capture an image, 'q' to quit."

# Start libcamera-still with proper preview resolution
while true; do
    libcamera-still --width "$width" --height "$height" --preview 0,0,640,480 --timeout 0 --keypress
    
    filename="$output_dir/image_${index}_${width}x${height}.jpg"
    libcamera-still --width "$width" --height "$height" --nopreview -o "$filename"
    echo "Saved: $filename"
    index=$((index + 1))  # Increment index for next capture
done
