#!/bin/bash

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <file1> <file2> <output_file>"
    exit 1
fi

file1=$1
file2=$2
output_file=$3

# Check if both files exist
if [ ! -f "$file1" ]; then
    echo "File $file1 does not exist."
    exit 1
fi

if [ ! -f "$file2" ]; then
    echo "File $file2 does not exist."
    exit 1
fi

# Read both files line by line and prepend lines from file1 to file2
paste -d ' ' "$file1" "$file2" > "$output_file"

echo "Output written to $output_file"