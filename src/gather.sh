#!/bin/sh

# This script first lists the entire directory hierarchy and then
# concatenates the contents of all files into a single file.

# The name of the output file.
OUTPUT_FILE="codebase_snapshot.txt"

# Clear the output file if it already exists to prevent appending.
> "$OUTPUT_FILE"

# --- Section 1: Directory Hierarchy ---
# Find all files and directories in the current directory and list them.
# The output is redirected (>>) into the specified output file.
echo "==================== DIRECTORY HIERARCHY ====================" >> "$OUTPUT_FILE"
find . >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# --- Section 2: File Contents ---
# Find all regular files (-type f) in the current directory (.) and
# its subdirectories.
# Use 'while read -r' to process each file path safely.
# For each file, a separator and the file path are added before its content.
echo "==================== FILE CONTENTS ====================" >> "$OUTPUT_FILE"
find . -type f | while read -r file; do
    echo "-------------------- FILE: $file --------------------" >> "$OUTPUT_FILE"
    cat "$file" >> "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE" # Add a newline for separation
done

echo "Successfully gathered all files and directory hierarchy into $OUTPUT_FILE"
