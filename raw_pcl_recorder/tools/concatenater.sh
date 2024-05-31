#!/bin/bash

# Get a list of PCD files in the data directory
pcd_files=(data/*.pcd)

# Check if there are any PCD files
if [ ${#pcd_files[@]} -eq 0 ]; then
    echo "No PCD files found in the data directory."
    exit 1
fi

# Calculate the number of splits
num_splits=4

# Calculate the number of files per split
files_per_split=$(( (${#pcd_files[@]} + num_splits - 1) / num_splits ))

echo "Total number of PCD files: ${#pcd_files[@]}"
echo "Number of splits: $num_splits"
echo "Number of files per split: $files_per_split"

# Iterate over the splits
for (( i = 0; i < num_splits; i++ )); do
    start=$(( i * files_per_split ))
    end=$(( (i + 1) * files_per_split ))

    # Check if end index is greater than the number of files
    if [ $end -gt ${#pcd_files[@]} ]; then
        end=${#pcd_files[@]}
    fi

    echo "Processing split $((i + 1)) of $num_splits (files $start to $((end - 1)))"

    # Concatenate the split range of files
    if [ $start -lt $end ]; then
        pcl_concatenate_points_pcd "${pcd_files[@]:$start:$((end - start))}" > testo.txt
        # Rename the output file
        output=concatenated_split_$i.pcd
        echo "Output: $output"
        if [ -f "output.pcd" ]; then
            mv "output.pcd" "$output"
            echo "Created $output"
        else
            echo "Error: output.pcd not found after concatenation"
            exit 1
        fi
        # echo "Created concatenated_split_$i.pcd with files ${pcd_files[@]:$start:$((end - start))}"
        echo "Created concatenated_split_$i.pcd with files}"
    else
        echo "No files to process in this split."
    fi
done

echo "Finished processing all splits."
