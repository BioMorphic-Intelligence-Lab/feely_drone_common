#!/bin/bash

# Batch xacro processing script for different cylinder radii
# Converts xacro files with varying cylinder radius from 0.01 to 0.5 in 0.01 steps

# Configuration
INPUT_XACRO="cylinder.xacro"  # Change this to your input xacro file
OUTPUT_DIR="./cylinders"  # Output directory for generated URDF files
START_RADIUS=0.005
END_RADIUS=0.5
STEP=0.005

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if input file exists
if [ ! -f "$INPUT_XACRO" ]; then
    print_error "Input xacro file '$INPUT_XACRO' not found!"
    print_error "Please set the INPUT_XACRO variable to point to your xacro file."
    exit 1
fi

# Check if xacro is installed
if ! command -v xacro &> /dev/null; then
    print_error "xacro command not found! Please install ros-$ROS_DISTRO-xacro"
    exit 1
fi

# Create output directory if it doesn't exist
if [ ! -d "$OUTPUT_DIR" ]; then
    print_status "Creating output directory: $OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR"
fi

# Initialize counters
total_files=0
successful_conversions=0
failed_conversions=0

print_status "Starting batch xacro conversion..."
print_status "Input file: $INPUT_XACRO"
print_status "Output directory: $OUTPUT_DIR"
print_status "Radius range: $START_RADIUS to $END_RADIUS (step: $STEP)"
echo

# Calculate total number of files for progress tracking
total_expected=$(echo "($END_RADIUS - $START_RADIUS) / $STEP + 1" | bc -l | cut -d. -f1)
print_status "Expected to generate $total_expected files"
echo

# Main conversion loop using bc for floating point arithmetic
radius=$START_RADIUS
while (( $(echo "$radius <= $END_RADIUS" | bc -l) )); do
    # Format radius to avoid scientific notation and ensure consistent naming
    radius_formatted=$(printf "%.3f" $radius)
    
    # Generate output filename
    output_file="$OUTPUT_DIR/cylinder_${radius_formatted}.urdf"
    
    # Progress indicator
    total_files=$((total_files + 1))
    printf "\r${GREEN}[PROGRESS]${NC} Processing radius %.2f (file %d/%d)..." $radius $total_files $total_expected
    
    # Run xacro conversion
    if xacro "$INPUT_XACRO" cylinder_radius:=$radius base_path:="../" > "$output_file" 2>/dev/null; then
        successful_conversions=$((successful_conversions + 1))
    else
        print_error "\nFailed to convert for radius $radius_formatted"
        failed_conversions=$((failed_conversions + 1))
        # Remove failed file if it exists
        [ -f "$output_file" ] && rm "$output_file"
    fi
    
    # Increment radius
    radius=$(echo "$radius + $STEP" | bc -l)
done

echo # New line after progress indicator

# Print summary
echo
print_status "Batch conversion completed!"
echo "----------------------------------------"
echo "Total files processed: $total_files"
echo "Successful conversions: $successful_conversions"
echo "Failed conversions: $failed_conversions"
echo "Output directory: $OUTPUT_DIR"

if [ $failed_conversions -gt 0 ]; then
    print_warning "Some conversions failed. Check your xacro file for errors."
    exit 1
else
    print_status "All conversions completed successfully!"
    echo
    print_status "Generated files:"
    ls -la "$OUTPUT_DIR"/*.urdf | head -5
    if [ $successful_conversions -gt 5 ]; then
        echo "... and $((successful_conversions - 5)) more files"
    fi
fi

# Optional: Show file sizes
echo
print_status "File size summary:"
du -sh "$OUTPUT_DIR"

exit 0