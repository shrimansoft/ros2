#!/bin/bash

# Bag utilities script for managing ROS2 bag files
# Provides commands for listing, copying, cleaning bag files

set -e

DATA_DIR="./rosbag_annotator/data"

# Ensure data directory exists
mkdir -p "$DATA_DIR"

# Function to display help
show_help() {
    echo "🗂️  ROS2 Bag Utilities"
    echo "====================="
    echo ""
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  list              - List all bag files in data directory"
    echo "  info <bag_name>   - Show information about a specific bag"
    echo "  copy <source>     - Copy a bag file to data directory"
    echo "  clean             - Remove all bag files (with confirmation)"
    echo "  size              - Show data directory size"
    echo ""
    echo "Examples:"
    echo "  $0 list"
    echo "  $0 info my_bag"
    echo "  $0 copy /path/to/bag_file"
    echo "  $0 clean"
}

# Function to list bags
list_bags() {
    echo "📁 Available bag files in $DATA_DIR:"
    echo "====================================="
    
    if [ ! -d "$DATA_DIR" ] || [ -z "$(ls -A "$DATA_DIR" 2>/dev/null)" ]; then
        echo "No bag files found."
        echo ""
        echo "To add bag files:"
        echo "  - Copy them to: $DATA_DIR"
        echo "  - Use: $0 copy /path/to/your/bag"
        return
    fi
    
    echo ""
    find "$DATA_DIR" -name "*.db3" -o -name "metadata.yaml" | sort | while read -r file; do
        if [[ "$file" == *.db3 ]]; then
            size=$(du -h "$file" 2>/dev/null | cut -f1)
            echo "📊 $(basename "$(dirname "$file")"): $size"
        fi
    done
    
    echo ""
    total_size=$(du -sh "$DATA_DIR" 2>/dev/null | cut -f1)
    total_files=$(find "$DATA_DIR" -type f | wc -l)
    echo "Total: $total_size in $total_files files"
}

# Function to show bag info
show_bag_info() {
    local bag_name="$1"
    
    if [ -z "$bag_name" ]; then
        echo "❌ Please specify a bag name"
        echo "Usage: $0 info <bag_name>"
        return 1
    fi
    
    local bag_path="$DATA_DIR/$bag_name"
    
    if [ ! -d "$bag_path" ]; then
        echo "❌ Bag '$bag_name' not found in $DATA_DIR"
        echo "Available bags:"
        list_bags
        return 1
    fi
    
    echo "📊 Bag Information: $bag_name"
    echo "==============================="
    
    if [ -f "$bag_path/metadata.yaml" ]; then
        echo "Metadata file: ✅ Found"
        echo ""
        echo "Bag details:"
        cat "$bag_path/metadata.yaml" | grep -E "(bag_size|duration|message_count|compression)" || echo "Basic metadata available"
    else
        echo "Metadata file: ❌ Not found"
    fi
    
    echo ""
    echo "Files:"
    find "$bag_path" -type f -exec ls -lh {} \; | awk '{print $5 "\t" $9}'
}

# Function to copy bag
copy_bag() {
    local source="$1"
    
    if [ -z "$source" ]; then
        echo "❌ Please specify source path"
        echo "Usage: $0 copy <source_path>"
        return 1
    fi
    
    if [ ! -e "$source" ]; then
        echo "❌ Source '$source' does not exist"
        return 1
    fi
    
    local basename_src=$(basename "$source")
    local dest="$DATA_DIR/$basename_src"
    
    echo "📥 Copying bag file..."
    echo "From: $source"
    echo "To:   $dest"
    
    if [ -d "$source" ]; then
        cp -r "$source" "$dest"
    else
        mkdir -p "$dest"
        cp "$source" "$dest/"
    fi
    
    echo "✅ Copy completed!"
    echo ""
    echo "Bag is now available as: $basename_src"
}

# Function to clean bags
clean_bags() {
    if [ ! -d "$DATA_DIR" ] || [ -z "$(ls -A "$DATA_DIR" 2>/dev/null)" ]; then
        echo "📁 Data directory is already empty"
        return
    fi
    
    echo "🗑️  This will permanently delete ALL bag files in:"
    echo "   $DATA_DIR"
    echo ""
    list_bags
    echo ""
    read -p "Are you absolutely sure? Type 'DELETE' to confirm: " confirm
    
    if [ "$confirm" = "DELETE" ]; then
        rm -rf "$DATA_DIR"/*
        echo "✅ All bag files have been deleted"
        mkdir -p "$DATA_DIR"
    else
        echo "❌ Cancelled - no files were deleted"
    fi
}

# Function to show size
show_size() {
    echo "📏 Data Directory Size"
    echo "====================="
    echo ""
    
    if [ -d "$DATA_DIR" ]; then
        du -sh "$DATA_DIR"
        echo ""
        echo "Breakdown:"
        find "$DATA_DIR" -maxdepth 1 -type d ! -path "$DATA_DIR" -exec du -sh {} \; 2>/dev/null | sort -hr
    else
        echo "Data directory does not exist yet"
    fi
}

# Main script logic
case "$1" in
    "list"|"")
        list_bags
        ;;
    "info")
        show_bag_info "$2"
        ;;
    "copy")
        copy_bag "$2"
        ;;
    "clean")
        clean_bags
        ;;
    "size")
        show_size
        ;;
    "help"|"-h"|"--help")
        show_help
        ;;
    *)
        echo "❌ Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
