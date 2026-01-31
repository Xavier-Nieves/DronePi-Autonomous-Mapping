#!/bin/bash

# Unitree L1 LiDAR Docker Helper Script

set -e

echo "Unitree L1 4D LiDAR Docker Helper"
echo "=================================="
echo ""

# Function to check if Docker is running
check_docker() {
    if ! docker info > /dev/null 2>&1; then
        echo "Error: Docker is not running. Please start Docker first."
        exit 1
    fi
}

# Function to enable X11 forwarding
enable_x11() {
    echo "Enabling X11 forwarding for GUI..."
    xhost +local:docker > /dev/null 2>&1
    echo "✓ X11 forwarding enabled"
}

# Function to find LiDAR device
find_device() {
    echo ""
    echo "Searching for LiDAR device..."
    if ls /dev/ttyUSB* > /dev/null 2>&1; then
        echo "Found USB devices:"
        ls -l /dev/ttyUSB*
    else
        echo "⚠ No USB serial devices found. Please check your connection."
    fi
    echo ""
}

# Main menu
show_menu() {
    echo "Choose an option:"
    echo "1) Build ROS Noetic (ROS1) image"
    echo "2) Build ROS2 Humble image (Point-LIO setup - RECOMMENDED)"
    echo "3) Run ROS Noetic container"
    echo "4) Run ROS2 Humble container (Point-LIO setup - RECOMMENDED)"
    echo "5) Find LiDAR device"
    echo "6) Stop all containers"
    echo "7) Clean up (remove containers and images)"
    echo "8) Exit"
    echo ""
    read -p "Enter your choice [1-8]: " choice
}

# Handle menu choices
handle_choice() {
    case $choice in
        1)
            echo "Building ROS Noetic image..."
            docker-compose build unitree-l1-ros1
            echo "✓ Build complete!"
            ;;
        2)
            echo "Building ROS2 Humble image (Point-LIO setup)..."
            docker-compose build unitree-l1-ros2
            echo "✓ Build complete!"
            ;;
        3)
            enable_x11
            echo "Starting ROS Noetic container..."
            docker-compose run --rm unitree-l1-ros1
            ;;
        4)
            enable_x11
            echo "Starting ROS2 Humble container (Point-LIO setup)..."
            docker-compose run --rm unitree-l1-ros2
            ;;
        5)
            find_device
            ;;
        6)
            echo "Stopping all containers..."
            docker-compose down
            echo "✓ All containers stopped"
            ;;
        7)
            read -p "This will remove all containers and images. Continue? (y/N): " confirm
            if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
                echo "Cleaning up..."
                docker-compose down
                docker rmi unitree-l1-lidar:ros1 unitree-l1-lidar:ros2-humble 2>/dev/null || true
                echo "✓ Cleanup complete!"
            else
                echo "Cancelled."
            fi
            ;;
        8)
            echo "Goodbye!"
            exit 0
            ;;
        *)
            echo "Invalid choice. Please try again."
            ;;
    esac
}

# Main script
check_docker
find_device

while true; do
    echo ""
    show_menu
    handle_choice
    echo ""
    read -p "Press Enter to continue..."
    clear
done
