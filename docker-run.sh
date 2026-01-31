#!/bin/bash
# Wrapper script to manage LiDAR and MAVROS Docker containers
# Usage: ./docker-run.sh [container] [command]
# Container: lidar, mavros, or all

set -e

CONTAINER="${1:-all}"
COMMAND="${2:-help}"

cd "$(dirname "$0")/docker"

# Map container names to docker-compose service names
get_service_name() {
    case "$1" in
        lidar) echo "lidar" ;;
        mavros) echo "mavros" ;;
        all) echo "" ;;  # Empty means all services
        *)
            echo "ERROR: Invalid container name: $1"
            echo "Valid containers: lidar, mavros, all"
            exit 1
            ;;
    esac
}

# Map container names to actual container names
get_container_name() {
    case "$1" in
        lidar) echo "lidar_container" ;;
        mavros) echo "mavros_container" ;;
        *)
            echo "ERROR: Cannot get container name for: $1"
            exit 1
            ;;
    esac
}

SERVICE=$(get_service_name "$CONTAINER")

case "$COMMAND" in
    build)
        if [ "$CONTAINER" = "all" ]; then
            echo "Building all Docker images..."
            docker-compose build
        else
            echo "Building $CONTAINER Docker image..."
            docker-compose build "$SERVICE"
        fi
        ;;
    up|start)
        echo "Starting $CONTAINER container(s)..."
        xhost +local:docker 2>/dev/null || true
        if [ "$CONTAINER" = "all" ]; then
            docker-compose up -d
            echo "All containers started."
            echo "Access LiDAR: ./docker-run.sh lidar shell"
            echo "Access MAVROS: ./docker-run.sh mavros shell"
        else
            docker-compose up -d "$SERVICE"
            CONTAINER_NAME=$(get_container_name "$CONTAINER")
            echo "$CONTAINER container started. Access with: ./docker-run.sh $CONTAINER shell"
        fi
        ;;
    shell|bash)
        if [ "$CONTAINER" = "all" ]; then
            echo "ERROR: Cannot open shell for 'all'. Specify: lidar or mavros"
            exit 1
        fi
        CONTAINER_NAME=$(get_container_name "$CONTAINER")
        echo "Connecting to $CONTAINER container..."
        docker exec -it "$CONTAINER_NAME" bash
        ;;
    stop)
        if [ "$CONTAINER" = "all" ]; then
            echo "Stopping all containers..."
            docker-compose down
        else
            echo "Stopping $CONTAINER container..."
            docker-compose stop "$SERVICE"
        fi
        ;;
    clean)
        if [ "$CONTAINER" = "all" ]; then
            echo "Removing all containers and images..."
            docker-compose down
            docker rmi unitree-l1-lidar:ros2-humble 2>/dev/null || true
            docker rmi mavros-px4:ros2-humble 2>/dev/null || true
        else
            echo "Removing $CONTAINER container..."
            docker-compose stop "$SERVICE"
            docker-compose rm -f "$SERVICE"
            if [ "$CONTAINER" = "lidar" ]; then
                docker rmi unitree-l1-lidar:ros2-humble 2>/dev/null || true
            elif [ "$CONTAINER" = "mavros" ]; then
                docker rmi mavros-px4:ros2-humble 2>/dev/null || true
            fi
        fi
        ;;
    logs)
        if [ "$CONTAINER" = "all" ]; then
            docker-compose logs -f
        else
            docker-compose logs -f "$SERVICE"
        fi
        ;;
    help|*)
        echo "Multi-Container Docker Manager for Drone Mapping System"
        echo ""
        echo "Usage: ./docker-run.sh [container] [command]"
        echo ""
        echo "Containers:"
        echo "  lidar   - LiDAR SLAM container (Unitree L1, Point-LIO)"
        echo "  mavros  - MAVROS/PX4 container (flight controller)"
        echo "  all     - All containers (default)"
        echo ""
        echo "Commands:"
        echo "  build   - Build Docker image(s)"
        echo "  start   - Start container(s) in background"
        echo "  shell   - Open bash shell (requires specific container)"
        echo "  stop    - Stop container(s)"
        echo "  clean   - Remove container(s) and image(s)"
        echo "  logs    - Show container logs"
        echo "  help    - Show this help message"
        echo ""
        echo "Examples:"
        echo "  ./docker-run.sh all build        # Build both containers"
        echo "  ./docker-run.sh lidar start      # Start only LiDAR container"
        echo "  ./docker-run.sh mavros shell     # Open shell in MAVROS container"
        echo "  ./docker-run.sh all stop         # Stop all containers"
        echo ""
        exit 0
        ;;
esac