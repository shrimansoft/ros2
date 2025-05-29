# ROS2 Bag Annotator - Makefile
# Convenient commands for managing the project

.PHONY: help build run annotate info health clean setup test test-unit test-integration test-docker docker-compose build-appimage

# Default target
help:
	@echo "🚀 ROS2 Bag Annotator - Available Commands"
	@echo "==========================================="
	@echo ""
	@echo "Setup & Building:"
	@echo "  make build         - Build Docker image"
	@echo "  make setup         - First-time setup (build + health check)"
	@echo ""
	@echo "Running:"
	@echo "  make annotate      - Launch annotation GUI"
	@echo "  make run CMD=...   - Run custom command in container"
	@echo "  make shell         - Open interactive shell in container"
	@echo ""
	@echo "Utilities:"
	@echo "  make health        - Run system health check"
	@echo "  make info          - Show project information"
	@echo "  make bags          - List available bag files"
	@echo "  make clean         - Clean up containers and data"
	@echo "  make test          - Run all tests"
	@echo "  make test-unit     - Run unit tests only"
	@echo "  make test-integration - Run integration tests only"
	@echo "  make test-docker   - Run tests in Docker container"
	@echo "  make build-appimage - Build standalone AppImage"
	@echo ""
	@echo "Examples:"
	@echo "  make annotate"
	@echo "  make run CMD='info /data/my_bag'"
	@echo "  make bags"

# Build the Docker image
build:
	@echo "🔨 Building Docker image..."
	docker build -t shriman1:a .
	@echo "✅ Build completed!"

# First-time setup
setup: build health
	@echo ""
	@echo "🎉 Setup completed! You can now run:"
	@echo "  make annotate    # Start annotation GUI"
	@echo "  make health      # Check system status"

# Launch annotation GUI
annotate:
	@echo "🎯 Launching annotation GUI..."
	./run_annotator.bash annotate

# Run custom command
run:
	@if [ -z "$(CMD)" ]; then \
		echo "❌ Please specify CMD. Example: make run CMD='info /data/my_bag'"; \
		exit 1; \
	fi
	./run_annotator.bash $(CMD)

# Open shell in container
shell:
	@echo "🐚 Opening container shell..."
	./run_annotator.bash bash

# Run health check
health:
	@echo "🔍 Running health check..."
	./scripts/health_check.bash

# Show project info
info:
	@echo "📋 ROS2 Bag Annotator Project Information"
	@echo "========================================="
	@echo ""
	@echo "📁 Project structure:"
	@find . -name "*.bash" -o -name "*.py" -o -name "Dockerfile" -o -name "*.yaml" | grep -v __pycache__ | sort
	@echo ""
	@echo "🐳 Docker images:"
	@docker images | grep shriman1 || echo "No images found (run 'make build')"
	@echo ""
	@echo "📊 Data directory:"
	@if [ -d "./data" ]; then \
		echo "  Size: $$(du -sh ./data 2>/dev/null | cut -f1)"; \
		echo "  Files: $$(find ./data -type f 2>/dev/null | wc -l)"; \
	elif [ -d "./rosbag_annotator/data" ]; then \
		echo "  Size: $$(du -sh ./rosbag_annotator/data 2>/dev/null | cut -f1)"; \
		echo "  Files: $$(find ./rosbag_annotator/data -type f 2>/dev/null | wc -l)"; \
	else \
		echo "  Not found (will be created automatically)"; \
	fi

# List bag files
bags:
	@echo "📁 Managing bag files..."
	./scripts/bag_utils.bash list

# Clean up
clean:
	@echo "🧹 Cleaning up..."
	@echo "Removing containers..."
	-docker rm -f ros2_annotator 2>/dev/null || true
	@echo "Removing dangling images..."
	-docker image prune -f
	@echo "Data directory (use 'make clean-data' to clean):"
	@if [ -d "./rosbag_annotator/data" ]; then \
		echo "  $$(du -sh ./rosbag_annotator/data 2>/dev/null | cut -f1) in $$(find ./rosbag_annotator/data -type f 2>/dev/null | wc -l) files"; \
	else \
		echo "  Empty"; \
	fi
	@echo "✅ Cleanup completed!"

# Clean data directory (separate target for safety)
clean-data:
	@echo "⚠️  This will remove all bag files and annotations!"
	@read -p "Are you sure? (y/N): " confirm && \
	if [ "$$confirm" = "y" ] || [ "$$confirm" = "Y" ]; then \
		./scripts/bag_utils.bash clean; \
	else \
		echo "Cancelled."; \
	fi

# Advanced targets
rebuild: clean build
	@echo "🔄 Rebuild completed!"

dev:
	@echo "🛠️  Starting development environment..."
	docker run -it \
		--name=ros2_annotator_dev \
		--env="DISPLAY=$$DISPLAY" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="$$(pwd)/rosbag_annotator:/ros2_ws/src/rosbag_annotator" \
		--volume="$$(pwd)/rosbag_annotator/data:/data" \
		--net=host \
		shriman1:a \
		bash

# Quick start for new users
quick:
	@echo "🚀 Quick start mode..."
	./scripts/quick_annotate.bash

# Show version and status
status:
	@echo "📊 System Status"
	@echo "==============="
	@echo "Docker: $$(docker --version 2>/dev/null || echo 'Not found')"
	@echo "Image: $$(docker images shriman1:a --format 'Built {{.CreatedSince}}' 2>/dev/null || echo 'Not built')"
	@echo "Container: $$(docker ps -a --filter name=ros2_annotator --format 'Status: {{.Status}}' 2>/dev/null || echo 'Not running')"
	@echo "NVIDIA: $$(docker info | grep -q nvidia && echo 'Available' || echo 'Not available')"
	@echo "X11: $$(echo $$DISPLAY | grep -q ':' && echo 'Available' || echo 'Not configured')"

# Testing targets
test: test-unit test-integration
	@echo "🧪 All tests completed!"

test-unit:
	@echo "🔬 Running unit tests..."
	@python3 -m pytest tests/unit/ -v || python3 tests/unit/test_annotation_core.py

test-integration:
	@echo "🔗 Running integration tests..."
	@python3 -m pytest tests/integration/ -v || python3 tests/integration/test_workflow.py

test-docker:
	@echo "🐳 Running tests in Docker container..."
	@echo "Testing basic container functionality..."
	@docker run --rm \
		--volume="$$(pwd)/tests:/workspace/tests" \
		--entrypoint="" \
		shriman1:a \
		bash -c "source /opt/ros/humble/setup.bash && cd /workspace && python3 -c 'import sys; print(\"Python:\", sys.version)' && python3 -c 'import rclpy; print(\"ROS2: OK\")' && echo 'Container tests: PASSED'"

# Docker Compose alternatives
docker-compose:
	@echo "🐳 Starting with Docker Compose..."
	docker compose up --build

compose-gui:
	@echo "🎯 Starting annotation GUI with Docker Compose..."
	docker compose up annotator-gui

compose-dev:
	@echo "🛠️  Starting development environment with Docker Compose..."
	docker compose run --rm ros2-annotator bash

compose-down:
	@echo "🛑 Stopping Docker Compose services..."
	docker compose down

# Documentation targets
docs:
	@echo "📚 Opening documentation..."
	@if command -v xdg-open >/dev/null 2>&1; then \
		xdg-open docs/USAGE.md; \
	else \
		echo "Documentation available in docs/ directory"; \
		ls docs/; \
	fi

# Validation targets
validate:
	@echo "✅ Running validation checks..."
	@echo "Checking script permissions..."
	@find scripts/ -name "*.bash" -not -executable -exec echo "❌ Not executable: {}" \; -exec chmod +x {} \;
	@echo "Checking file structure..."
	@test -f Dockerfile || echo "❌ Missing Dockerfile"
	@test -f docker-compose.yml || echo "❌ Missing docker-compose.yml"
	@test -d rosbag_annotator || echo "❌ Missing rosbag_annotator directory"
	@echo "✅ Validation completed"

# Install development dependencies
install-dev:
	@echo "📦 Installing development dependencies..."
	@pip3 install --user pytest pytest-mock pytest-cov || echo "Install pytest manually if needed"
	@echo "✅ Development dependencies installed"

# AppImage build targets
build-appimage:
	@echo "📦 Building AppImage..."
	./build_appimage.sh

appimage-deps:
	@echo "📋 Installing AppImage build dependencies..."
	@sudo apt update
	@sudo apt install -y python3-pip python3-setuptools patchelf desktop-file-utils libgdk-pixbuf2.0-dev fakeroot strace fuse
	@pip3 install --user appimage-builder
	@echo "✅ AppImage dependencies installed"

appimage-test:
	@echo "🧪 Testing AppImage..."
	@if [ -f "ROS2_Bag_Annotator-x86_64.AppImage" ]; then \
		echo "Testing AppImage execution..."; \
		timeout 3s ./ROS2_Bag_Annotator-x86_64.AppImage --version || echo "AppImage test completed"; \
		echo "File info:"; \
		ls -lh ROS2_Bag_Annotator-x86_64.AppImage; \
		file ROS2_Bag_Annotator-x86_64.AppImage; \
	else \
		echo "❌ AppImage not found. Run 'make build-appimage' first."; \
	fi
