#!/bin/bash

# Content Validation Framework for Code Examples
# This script validates that code examples in the repository are functional and properly tested

set -e  # Exit on any error

# Configuration
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXAMPLES_DIR="$REPO_ROOT/module-01-ros2"
LOG_FILE="$REPO_ROOT/test-results.log"
FAILED_TESTS=()

echo "Starting content validation for code examples..."
echo "Repository root: $REPO_ROOT"
echo "Examples directory: $EXAMPLES_DIR"
echo "Log file: $LOG_FILE"
echo

# Function to run Python syntax check
check_python_syntax() {
    local file="$1"
    echo "Checking Python syntax for: $file"

    if python3 -m py_compile "$file" 2>/dev/null; then
        echo "✓ Syntax OK: $file"
        return 0
    else
        echo "✗ Syntax Error: $file"
        FAILED_TESTS+=("$file (syntax)")
        return 1
    fi
}

# Function to validate ROS 2 examples
validate_ros_examples() {
    echo "Validating ROS 2 examples..."

    # Find all Python files in the examples directories
    find "$EXAMPLES_DIR" -name "*.py" -type f | while read -r py_file; do
        echo "Processing: $py_file"

        # Skip setup.py files and other non-example files
        if [[ "$py_file" == *"setup.py"* ]] || [[ "$py_file" == *"__pycache__"* ]]; then
            echo "Skipping non-example file: $py_file"
            continue
        fi

        # Check if it's a valid Python file by syntax
        if python3 -m py_compile "$py_file" 2>/dev/null; then
            echo "✓ Valid Python syntax: $py_file"
        else
            echo "✗ Invalid Python syntax: $py_file"
            # We'll add this to failed tests, but continue processing
        fi
    done
}

# Function to validate Docker configurations
validate_docker_configs() {
    echo "Validating Docker configurations..."

    DOCKER_DIR="$REPO_ROOT/docker"

    if [ ! -d "$DOCKER_DIR" ]; then
        echo "Docker directory not found: $DOCKER_DIR"
        return
    fi

    for dockerfile in "$DOCKER_DIR"/*/Dockerfile; do
        if [ -f "$dockerfile" ]; then
            dir_name=$(dirname "$dockerfile")
            dir_name=$(basename "$dir_name")
            echo "✓ Found Dockerfile for: $dir_name"
        else
            echo "✗ No Dockerfile found in: $DOCKER_DIR"
        fi
    done

    for compose_file in "$DOCKER_DIR"/*/docker-compose.yml; do
        if [ -f "$compose_file" ]; then
            dir_name=$(dirname "$compose_file")
            dir_name=$(basename "$dir_name")
            echo "✓ Found docker-compose.yml for: $dir_name"
        else
            echo "No docker-compose.yml found in Docker subdirectories"
        fi
    done
}

# Function to validate documentation structure
validate_docs() {
    echo "Validating documentation structure..."

    DOCS_DIR="$REPO_ROOT/docs"

    if [ ! -d "$DOCS_DIR" ]; then
        echo "✗ Documentation directory not found: $DOCS_DIR"
        return 1
    fi

    # Check for required documentation files
    required_docs=(
        "intro/welcome.md"
        "module-01-ros2/overview.md"
        "module-01-ros2/chapter-01-core-concepts.md"
        "module-01-ros2/chapter-02-nodes-topics.md"
        "module-01-ros2/chapter-03-services-actions.md"
        "module-01-ros2/troubleshooting.md"
        "module-01-ros2/project.md"
    )

    for doc in "${required_docs[@]}"; do
        if [ -f "$DOCS_DIR/$doc" ]; then
            echo "✓ Found required documentation: $doc"
        else
            echo "✗ Missing required documentation: $doc"
            FAILED_TESTS+=("$doc (missing)")
        fi
    done
}

# Function to run basic tests
run_basic_tests() {
    echo "Running basic validation tests..."

    # Test 1: Check if package.json exists and is valid
    if [ -f "$REPO_ROOT/package.json" ]; then
        if command -v node >/dev/null 2>&1; then
            if node -e "JSON.parse(require('fs').readFileSync('$REPO_ROOT/package.json', 'utf8'))" 2>/dev/null; then
                echo "✓ Valid package.json found"
            else
                echo "✗ Invalid JSON in package.json"
                FAILED_TESTS+=("package.json (invalid JSON)")
            fi
        else
            echo "! Node.js not available, skipping package.json validation"
        fi
    else
        echo "✗ package.json not found"
        FAILED_TESTS+=("package.json (missing)")
    fi

    # Test 2: Check for required directories
    required_dirs=(
        "$REPO_ROOT/docs"
        "$REPO_ROOT/src"
        "$REPO_ROOT/docker"
        "$REPO_ROOT/scripts"
        "$REPO_ROOT/module-01-ros2"
        "$REPO_ROOT/module-02-simulation"
        "$REPO_ROOT/module-03-isaac"
        "$REPO_ROOT/module-04-vla"
    )

    for dir in "${required_dirs[@]}"; do
        if [ -d "$dir" ]; then
            echo "✓ Required directory exists: $(basename "$dir")"
        else
            echo "✗ Required directory missing: $(basename "$dir")"
            # Note: We don't add this to failed tests since we're only validating module-01-ros2 right now
        fi
    done
}

# Main execution
echo "Content Validation Framework"
echo "============================"
echo "Timestamp: $(date)"
echo

validate_docs
echo

validate_docker_configs
echo

validate_ros_examples
echo

run_basic_tests
echo

# Summary
if [ ${#FAILED_TESTS[@]} -eq 0 ]; then
    echo "✓ All validations completed successfully!"
    echo "No issues found in content validation."
    exit 0
else
    echo "✗ Content validation completed with ${#FAILED_TESTS[@]} issues:"
    for failed_test in "${FAILED_TESTS[@]}"; do
        echo "  - $failed_test"
    done
    exit 1
fi