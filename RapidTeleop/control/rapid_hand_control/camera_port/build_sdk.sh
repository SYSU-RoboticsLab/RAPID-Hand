#!/bin/bash
set -e  # Exit immediately on error

# Step 1: Navigate to the SDK source directory
echo "[1/5] Entering SDK source directory..."
cd "$(dirname "$0")/pyorbbecsdk"

# Step 2: Prepare build directory
echo "[2/5] Creating build directory..."
mkdir -p build && cd build

# Step 3: Run CMake configuration
echo "[3/5] Configuring with CMake..."
if ! command -v pybind11-config &> /dev/null; then
    echo "Error: pybind11-config not found. Please make sure pybind11 is installed."
    exit 1
fi

cmake -Dpybind11_DIR=$(pybind11-config --cmakedir) ..

# Step 4: Compile the SDK
echo "[4/5] Building the SDK..."
make -j"$(nproc)"
make install

# Step 5: Copy built libraries to camera_sdk/
echo "[5/5] Copying compiled libraries to camera_sdk/..."
cd ..
mkdir -p ../camera_sdk
cp -r install/lib/* ../camera_sdk/

echo "✅ Build completed successfully. Libraries copied to camera_sdk/"
