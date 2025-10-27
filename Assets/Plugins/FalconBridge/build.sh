#!/bin/bash

# FalconBridge Build Script for macOS
# This script automates the building process of the FalconBridge Unity plugin

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
OUTPUT_DIR="$SCRIPT_DIR/../macOS"

echo "=================================="
echo "FalconBridge Build Script"
echo "=================================="
echo ""

# Check if libnifalcon is built
LIBNIFALCON_LIB="$SCRIPT_DIR/../../../libnifalcon/build/lib/libnifalcon.dylib"
echo "Checking for libnifalcon in submodule..."
if [ ! -f "$LIBNIFALCON_LIB" ]; then
    echo "ERROR: libnifalcon not built!"
    echo ""
    echo "Please build libnifalcon first:"
    echo "  cd $SCRIPT_DIR/../../../libnifalcon"
    echo "  mkdir -p build && cd build"
    echo "  cmake .."
    echo "  make"
    echo ""
    exit 1
fi

echo "✓ libnifalcon found in submodule"
echo ""

# Create build directory
echo "Creating build directory..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
echo "✓ Build directory ready: $BUILD_DIR"
echo ""

# Run CMake
echo "Running CMake configuration..."
cmake .. || {
    echo "ERROR: CMake configuration failed!"
    exit 1
}
echo "✓ CMake configuration successful"
echo ""

# Build the plugin
echo "Building FalconBridge.bundle..."
make || {
    echo "ERROR: Build failed!"
    exit 1
}
echo "✓ Build successful"
echo ""

# Check if output exists
BUNDLE_DIR="$OUTPUT_DIR/FalconBridge.bundle"
BUNDLE_EXEC="$BUNDLE_DIR/Contents/MacOS/FalconBridge"
FRAMEWORKS_DIR="$BUNDLE_DIR/Contents/Frameworks"

if [ ! -f "$BUNDLE_EXEC" ]; then
    echo "ERROR: Build succeeded but FalconBridge executable not found!"
    echo "Expected: $BUNDLE_EXEC"
    exit 1
fi

# Fix library dependencies
echo "Fixing library dependencies..."
mkdir -p "$FRAMEWORKS_DIR"

# Copy libnifalcon
LIBNIFALCON_SRC="$SCRIPT_DIR/../../../libnifalcon/build/lib/libnifalcon.1.0.2.dylib"
if [ -f "$LIBNIFALCON_SRC" ]; then
    cp "$LIBNIFALCON_SRC" "$FRAMEWORKS_DIR/"
    install_name_tool -change /usr/local/lib/libnifalcon.1.0.2.dylib @loader_path/../Frameworks/libnifalcon.1.0.2.dylib "$BUNDLE_EXEC"
    install_name_tool -id @rpath/libnifalcon.1.0.2.dylib "$FRAMEWORKS_DIR/libnifalcon.1.0.2.dylib"
    echo "✓ libnifalcon copied and linked"
else
    echo "WARNING: libnifalcon not found at $LIBNIFALCON_SRC"
fi

# Copy libusb
LIBUSB_PATH=$(find /opt/homebrew/opt/libusb/lib -name "libusb-1.0.*.dylib" | head -1)
if [ -n "$LIBUSB_PATH" ]; then
    LIBUSB_NAME=$(basename "$LIBUSB_PATH")
    cp "$LIBUSB_PATH" "$FRAMEWORKS_DIR/"
    install_name_tool -change "$LIBUSB_PATH" "@loader_path/$LIBUSB_NAME" "$FRAMEWORKS_DIR/libnifalcon.1.0.2.dylib"
    echo "✓ libusb copied and linked"
else
    echo "WARNING: libusb not found"
fi

# Verify dependencies
echo ""
echo "Final dependencies:"
otool -L "$BUNDLE_EXEC" | grep -v ":" || true
echo ""

# Remove .meta file to force Unity to regenerate it
META_FILE="$BUNDLE_DIR.meta"
if [ -f "$META_FILE" ]; then
    rm "$META_FILE"
    echo "✓ Removed old .meta file"
fi

echo "=================================="
echo "Build Complete!"
echo "=================================="
echo ""
echo "Plugin location: $BUNDLE_DIR"
echo ""
echo "Next steps:"
echo "1. Return to Unity Editor"
echo "2. Unity should automatically detect the new plugin"
echo "3. Open Assets/Scenes/SampleScene.unity"
echo "4. Connect your Novint Falcon device"
echo "5. Click Play to test!"
echo ""
