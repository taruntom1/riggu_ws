#!/bin/bash

echo "=== GPU Acceleration Test ==="
echo

echo "1. Checking OpenGL renderer:"
if command -v glxinfo >/dev/null 2>&1; then
    glxinfo | grep "OpenGL renderer"
    echo
    echo "Looking for GPU acceleration (should NOT show 'llvmpipe'):"
    if glxinfo | grep -q "llvmpipe"; then
        echo "❌ Software rendering detected (llvmpipe)"
        echo "GPU acceleration is NOT working"
    else
        echo "✅ Hardware acceleration detected"
        echo "GPU acceleration appears to be working"
    fi
else
    echo "❌ glxinfo not available"
fi

echo
echo "2. Checking OpenGL version:"
if command -v glxinfo >/dev/null 2>&1; then
    glxinfo | grep "OpenGL version"
fi

echo
echo "3. Checking GPU devices:"
ls -la /dev/dri/ 2>/dev/null || echo "❌ No DRI devices found"

echo
echo "4. Checking WSL GPU libraries:"
ls -la /usr/lib/wsl/ 2>/dev/null || echo "❌ WSL GPU libraries not mounted"

echo
echo "5. Environment variables:"
echo "MESA_D3D12_DEFAULT_ADAPTER_NAME: ${MESA_D3D12_DEFAULT_ADAPTER_NAME:-not set}"
echo "LD_LIBRARY_PATH: ${LD_LIBRARY_PATH:-not set}"
echo "LIBVA_DRIVER_NAME: ${LIBVA_DRIVER_NAME:-not set}"
echo "__NV_PRIME_RENDER_OFFLOAD: ${__NV_PRIME_RENDER_OFFLOAD:-not set}"
echo "__GLX_VENDOR_LIBRARY_NAME: ${__GLX_VENDOR_LIBRARY_NAME:-not set}"

echo
echo "=== End GPU Test ==="
