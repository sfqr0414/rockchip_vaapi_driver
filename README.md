# Rockchip VA-API Bridge (VA-API -> Rockchip MPP)

This repository contains a minimal VA-API driver (`librockchip_drv_video.so`) that aims to bridge Firefox's VA-API usage to Rockchip MPP on RK3588 (Orange Pi 5 Max).

## What is included

- **`src/driver.cpp`**: VA-API DDI driver implementing the basic VA-API function table.
- **`src/mpp_decoder.*`**: Minimal decoder wrapper to talk to the Rockchip MPP API.
- **`src/util/*`**: Utilities including a `util::log` formatter and a lock-free `AtomicSyncQueue`.
- **`mpp/`**: Stub headers for Rockchip MPP API (used when system `rockchip-mpp` headers/libraries are not available).
- **`deploy.sh`**: Build and install the driver into `/usr/lib/aarch64-linux-gnu/dri/`.

## Build

```bash
./deploy.sh
```

## Usage

Set the following environment variables before launching Firefox:

```bash
export LIBVA_DRIVER_NAME=rockchip
export MOZ_ENABLE_WAYLAND=1
firefox
```

## Notes

- This implementation is **a minimal skeleton** that demonstrates the required structure (VA-API DDI entry points, atomic sync queue, DMA-BUF export). It includes a stubbed MPP implementation so it can build on systems without Rockchip MPP installed.

- To run on actual hardware (RK3588), ensure the upstream `librockchip-mpp-dev` (or equivalent) is installed and that `/dev/mpp_service` is accessible from Firefox.

- For real zero-copy decoding, the driver must properly parse VA buffers and feed bitstream into MPP, then expose the decoded frame DMA-BUF via `VASurfaceAttribExternalBuffers` / `vaExportSurfaceHandle`.
