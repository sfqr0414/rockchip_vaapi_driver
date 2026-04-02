# Gecko Compatibility Report

This repository includes two standalone validation tools:
- `build/gecko_vaapi_probe`
- `build/panthor_egl_import_test`

Purpose:
- Validate the VA-API contract Gecko expects independently of Firefox policy gating.
- Confirm `vaGetDisplayDRM` initialization, `DRM_PRIME_2` surface attributes, `VA_EXPORT_SURFACE_SEPARATE_LAYERS`, and direct DMA-BUF access.
- Optionally validate the `DMA-BUF -> EGLImage -> GLES texture -> glDrawArrays` path on the local GPU stack.

Validated contract fields:
- `vaGetDisplayDRM`: required by Gecko VA display creation.
- `vaQuerySurfaceAttributes`: must advertise `VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2`.
- DRM modifier: must include `DRM_FORMAT_MOD_LINEAR` for the exported surface object.
- `vaExportSurfaceHandle`: must accept `VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_SEPARATE_LAYERS`.
- NV12 export layout:
  Layer 0 format `R8`
  Layer 1 format `GR88`
- P010 export layout:
  Layer 0 format `R16`
  Layer 1 format `GR1616`
- Zero-copy evidence: exported DMA-BUF can be mapped directly without any CPU copy step in the probe.

Reproduction:
```bash
cmake -S . -B build
cmake --build build -j4 --target gecko_vaapi_probe panthor_egl_import_test
LIBVA_DRIVER_NAME=rockchip ./build/gecko_vaapi_probe --export-type prime2
LIBVA_DRIVER_NAME=rockchip ./build/gecko_vaapi_probe --export-type prime2 --import-egl
LIBVA_DRIVER_NAME=rockchip ./build/gecko_vaapi_probe --input "videos/Test Jellyfin 1080p AVC 20M.mp4" --export-type prime2
LIBVA_DRIVER_NAME=rockchip ./build/gecko_vaapi_probe --input "videos/Test Jellyfin 1080p AVC 20M.mp4" --export-type prime2 --import-egl
./build/panthor_egl_import_test
```

Interpretation:
- If PRIME_2 export succeeds but Firefox still decodes through shmem/software paths, the remaining blocker is outside the driver export contract.
- If `--import-egl` succeeds as well, the exported DMA-BUF is acceptable to the local EGL/GLES stack and Firefox has even less justification to reject the zero-copy path.

Observed on this machine:
- After removing the tmpfile-backed fallback export path, a blank-surface probe no longer exports anything: `vaSyncSurface` now fails with `VA_STATUS_ERROR_INVALID_SURFACE`, which is the expected behavior for a surface with no decoded output attached.
- `LIBVA_DRIVER_NAME=rockchip ./build/gecko_vaapi_probe --input "videos/Test Jellyfin 1080p AVC 20M.mp4" --export-type prime2` decodes a real H.264 frame, reports `mpp: zero-copy frame ... fd=15`, and exports a valid PRIME_2 descriptor from that decoded surface.
- `LIBVA_DRIVER_NAME=rockchip ./build/gecko_vaapi_probe --input "videos/Test Jellyfin 1080p AVC 20M.mp4" --export-type prime2 --import-egl` succeeds end-to-end on Panthor/Mesa; the child probe reports `renderer=Mali-G610 (Panfrost)` and completes the EGLImage import + GLES draw path successfully.
- `./build/panthor_egl_import_test` uses a pure DRM dumb-buffer allocation path with one DMA-BUF object, `R8 + GR88` separate layers, and the same `eglCreateImageKHR` import model; Panthor itself cannot allocate dumb buffers, so the tool falls back to `/dev/dri/card0` for allocation and still imports successfully on Mesa/Panfrost.
- The real decoded-frame probe now shows the exported object resolving to `/dmabuf:` instead of `/tmp/rockchip-vaapi-XXXXXX (deleted)`.
- The corresponding `fdinfo` now includes dma-buf fields again (`size`, `count`, and `exp_name`), confirming that the exported object identity matches a normal kernel dma-buf instead of a deleted tmpfile.

Current conclusion:
- The VA-API driver satisfies Gecko's export contract.
- The old Mesa crash was caused by the driver's deleted-tmpfile fallback export path, not by PRIME_2 layer descriptors and not by Panthor's inability to import real decoded MPP dma-bufs.
- On the fixed path, a real decoded MPP frame exports as a standard `/dmabuf:` object and imports successfully through `EGLImage -> GLES` on Panthor/Mesa.
- If Firefox still refuses to use the zero-copy path after this point, the remaining blocker is now much more likely to be Firefox policy/process gating or a Firefox-specific import/use-site issue rather than the kernel object exported by this driver.