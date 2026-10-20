# Firefox VA-API test suites

This directory contains the Firefox/VA-API playback harnesses.  All paths are
resolved from the repository root, so the scripts can be started from any
working directory.

| File | Purpose |
| --- | --- |
| `firefox_headless_video_test.html` | Browser-side playback/seek/sharpness/telemetry page. |
| `firefox_playback_server.py` | Static HTTP server plus `/__status__` and `/__telemetry__` endpoints. |
| `firefox_playback_benchmark.py` | Selenium benchmark, falls back to the shared harness without Selenium. |
| `measure_firefox_playback.py` | Full playback and seek matrix with `/proc/interrupts` sampling. |
| `run_firefox_vaapi_probe.sh` | Launches Firefox with a zero-copy VA-API profile. |

Typical run:

```bash
export LIBVA_DRIVERS_PATH="$PWD/build-work"
export LIBVA_DRIVER_NAME=rockchip
export ROCKCHIP_VAAPI_BUILD_DIR="$PWD/build-work"

python3 tools/firefox/measure_firefox_playback.py \
  --video "videos/Test Jellyfin 1080p AVC 20M.mp4" --mode hw --duration 45
```

Switch to a second video in the same browser session (the regression that used
to leave the second video stuck after the first one ended):

```bash
python3 tools/firefox/measure_firefox_playback.py \
  --video "videos/Test Jellyfin 1080p HEVC 8bit 20M.mp4" --mode hw --duration 20 \
  --switch-to "videos/Test Jellyfin 1080p AVC 20M.mp4" --switch-after 4
```

The bridge treats a new VA context created while an older context is still
alive as a decoder switch.  It tears down the old single-codec MPP session,
re-registers the existing VA surfaces, and initializes MPP for the new
profile, resolution, and bit depth.  The old MPP session cannot be reused:
Firefox keeps both VA contexts alive briefly and reuses the same VA display.

The server still serves the repository root because the HTML addresses the
videos as `videos/<name>.mp4`; only the test page itself moved under
`tools/firefox/`.

## MPP ABI boundary

The bridge only includes the installed public MPP headers
(`/usr/include/rockchip/*.h`) and links `librockchip_mpp.so`.  It must not
include an MPP source checkout or compile MPP source objects.  Run
`tools/check_mpp_boundary.sh` (also wired as the `check-mpp-boundary` CMake
target) after changing the decoder.

## Zero-copy and 10-bit output

AVC and 8-bit HEVC use the direct NV12 DMA-BUF export path: MPP writes the
decoded frame into the VA surface and Firefox imports that same DMA-BUF.

AV1 10-bit is different.  The public MPP ABI emits Rockchip's packed
`MPP_FMT_YUV420SP_10BIT` layout; it does not expose a P010 output mode on this
SoC.  The bridge therefore expands packed 10-bit to P010 on the export path
(bit-exact with the software decoder, verified by PSNR/cmp) and parallelizes
that conversion across the available cores (capped at eight).  The export
DMA-BUF is allocated cacheable and bracketed with `DMA_BUF_IOCTL_SYNC`, which
removed the uncached-write stall that used to make 1080p AV1 10-bit slower
than software decode.  There is no extra full-frame memcpy: the conversion
lands directly in the exported DMA-BUF that Firefox consumes.  Keeping P010 is required for correct colours and sharpness; the
old NV12 fallback produced black/blurred AV1 output on the test stream.

### Optional imported P010 buffers

For experiments with a literal single-buffer AV1 path, set
`ROCKCHIP_VAAPI_EXTERNAL_P010=1`.  The decoder then imports twenty P010-sized
cacheable DMA-BUFs into an MPP external buffer group.  MPP writes its packed
10-bit frame into the front of one imported buffer and the bridge expands it
in place before exporting that same descriptor.  The result is bit-exact with
software P010 (verified with `cmp`), and no second frame-sized DMA-BUF is
allocated.

The in-place expansion is serial because its P010 destination overlaps the
packed source.  Set `ROCKCHIP_VAAPI_EXTERNAL_P010_SCRATCH=1` to copy the packed
frame to a reusable scratch area and expand in parallel; this is faster but
adds one CPU-side copy.  The imported path remains opt-in: on this board the
external-group scheduling overhead makes it slower than the default direct MPP
buffer export, so the default remains the faster path.

The default AV1 10-bit path still uses MPP's packed output buffer and expands
it directly into the exported DMA-BUF, with no extra full-frame DMA-BUF copy.

### RGA P010 on RK3588

The RGA API exposes `RK_FORMAT_P010` (and `DRM_FORMAT_P010`) in librga
1.10.6, but the RK3588 RGA3/RGA2 hardware on this board rejects it: the
1.10.6 changelog recommends RGA driver 1.3.13 or newer (minimum 1.2.4), while
the installed kernel driver is 1.3.7.  A minimal `imcvtcolor(..., P010)` probe
returns `Unsupported function: dst unsupported this format, format =
0x4000`, and a legacy/`imcopy` P010 probe fails in the RGA job submission
(`rga: ... no core match` / `submit failed`).  The bridge therefore keeps the
verified CPU expansion as the default and never claims a hardware RGA P010
path that this kernel cannot execute.  Re-test the probe after upgrading the
RGA kernel driver to 1.3.13+.

### AV1 8-bit web streams and hidden frames

Bilibili's AV1 player emits `show_frame=0` reference frames before displayable
frames.  MPP decodes those frames but deliberately returns no displayable
frame for them.  `vaSyncSurface` still runs for every VA surface, so treating
the hidden surface as pending deadlocked after the first group of pictures
(`completed_job=0`).  The bridge now completes hidden-frame syncs with a valid
black export buffer, which satisfies FFmpeg's `vaExportSurfaceHandle` check
without publishing a fake display frame; MPP keeps the real reference pixels
internally.  Bit depth is selected from the VA surfaces created by the client,
not from the AV1 profile: AV1 Profile0 is used for both 8-bit and 10-bit, and
forcing 10-bit MPP output on an 8-bit stream also stalled.
