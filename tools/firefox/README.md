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
that conversion across up to four cores.  There is no extra full-frame
memcpy: the conversion lands directly in the exported DMA-BUF that Firefox
consumes.  Keeping P010 is required for correct colours and sharpness; the
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
