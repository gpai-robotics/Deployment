# `VERS_1.22.0 not found` when starting `go2_ctrl`

## Symptom

Starting the controller fails before it runs:

```
go2_ctrl: /lib64/libonnxruntime.so.1: version `VERS_1.22.0' not found
```

## Cause

`go2_ctrl` requires the bundled ONNX Runtime ABI version `VERS_1.22.0`.
The displayed path, `/lib64/libonnxruntime.so.1`, is the system library. It
was version `1.22.2`, which exports `VERS_1.22.2`, not `VERS_1.22.0`.

This happened because the existing generated controller build directory had
been configured from a different checkout path:

```
go2/reference_repos/Unitree_mjlab_repo/unitree_rl_mjlab/...
```

The launcher uses the canonical path instead:

```
go2/reference_repos/unitree_rl_mjlab/...
```

CMake embeds an absolute runtime-library path in the binary. The stale path
did not exist, so the dynamic loader fell back to the incompatible system
library.

## Fix applied

The controller was rebuilt from the canonical checkout. Its `RUNPATH` now
points to this bundled library:

```
go2/reference_repos/unitree_rl_mjlab/deploy/thirdparty/
onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1
```

The launcher also puts that directory first in `LD_LIBRARY_PATH`. The build
helper now detects a CMake build tree configured from another source path,
resets only that generated controller build directory, and rebuilds it.

## What to do next time

From the repository root, rebuild the controller and then start it:

```bash
export REPO="$PWD"
bash go2/scripts/deploy/build_unitree_mjlab_runtime.sh controller
bash "$REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh" controller
```

The rebuild is expected to say that shared libraries for `go2_ctrl` resolve.
If it reports a stale controller build directory, that reset is intentional:
it removes generated CMake output only, not source or policy files.

## Fast diagnosis

Run these commands before changing any system libraries:

```bash
CTRL="$REPO/go2/reference_repos/unitree_rl_mjlab/deploy/robots/go2/build/go2_ctrl"
ldd "$CTRL" | grep libonnxruntime
readelf -d "$CTRL" | grep -E 'RPATH|RUNPATH'
readelf --version-info "$CTRL" | grep -A1 'File: libonnxruntime.so.1'
```

Healthy output resolves `libonnxruntime.so.1` inside the repository's
`onnxruntime-linux-x64-1.22.0/lib` directory and shows a required version of
`VERS_1.22.0`.

If `ldd` resolves it to `/lib64/libonnxruntime.so.1`, rebuild with the command
above. Do **not** replace, symlink, or delete the system `/lib64` library: it
may be used by other applications and does not fix the stale-build cause.

## Separate simulator issue

`build_unitree_mjlab_runtime.sh controller` now validates only the controller.
If a simulator check separately reports `libmujoco.so.3.3.6 => not found`, that
is a MuJoCo runtime-library issue, not an ONNX Runtime/controller issue.
