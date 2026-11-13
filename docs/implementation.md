# Implementation record — webcam Dobot control

The approved conversation plan is implemented in place. This workspace has an empty, read-only `.git` directory that is not a usable Git repository, so no branches, worktrees or commits were created.

## Architecture

- `control.py`: validated configuration, normalized XY zones, bounded incremental movement and pickup/place/recovery state machine.
- `vision.py`: MediaPipe Tasks Hand Landmarker, aspect-corrected finger angles and palm center; two-hand detection.
- `runtime.py`: independent control thread, latest-frame observation, bounded keyboard queue and priority stop event; teaching workflow.
- `sdk.py`: vendor-provided Python wrapper in a spawned process, bounded RPC and terminal failures. Matching vendor wrapper and x64 DLLs are supplied by the operator, not redistributed here.
- `app.py`: native OpenCV window, model download, CLI and a headless simulated pickup/place demo.

## Progress and review

- Task 1 complete: state machine and simulator, initial 11 tests passed.
- Task 2 complete: gesture recognition and teaching, 18 tests passed.
- Task 3 complete: SDK process isolation and timeout tests, 21 tests passed.
- Task 4 complete: runtime and camera window, 29 tests passed.
- Fresh read-only implementation review found simulator-calibration recovery bypass and stale actuation after slow SDK polling. Both were reproduced with failing tests and fixed: real calibration required for recovery, real elapsed freshness recheck, SDK call timeouts reduced to 200 ms and automatic commands receive the frame deadline. Suite after fixes: 31/31.
- Task 5: Windows installation and operator documentation added; final verification recorded below when complete.

## Decisions and limits

- Work directly in the supplied empty directory; Git metadata is not usable.
- Software stop is best effort. A communication failure cannot prove physical stopping. Killing a blocked SDK worker prevents late retries but cannot undo commands already received by the robot.
- Teaching is deliberate 1 mm keyboard movement before bounds are known; the operator confirms reachability/clearance of the whole rectangle. Two reachable corners alone are insufficient.
- Windows x64 Python 3.11 is the runtime target. Dependency-free unit tests can also run on the available Linux Python 3.14.
- Installing Python 3.11 / camera dependencies in this sandbox was attempted but DNS/network access failed. No camera, robot USB port or Windows runtime is available here. Webcam inference, vendor DLL loading and physical movement remain hardware acceptance checks.
- SDK functions run through the matching vendor wrapper rather than duplicated native ctypes signatures, to avoid ABI differences between SDK releases.

## Final verification

- `python -m unittest discover -s tests -v`: 34 tests passed on Linux Python 3.14.8, including expired native-command deadlines, a stalled movement, and interruption after suction activation followed by manual recovery.
- `python -m compileall -q dobot_control tests`: passed.
- `python -m dobot_control --help`: passed.
- `python -m dobot_control --headless-demo`: passed; 504 bounded movement segments, pump events `[true, false]`, final Z `50.0`, final state `paused`.
- No pending review findings remain. No files were committed or pushed; no working Git repository is available.
- Camera inference and physical USB/DLL behavior were not exercised in this environment; see the README hardware acceptance sequence.
