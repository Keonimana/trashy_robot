# trashy-robot

An autonomous RC-scale car that finds trash with an OAK-D Lite stereo camera and
drives up to it using closed-loop steering and speed control. It runs on an Nvidia
Jetson TX2 as a set of small processes that communicate over ZeroMQ.

Built in spring 2024 by Keoni Spencer and Oliver Ye. Oliver built and trained the
learned classifier; Keoni built the camera, color detector, and control stack.

## How it works

```
                        RGB frames (:5553)                target centroid (:5555)
 OAK-D Lite ──► camera ───────────────────► classifier ───────────┬──► auto_driver
                  ▲ │                                             │        ▲ │
                  │ └── depth at target (:5554) ──────────────────┼────────┘ │
                  └───────────────────────────────────────────────┘          │
                         (centroid re-centers the depth ROI)                 │
                                                       steer, throttle (:5556)
                                                                             ▼
                                       servo + ESC ◄── PCA9685 ◄── cmd_reciever
```

| Component | What it does |
| --- | --- |
| `camera/` | DepthAI pipeline on the OAK-D Lite. Publishes JPEG-encoded 300x300 RGB preview frames, and re-centers a spatial-location ROI on the current target to publish its depth. |
| `classifier/naive_classifier/` | Learned detector, the primary one: an SSDLite320 + MobileNetV3 network (PyTorch). Publishes the center of its best box. |
| `classifier/src/classifier.cpp` | Fallback detector: HSV color threshold, largest contour, image moments. |
| `controller/src/auto_driver.cpp` | Two PID loops: steering from horizontal pixel error, throttle from saturated depth error. Publishes normalized commands at 20 Hz. The gains in the tree are proportional-only, which is what the car last ran with. |
| `controller/src/cmd_reciever.cpp` | Maps normalized commands to PWM, smooths throttle with a moving average, and drives the servo and ESC through a PCA9685 over I2C. |
| `controller/src/manual_driver.cpp` | Keyboard driving for testing the drivetrain. |
| `controller/src/point_generator.cpp` | Publishes random points on :5555 for testing without a detector. |

Both detectors publish the same message on tcp :5555: `CommandPoint {float x, y, z}`,
with `x, y` in pixels of the 300x300 preview frame, `z = 0`, and `(-1, -1, 0)` when
there is no target. `auto_driver` fills in `z` from the depth stream.

### Detectors

The learned classifier is the primary detector, but it performed poorly in testing.
The color detector was kept as the reliable fallback, and it is the one the control
loop was tuned against. It needs the target to contrast with the floor: the HSV range
in `classifier.cpp` picks out red and pink objects.

## Building

Tested dependencies: OpenCV, libzmq and cppzmq, ncurses, and
[depthai-core](https://github.com/luxonis/depthai-core) at the pinned commit below (v2.25.1).
Newer depthai-core releases have a different API. Run these from the repo root.

```bash
# I2C builds in place because PCA9685 links against controller/libs/I2C/src/libI2C.so
cmake -S controller/libs/I2C/src -B controller/libs/I2C/src
cmake --build controller/libs/I2C/src
cmake -S controller/libs/PCA9685/src -B controller/libs/PCA9685/src/build
cmake --build controller/libs/PCA9685/src/build

# controller and HSV detector
cmake -S controller -B controller/build && cmake --build controller/build
cmake -S classifier -B classifier/build && cmake --build classifier/build

# depthai-core is not vendored
git clone https://github.com/luxonis/depthai-core.git libs/depthai-core
git -C libs/depthai-core checkout c21bdd3746c4ad7134b67c2c75a11fbdd0a01c40
git -C libs/depthai-core submodule update --init --recursive
cmake -S libs/depthai-core -B libs/depthai-core/build
cmake --build libs/depthai-core/build

# camera
cmake -S camera -B camera/build -D"depthai_DIR=$PWD/libs/depthai-core/build"
cmake --build camera/build
```

The learned classifier also needs Python packages. The first run downloads the
torchvision MobileNetV3 backbone before loading the trained weights.

```bash
pip install torch torchvision opencv-python pyzmq pillow numpy
```

## Running

Start each process in its own terminal on the Jetson. Keep the car on a stand the
first time, since `cmd_reciever` arms the ESC by sweeping the throttle on startup.

```bash
camera/build/bin/camera
classifier/run_classifier.sh          # learned classifier; add "hsv" for the color detector
controller/build/bin/auto_driver
controller/build/bin/cmd_reciever
```

## Known limitations

- The car stops at the target; there is no pickup mechanism in this code.
- Ports are hard-coded and all processes assume they run on the same host.

## Credits

- Keoni Spencer: camera pipeline, color detector, control stack.
- Oliver Ye: learned classifier and its training.
- `controller/libs/` vendors the [PCA9685](https://github.com/TeraHz/PCA9685) and
  [I2C](https://github.com/TeraHz/I2C) drivers by TeraHz.
