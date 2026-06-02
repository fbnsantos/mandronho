# Two-camera visual-servo C library

This library computes Cartesian velocity references from object observations.

## Camera 1

Camera 1 is mounted after joint 1.

Its task is to align the detected object with a configurable vertical line:

```text
u_target = cx + horizontal_line_offset_px
```

It generates a planar velocity reference in the robot base frame:

```text
vx, vy
```

The helper:

```c
vision_servo_update_camera1_pose_from_q1(...)
```

updates the camera orientation when joint 1 rotates.

## Camera 2

Camera 2 is mounted on link 4.

Its task is to:

- centre the detected object horizontally;
- centre the detected object vertically;
- reach the configured target distance.

It generates:

```text
vx, vy, vz
```

The helper:

```c
vision_servo_update_camera2_pose_from_T04(...)
```

uses the manipulator homogeneous matrix `T04` to transform camera-frame velocity references into the robot base frame.

## Required observation

For each camera, provide:

```c
VisionServoObjectObservation observation = {
    .u_px = ...,
    .v_px = ...,
    .distance = ...
};
```

`distance` is the object depth along the optical axis. It must be positive.

## Camera convention

```text
+X: right in the image
+Y: down in the image
+Z: forward along the optical axis
```

If the physical camera uses a different convention, configure its fixed mounting rotation matrix.

## Compile as a library

```bash
gcc -Wall -Wextra -std=c11 -c vision_servo.c -o vision_servo.o
ar rcs libvision_servo.a vision_servo.o
```

## Compile the demonstration program

```bash
gcc -Wall -Wextra -std=c11 \
    -DVISION_SERVO_BUILD_DEMO \
    vision_servo.c -o vision_servo_demo -lm
```

## Test camera 1

```bash
./vision_servo_demo cam1 410 245 1.20 30 10
```

Arguments:

```text
cam1 object_u_px object_v_px distance q1_deg line_offset_px
```

## Test camera 2

```bash
./vision_servo_demo cam2 370 260 0.80 0.50
```

Arguments:

```text
cam2 object_u_px object_v_px distance target_distance
```

## Integration with the manipulator library

Typical flow:

```c
VisionServoVelocity desired_velocity;
VisionServoObjectObservation observation;

/* Update camera 2 orientation from the manipulator digital model. */
vision_servo_update_camera2_pose_from_T04(
    manipulator_state.T04,
    R_camera2_mount,
    &camera2_config.pose);

/* Calculate Cartesian velocity reference. */
vision_servo_camera2_compute_cartesian_velocity(
    &camera2_config,
    &observation,
    &desired_velocity);

/* Send desired_velocity to the manipulator velocity controller. */
```

For camera 1, use:

```c
vision_servo_update_camera1_pose_from_q1(
    manipulator_state.q1,
    R_camera1_mount,
    &camera1_config.pose);
```

## Important design note

This is a proportional visual-servo controller. It assumes that the object detector or tracker already provides pixel coordinates and distance. It does not perform image acquisition, object detection or depth estimation.
