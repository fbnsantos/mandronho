#ifndef VISION_SERVO_H
#define VISION_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Visual-servo library for two cameras.
 *
 * Camera-frame convention:
 *
 *   +X: right in the image
 *   +Y: down in the image
 *   +Z: forward, along the optical axis
 *
 * Base-frame velocities are returned after applying the configured
 * camera-to-base rotation matrix.
 *
 * Camera 1:
 *   - mounted after joint 1;
 *   - aligns the object with a configurable vertical image line;
 *   - returns planar base-frame velocity vx, vy.
 *
 * Camera 2:
 *   - mounted on the prismatic fourth link;
 *   - centres the object in the image;
 *   - regulates its distance to a configurable target distance;
 *   - returns base-frame velocity vx, vy, vz.
 */

/* ============================================================
 * Default parameters: edit these values for your application.
 * ============================================================ */

/* Camera 1 intrinsics [pixels]. */
#define VS_DEFAULT_CAM1_FX                    640.0
#define VS_DEFAULT_CAM1_FY                    640.0
#define VS_DEFAULT_CAM1_CX                    320.0
#define VS_DEFAULT_CAM1_CY                    240.0

/*
 * Desired vertical alignment line for camera 1:
 *
 *   target_u = cx + horizontal_line_offset_px
 */
#define VS_DEFAULT_CAM1_LINE_OFFSET_PX          0.0

/*
 * Lateral visual-servo gain.
 *
 * The commanded camera-frame lateral velocity is:
 *
 *   vx_camera = gain * depth * normalized_horizontal_error
 */
#define VS_DEFAULT_CAM1_LATERAL_GAIN            0.8

#define VS_DEFAULT_CAM1_MAX_PLANAR_SPEED        0.10
#define VS_DEFAULT_CAM1_HORIZONTAL_DEADBAND_PX  2.0

/* Camera 2 intrinsics [pixels]. */
#define VS_DEFAULT_CAM2_FX                    640.0
#define VS_DEFAULT_CAM2_FY                    640.0
#define VS_DEFAULT_CAM2_CX                    320.0
#define VS_DEFAULT_CAM2_CY                    240.0

/* Configurable image-centre offsets for camera 2 [pixels]. */
#define VS_DEFAULT_CAM2_OFFSET_U_PX              0.0
#define VS_DEFAULT_CAM2_OFFSET_V_PX              0.0

/* Desired object distance along the camera optical axis [m]. */
#define VS_DEFAULT_CAM2_TARGET_DISTANCE          0.50

/* Visual-servo gains. */
#define VS_DEFAULT_CAM2_HORIZONTAL_GAIN          0.8
#define VS_DEFAULT_CAM2_VERTICAL_GAIN            0.8
#define VS_DEFAULT_CAM2_DEPTH_GAIN               0.6

/* Velocity limits in the camera frame [m/s]. */
#define VS_DEFAULT_CAM2_MAX_VX                   0.10
#define VS_DEFAULT_CAM2_MAX_VY                   0.10
#define VS_DEFAULT_CAM2_MAX_VZ                   0.10

#define VS_DEFAULT_CAM2_IMAGE_DEADBAND_PX        2.0
#define VS_DEFAULT_CAM2_DISTANCE_DEADBAND        0.005

typedef enum
{
    VS_OK = 0,
    VS_ERROR_INVALID_ARGUMENT,
    VS_ERROR_INVALID_INTRINSICS,
    VS_ERROR_INVALID_DISTANCE
} VisionServoStatus;

typedef struct
{
    double fx;
    double fy;
    double cx;
    double cy;
} VisionServoCameraIntrinsics;

typedef struct
{
    /*
     * Rotation matrix that converts camera-frame vectors into
     * robot-base-frame vectors:
     *
     *   velocity_base = R_camera_to_base * velocity_camera
     */
    double R_camera_to_base[3][3];
} VisionServoCameraPose;

typedef struct
{
    /*
     * Detected object centre in image coordinates [pixels].
     */
    double u_px;
    double v_px;

    /*
     * Object distance along the optical axis [m].
     * Must be positive.
     */
    double distance;
} VisionServoObjectObservation;

typedef struct
{
    double vx;
    double vy;
    double vz;
} VisionServoVelocity;

typedef struct
{
    VisionServoCameraIntrinsics intrinsics;
    VisionServoCameraPose pose;

    double horizontal_line_offset_px;
    double lateral_gain;
    double maximum_planar_speed;
    double horizontal_deadband_px;
} VisionServoCamera1Config;

typedef struct
{
    VisionServoCameraIntrinsics intrinsics;
    VisionServoCameraPose pose;

    double image_offset_u_px;
    double image_offset_v_px;
    double target_distance;

    double horizontal_gain;
    double vertical_gain;
    double depth_gain;

    double maximum_vx;
    double maximum_vy;
    double maximum_vz;

    double image_deadband_px;
    double distance_deadband;
} VisionServoCamera2Config;

/* Configuration. */
void vision_servo_default_camera1_config(VisionServoCamera1Config *config);
void vision_servo_default_camera2_config(VisionServoCamera2Config *config);
const char *vision_servo_status_string(VisionServoStatus status);

/* Rotation helpers. */
void vision_servo_identity_rotation(double R[3][3]);
void vision_servo_rotation_x(double angle_rad, double R[3][3]);
void vision_servo_rotation_y(double angle_rad, double R[3][3]);
void vision_servo_rotation_z(double angle_rad, double R[3][3]);

void vision_servo_multiply_rotation(
    const double A[3][3],
    const double B[3][3],
    double result[3][3]);

void vision_servo_apply_rotation(
    const double R[3][3],
    const VisionServoVelocity *input,
    VisionServoVelocity *output);

/*
 * Camera 1 pose helper.
 *
 * Use when camera 1 rotates with joint q1:
 *
 *   R_camera_to_base = Rz(q1) * R_mount
 *
 * R_mount represents the fixed camera orientation relative to the
 * coordinate system immediately after joint 1.
 */
void vision_servo_update_camera1_pose_from_q1(
    double q1_rad,
    const double R_mount[3][3],
    VisionServoCameraPose *pose);

/*
 * Camera 2 pose helper.
 *
 * Use when T04 is provided by the manipulator kinematics:
 *
 *   R_camera_to_base = R04 * R_mount
 *
 * R_mount represents the fixed camera orientation relative to link 4.
 */
void vision_servo_update_camera2_pose_from_T04(
    const double T04[4][4],
    const double R_mount[3][3],
    VisionServoCameraPose *pose);

/*
 * Camera 1 controller.
 *
 * Aligns the detected object with:
 *
 *   u_target = cx + horizontal_line_offset_px
 *
 * The output is forced to a planar base-frame velocity:
 *
 *   vz = 0
 */
VisionServoStatus vision_servo_camera1_compute_planar_velocity(
    const VisionServoCamera1Config *config,
    const VisionServoObjectObservation *observation,
    VisionServoVelocity *velocity_base);

/*
 * Camera 2 controller.
 *
 * Centres the object at:
 *
 *   u_target = cx + image_offset_u_px
 *   v_target = cy + image_offset_v_px
 *
 * and regulates:
 *
 *   distance -> target_distance
 */
VisionServoStatus vision_servo_camera2_compute_cartesian_velocity(
    const VisionServoCamera2Config *config,
    const VisionServoObjectObservation *observation,
    VisionServoVelocity *velocity_base);

#ifdef __cplusplus
}
#endif

#endif
