#include "vision_servo.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define VS_EPSILON 1e-12

static double vision_servo_clamp(double value, double minimum, double maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static double vision_servo_apply_deadband(double error, double deadband)
{
    return fabs(error) <= deadband ? 0.0 : error;
}

static void vision_servo_limit_planar_norm(
    VisionServoVelocity *velocity,
    double maximum_planar_speed)
{
    const double norm = hypot(velocity->vx, velocity->vy);

    if (maximum_planar_speed > 0.0 &&
        norm > maximum_planar_speed)
    {
        const double scale = maximum_planar_speed / norm;

        velocity->vx *= scale;
        velocity->vy *= scale;
    }
}

static int vision_servo_intrinsics_are_valid(
    const VisionServoCameraIntrinsics *intrinsics)
{
    return intrinsics != NULL &&
           fabs(intrinsics->fx) > VS_EPSILON &&
           fabs(intrinsics->fy) > VS_EPSILON;
}

void vision_servo_identity_rotation(double R[3][3])
{
    int row;
    int column;

    for (row = 0; row < 3; row++)
    {
        for (column = 0; column < 3; column++)
        {
            R[row][column] = row == column ? 1.0 : 0.0;
        }
    }
}

void vision_servo_rotation_x(double angle_rad, double R[3][3])
{
    const double c = cos(angle_rad);
    const double s = sin(angle_rad);

    vision_servo_identity_rotation(R);

    R[1][1] = c;
    R[1][2] = -s;
    R[2][1] = s;
    R[2][2] = c;
}

void vision_servo_rotation_y(double angle_rad, double R[3][3])
{
    const double c = cos(angle_rad);
    const double s = sin(angle_rad);

    vision_servo_identity_rotation(R);

    R[0][0] = c;
    R[0][2] = s;
    R[2][0] = -s;
    R[2][2] = c;
}

void vision_servo_rotation_z(double angle_rad, double R[3][3])
{
    const double c = cos(angle_rad);
    const double s = sin(angle_rad);

    vision_servo_identity_rotation(R);

    R[0][0] = c;
    R[0][1] = -s;
    R[1][0] = s;
    R[1][1] = c;
}

void vision_servo_multiply_rotation(
    const double A[3][3],
    const double B[3][3],
    double result[3][3])
{
    double temporary[3][3] = {{0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0}};
    int row;
    int column;
    int index;

    for (row = 0; row < 3; row++)
    {
        for (column = 0; column < 3; column++)
        {
            for (index = 0; index < 3; index++)
            {
                temporary[row][column] +=
                    A[row][index] * B[index][column];
            }
        }
    }

    memcpy(result, temporary, sizeof(temporary));
}

void vision_servo_apply_rotation(
    const double R[3][3],
    const VisionServoVelocity *input,
    VisionServoVelocity *output)
{
    output->vx =
        R[0][0] * input->vx +
        R[0][1] * input->vy +
        R[0][2] * input->vz;

    output->vy =
        R[1][0] * input->vx +
        R[1][1] * input->vy +
        R[1][2] * input->vz;

    output->vz =
        R[2][0] * input->vx +
        R[2][1] * input->vy +
        R[2][2] * input->vz;
}

void vision_servo_default_camera1_config(VisionServoCamera1Config *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));

    config->intrinsics.fx = VS_DEFAULT_CAM1_FX;
    config->intrinsics.fy = VS_DEFAULT_CAM1_FY;
    config->intrinsics.cx = VS_DEFAULT_CAM1_CX;
    config->intrinsics.cy = VS_DEFAULT_CAM1_CY;

    vision_servo_identity_rotation(config->pose.R_camera_to_base);

    config->horizontal_line_offset_px =
        VS_DEFAULT_CAM1_LINE_OFFSET_PX;

    config->lateral_gain =
        VS_DEFAULT_CAM1_LATERAL_GAIN;

    config->maximum_planar_speed =
        VS_DEFAULT_CAM1_MAX_PLANAR_SPEED;

    config->horizontal_deadband_px =
        VS_DEFAULT_CAM1_HORIZONTAL_DEADBAND_PX;
}

void vision_servo_default_camera2_config(VisionServoCamera2Config *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));

    config->intrinsics.fx = VS_DEFAULT_CAM2_FX;
    config->intrinsics.fy = VS_DEFAULT_CAM2_FY;
    config->intrinsics.cx = VS_DEFAULT_CAM2_CX;
    config->intrinsics.cy = VS_DEFAULT_CAM2_CY;

    vision_servo_identity_rotation(config->pose.R_camera_to_base);

    config->image_offset_u_px =
        VS_DEFAULT_CAM2_OFFSET_U_PX;

    config->image_offset_v_px =
        VS_DEFAULT_CAM2_OFFSET_V_PX;

    config->target_distance =
        VS_DEFAULT_CAM2_TARGET_DISTANCE;

    config->horizontal_gain =
        VS_DEFAULT_CAM2_HORIZONTAL_GAIN;

    config->vertical_gain =
        VS_DEFAULT_CAM2_VERTICAL_GAIN;

    config->depth_gain =
        VS_DEFAULT_CAM2_DEPTH_GAIN;

    config->maximum_vx =
        VS_DEFAULT_CAM2_MAX_VX;

    config->maximum_vy =
        VS_DEFAULT_CAM2_MAX_VY;

    config->maximum_vz =
        VS_DEFAULT_CAM2_MAX_VZ;

    config->image_deadband_px =
        VS_DEFAULT_CAM2_IMAGE_DEADBAND_PX;

    config->distance_deadband =
        VS_DEFAULT_CAM2_DISTANCE_DEADBAND;
}

const char *vision_servo_status_string(VisionServoStatus status)
{
    switch (status)
    {
        case VS_OK:
            return "OK";

        case VS_ERROR_INVALID_ARGUMENT:
            return "invalid argument";

        case VS_ERROR_INVALID_INTRINSICS:
            return "invalid camera intrinsics";

        case VS_ERROR_INVALID_DISTANCE:
            return "object distance must be positive";

        default:
            return "unknown error";
    }
}

void vision_servo_update_camera1_pose_from_q1(
    double q1_rad,
    const double R_mount[3][3],
    VisionServoCameraPose *pose)
{
    double R_joint_1[3][3];

    if (R_mount == NULL || pose == NULL)
    {
        return;
    }

    vision_servo_rotation_z(q1_rad, R_joint_1);

    vision_servo_multiply_rotation(
        R_joint_1,
        R_mount,
        pose->R_camera_to_base);
}

void vision_servo_update_camera2_pose_from_T04(
    const double T04[4][4],
    const double R_mount[3][3],
    VisionServoCameraPose *pose)
{
    double R04[3][3];
    int row;
    int column;

    if (T04 == NULL || R_mount == NULL || pose == NULL)
    {
        return;
    }

    for (row = 0; row < 3; row++)
    {
        for (column = 0; column < 3; column++)
        {
            R04[row][column] = T04[row][column];
        }
    }

    vision_servo_multiply_rotation(
        R04,
        R_mount,
        pose->R_camera_to_base);
}

VisionServoStatus vision_servo_camera1_compute_planar_velocity(
    const VisionServoCamera1Config *config,
    const VisionServoObjectObservation *observation,
    VisionServoVelocity *velocity_base)
{
    double target_u_px;
    double horizontal_error_px;
    double normalized_horizontal_error;
    VisionServoVelocity velocity_camera;

    if (config == NULL ||
        observation == NULL ||
        velocity_base == NULL)
    {
        return VS_ERROR_INVALID_ARGUMENT;
    }

    if (!vision_servo_intrinsics_are_valid(&config->intrinsics))
    {
        return VS_ERROR_INVALID_INTRINSICS;
    }

    if (observation->distance <= 0.0)
    {
        return VS_ERROR_INVALID_DISTANCE;
    }

    target_u_px =
        config->intrinsics.cx +
        config->horizontal_line_offset_px;

    horizontal_error_px =
        observation->u_px -
        target_u_px;

    horizontal_error_px =
        vision_servo_apply_deadband(
            horizontal_error_px,
            config->horizontal_deadband_px);

    normalized_horizontal_error =
        horizontal_error_px /
        config->intrinsics.fx;

    /*
     * Positive error means that the target is to the right.
     * Move the camera to the right to reduce the relative error.
     */
    velocity_camera.vx =
        config->lateral_gain *
        observation->distance *
        normalized_horizontal_error;

    velocity_camera.vy = 0.0;
    velocity_camera.vz = 0.0;

    vision_servo_apply_rotation(
        config->pose.R_camera_to_base,
        &velocity_camera,
        velocity_base);

    /*
     * Camera 1 is used only for planar alignment.
     */
    velocity_base->vz = 0.0;

    vision_servo_limit_planar_norm(
        velocity_base,
        config->maximum_planar_speed);

    return VS_OK;
}

VisionServoStatus vision_servo_camera2_compute_cartesian_velocity(
    const VisionServoCamera2Config *config,
    const VisionServoObjectObservation *observation,
    VisionServoVelocity *velocity_base)
{
    double target_u_px;
    double target_v_px;

    double horizontal_error_px;
    double vertical_error_px;
    double distance_error;

    double normalized_horizontal_error;
    double normalized_vertical_error;

    VisionServoVelocity velocity_camera;

    if (config == NULL ||
        observation == NULL ||
        velocity_base == NULL)
    {
        return VS_ERROR_INVALID_ARGUMENT;
    }

    if (!vision_servo_intrinsics_are_valid(&config->intrinsics))
    {
        return VS_ERROR_INVALID_INTRINSICS;
    }

    if (observation->distance <= 0.0 ||
        config->target_distance <= 0.0)
    {
        return VS_ERROR_INVALID_DISTANCE;
    }

    target_u_px =
        config->intrinsics.cx +
        config->image_offset_u_px;

    target_v_px =
        config->intrinsics.cy +
        config->image_offset_v_px;

    horizontal_error_px =
        observation->u_px -
        target_u_px;

    vertical_error_px =
        observation->v_px -
        target_v_px;

    distance_error =
        observation->distance -
        config->target_distance;

    horizontal_error_px =
        vision_servo_apply_deadband(
            horizontal_error_px,
            config->image_deadband_px);

    vertical_error_px =
        vision_servo_apply_deadband(
            vertical_error_px,
            config->image_deadband_px);

    distance_error =
        vision_servo_apply_deadband(
            distance_error,
            config->distance_deadband);

    normalized_horizontal_error =
        horizontal_error_px /
        config->intrinsics.fx;

    normalized_vertical_error =
        vertical_error_px /
        config->intrinsics.fy;

    /*
     * The target position in the camera image is regulated by moving
     * the camera in the same lateral direction as the observed error.
     *
     * Positive depth error means that the object is too far away.
     * Move the camera forward to approach it.
     */
    velocity_camera.vx =
        config->horizontal_gain *
        observation->distance *
        normalized_horizontal_error;

    velocity_camera.vy =
        config->vertical_gain *
        observation->distance *
        normalized_vertical_error;

    velocity_camera.vz =
        config->depth_gain *
        distance_error;

    velocity_camera.vx =
        vision_servo_clamp(
            velocity_camera.vx,
            -config->maximum_vx,
             config->maximum_vx);

    velocity_camera.vy =
        vision_servo_clamp(
            velocity_camera.vy,
            -config->maximum_vy,
             config->maximum_vy);

    velocity_camera.vz =
        vision_servo_clamp(
            velocity_camera.vz,
            -config->maximum_vz,
             config->maximum_vz);

    vision_servo_apply_rotation(
        config->pose.R_camera_to_base,
        &velocity_camera,
        velocity_base);

    return VS_OK;
}

/* ============================================================
 * Optional command-line demonstration.
 *
 * Compile with:
 *
 *   gcc -Wall -Wextra -std=c11 \
 *       -DVISION_SERVO_BUILD_DEMO \
 *       vision_servo.c -o vision_servo_demo -lm
 * ============================================================ */

#ifdef VISION_SERVO_BUILD_DEMO

static int vision_servo_parse_double(const char *text, double *value)
{
    char *end_pointer;

    if (text == NULL || value == NULL)
    {
        return 0;
    }

    *value = strtod(text, &end_pointer);

    return text != end_pointer &&
           *end_pointer == '\0';
}

static void vision_servo_print_usage(const char *program_name)
{
    printf("Usage:\n\n");

    printf("  %s cam1 object_u_px object_v_px distance q1_deg line_offset_px\n",
           program_name);

    printf("  %s cam2 object_u_px object_v_px distance target_distance\n\n",
           program_name);

    printf("Examples:\n\n");

    printf("  %s cam1 410 245 1.20 30 10\n",
           program_name);

    printf("  %s cam2 370 260 0.80 0.50\n",
           program_name);
}

int main(int argc, char *argv[])
{
    VisionServoObjectObservation observation;
    VisionServoVelocity velocity;
    VisionServoStatus status;

    if (argc < 2)
    {
        vision_servo_print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    if (strcmp(argv[1], "cam1") == 0)
    {
        VisionServoCamera1Config config;
        double q1_degrees;
        double R_mount[3][3];

        if (argc != 7)
        {
            vision_servo_print_usage(argv[0]);
            return EXIT_FAILURE;
        }

        if (!vision_servo_parse_double(argv[2], &observation.u_px) ||
            !vision_servo_parse_double(argv[3], &observation.v_px) ||
            !vision_servo_parse_double(argv[4], &observation.distance) ||
            !vision_servo_parse_double(argv[5], &q1_degrees) ||
            !vision_servo_parse_double(argv[6], &config.horizontal_line_offset_px))
        {
            fprintf(stderr, "Error: invalid numerical argument.\n");
            return EXIT_FAILURE;
        }

        vision_servo_default_camera1_config(&config);
        config.horizontal_line_offset_px = strtod(argv[6], NULL);

        /*
         * Demonstration assumption:
         * the camera frame is initially aligned with the coordinate
         * system immediately after joint 1.
         *
         * Replace R_mount with the real mounting orientation.
         */
        vision_servo_identity_rotation(R_mount);

        vision_servo_update_camera1_pose_from_q1(
            q1_degrees * M_PI / 180.0,
            R_mount,
            &config.pose);

        status = vision_servo_camera1_compute_planar_velocity(
            &config,
            &observation,
            &velocity);

        if (status != VS_OK)
        {
            fprintf(stderr,
                    "Error: %s.\n",
                    vision_servo_status_string(status));

            return EXIT_FAILURE;
        }

        printf("Camera 1 planar velocity in robot base frame:\n");
        printf("  vx = %.6f m/s\n", velocity.vx);
        printf("  vy = %.6f m/s\n", velocity.vy);
        printf("  vz = %.6f m/s\n", velocity.vz);

        return EXIT_SUCCESS;
    }

    if (strcmp(argv[1], "cam2") == 0)
    {
        VisionServoCamera2Config config;

        if (argc != 6)
        {
            vision_servo_print_usage(argv[0]);
            return EXIT_FAILURE;
        }

        if (!vision_servo_parse_double(argv[2], &observation.u_px) ||
            !vision_servo_parse_double(argv[3], &observation.v_px) ||
            !vision_servo_parse_double(argv[4], &observation.distance))
        {
            fprintf(stderr, "Error: invalid numerical argument.\n");
            return EXIT_FAILURE;
        }

        vision_servo_default_camera2_config(&config);

        if (!vision_servo_parse_double(argv[5], &config.target_distance))
        {
            fprintf(stderr, "Error: invalid numerical argument.\n");
            return EXIT_FAILURE;
        }

        /*
         * Demonstration assumption:
         * camera 2 is aligned with the robot-base frame.
         *
         * In the manipulator application, update the pose using:
         *
         *   vision_servo_update_camera2_pose_from_T04(...)
         */
        vision_servo_identity_rotation(
            config.pose.R_camera_to_base);

        status = vision_servo_camera2_compute_cartesian_velocity(
            &config,
            &observation,
            &velocity);

        if (status != VS_OK)
        {
            fprintf(stderr,
                    "Error: %s.\n",
                    vision_servo_status_string(status));

            return EXIT_FAILURE;
        }

        printf("Camera 2 Cartesian velocity in robot base frame:\n");
        printf("  vx = %.6f m/s\n", velocity.vx);
        printf("  vy = %.6f m/s\n", velocity.vy);
        printf("  vz = %.6f m/s\n", velocity.vz);

        return EXIT_SUCCESS;
    }

    fprintf(stderr, "Error: unknown mode '%s'.\n", argv[1]);
    vision_servo_print_usage(argv[0]);

    return EXIT_FAILURE;
}

#endif
