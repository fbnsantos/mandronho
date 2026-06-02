#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Classical DH model:
 * 1: theta=q1, d=d1, a=0,  alpha=-90 deg  (rotary actuator)
 * 2: theta=q2, d=0,  a=a2, alpha=0       (linear actuator)
 * 3: theta=q3, d=0,  a=0,  alpha=90 deg  (linear actuator)
 * 4: theta=0,  d=q4, a=0,  alpha=0       (prismatic actuator)
 */

/* ===== Default robot parameters: edit these values ===== */
#define MANIP_DEFAULT_D1                       1.000000
#define MANIP_DEFAULT_A2                       1.00000
#define MANIP_DEFAULT_Q1_MAX_VELOCITY          0.500000 /* rad/s */
#define MANIP_DEFAULT_Q2_LINEAR_MAX_VELOCITY   0.050000 /* m/s */
#define MANIP_DEFAULT_Q3_LINEAR_MAX_VELOCITY   0.050000 /* m/s */
#define MANIP_DEFAULT_Q4_MIN_POSITION          0.100000 /* m */
#define MANIP_DEFAULT_Q4_MAX_POSITION          2.000000 /* m */
#define MANIP_DEFAULT_Q4_MAX_VELOCITY          0.050000 /* m/s */
#define MANIP_DEFAULT_DAMPING                  0.001000

/* Joint 2 triangular linkage: phi = direction*q + angle_offset */
#define MANIP_DEFAULT_Q2_LINK_FIXED_1          0.300000
#define MANIP_DEFAULT_Q2_LINK_FIXED_2          0.250000
#define MANIP_DEFAULT_Q2_LINK_ANGLE_OFFSET     0.000000
#define MANIP_DEFAULT_Q2_LINK_DIRECTION        1.000000
#define MANIP_DEFAULT_Q2_ACTUATOR_MIN_LENGTH   0.080000
#define MANIP_DEFAULT_Q2_ACTUATOR_MAX_LENGTH   0.500000

/* Joint 3 triangular linkage */
#define MANIP_DEFAULT_Q3_LINK_FIXED_1          0.280000
#define MANIP_DEFAULT_Q3_LINK_FIXED_2          0.200000
#define MANIP_DEFAULT_Q3_LINK_ANGLE_OFFSET     0.000000
#define MANIP_DEFAULT_Q3_LINK_DIRECTION        1.000000
#define MANIP_DEFAULT_Q3_ACTUATOR_MIN_LENGTH   0.080000
#define MANIP_DEFAULT_Q3_ACTUATOR_MAX_LENGTH   0.450000

#define MANIP_MAX_IK_SOLUTIONS 4

typedef enum {
    MANIP_OK = 0,
    MANIP_ERROR_INVALID_ARGUMENT,
    MANIP_ERROR_INVALID_GEOMETRY,
    MANIP_ERROR_OUTSIDE_LIMITS,
    MANIP_ERROR_UNREACHABLE,
    MANIP_ERROR_SINGULAR,
    MANIP_ERROR_NUMERICAL
} ManipulatorStatus;

typedef struct {
    double fixed_length_1;
    double fixed_length_2;
    double angle_offset;
    double direction;
    double minimum_length;
    double maximum_length;
    double maximum_velocity;
} ManipulatorLinearActuatorGeometry;

typedef struct {
    double d1;
    double a2;
    double q1_maximum_velocity;
    double q4_minimum_position;
    double q4_maximum_position;
    double q4_maximum_velocity;
    double damping;
    ManipulatorLinearActuatorGeometry q2_actuator;
    ManipulatorLinearActuatorGeometry q3_actuator;
} ManipulatorConfig;

typedef struct {
    double q1, q2, q3, q4;
    double q2_actuator_length;
    double q3_actuator_length;
    double x, y, z;
    double T04[4][4];
    double J[6][4];
    double Jv[3][4];
} ManipulatorState;

typedef struct { double vx, vy, vz; } ManipulatorCartesianVelocity;
typedef struct { double q1_dot, q2_dot, q3_dot, q4_dot; } ManipulatorJointVelocity;
typedef struct {
    double q1_rotary_velocity;
    double q2_linear_velocity;
    double q3_linear_velocity;
    double q4_linear_velocity;
} ManipulatorActuatorCommand;
typedef struct { double q1, q2, q3, q4; } ManipulatorIKSolution;

void manipulator_default_config(ManipulatorConfig *config);
ManipulatorStatus manipulator_validate_config(const ManipulatorConfig *config);
const char *manipulator_status_string(ManipulatorStatus status);

ManipulatorStatus manipulator_joint_angle_to_linear_length(double q, const ManipulatorLinearActuatorGeometry *g, double *length);
ManipulatorStatus manipulator_linear_length_to_joint_angle(double length, const ManipulatorLinearActuatorGeometry *g, double *q);
ManipulatorStatus manipulator_joint_velocity_to_linear_velocity(double q, double q_dot, const ManipulatorLinearActuatorGeometry *g, double *length_dot);
ManipulatorStatus manipulator_linear_velocity_to_joint_velocity(double q, double length_dot, const ManipulatorLinearActuatorGeometry *g, double *q_dot);

void manipulator_compute_forward_matrix(const ManipulatorConfig *config, double q1, double q2, double q3, double q4, double T04[4][4]);
void manipulator_compute_geometric_jacobian(const ManipulatorConfig *config, double q1, double q2, double q3, double q4, double J[6][4]);
void manipulator_extract_translational_jacobian(const double J[6][4], double Jv[3][4]);
ManipulatorStatus manipulator_update_state_from_joints(const ManipulatorConfig *config, double q1, double q2, double q3, double q4, ManipulatorState *state);
ManipulatorStatus manipulator_update_state_from_actuators(const ManipulatorConfig *config, double q1, double lq2, double lq3, double q4, ManipulatorState *state);
ManipulatorStatus manipulator_inverse_position_for_q4(const ManipulatorConfig *config, double x, double y, double z, double q4, ManipulatorIKSolution solutions[], size_t *count);

ManipulatorStatus manipulator_estimate_joint_velocity(const ManipulatorConfig *config, const ManipulatorState *state, ManipulatorCartesianVelocity desired, ManipulatorJointVelocity *qdot);
ManipulatorStatus manipulator_joint_velocity_to_actuator_command(const ManipulatorConfig *config, const ManipulatorState *state, ManipulatorJointVelocity requested, ManipulatorActuatorCommand *command, ManipulatorJointVelocity *saturated);
ManipulatorStatus manipulator_velocity_control_step(const ManipulatorConfig *config, ManipulatorState *state, ManipulatorCartesianVelocity desired, double dt, ManipulatorActuatorCommand *command, ManipulatorJointVelocity *applied);
void manipulator_cartesian_velocity_from_joint_velocity(const ManipulatorState *state, ManipulatorJointVelocity qdot, ManipulatorCartesianVelocity *velocity);

#ifdef __cplusplus
}
#endif
#endif
