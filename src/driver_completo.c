#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)

#define MAX_IK_SOLUTIONS 4
#define IK_TOLERANCE 1e-9
#define MATRIX_TOLERANCE 1e-12
#define DEFAULT_DAMPING 1e-6

/*
 * Robot defined by the classical Denavit-Hartenberg table:
 *
 *   i | theta | d  | a  | alpha
 *  ---|-------|----|----|--------
 *   1 | q1    | d1 | 0  | -90 deg
 *   2 | q2    | 0  | a2 |   0 deg
 *   3 | q3    | 0  | 0  |  90 deg
 *   4 | 0     | q4 | 0  |   0 deg
 *
 * Joint types:
 *
 *   q1: revolute
 *   q2: revolute
 *   q3: revolute
 *   q4: prismatic
 */

typedef struct
{
    double q1;
    double q2;
    double q3;
    double q4;
} JointConfiguration;

typedef enum
{
    IK_SUCCESS = 0,
    IK_UNREACHABLE_POSITION,
    IK_INVALID_GEOMETRY,
    IK_SINGULAR_Q1
} IKStatus;

/* ============================================================
 * General-purpose utility functions
 * ============================================================ */

int parse_double(const char *text, double *value)
{
    char *end_pointer;

    errno = 0;
    *value = strtod(text, &end_pointer);

    if (text == end_pointer ||
        *end_pointer != '\0' ||
        errno == ERANGE)
    {
        return 0;
    }

    return 1;
}

double normalize_angle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}

double angular_distance(double angle1, double angle2)
{
    return fabs(normalize_angle(angle1 - angle2));
}

void print_matrix_4x4(const double matrix[4][4])
{
    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            printf("%12.6f ", matrix[row][column]);
        }

        printf("\n");
    }
}

void print_matrix_6x4(const double matrix[6][4])
{
    for (int row = 0; row < 6; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            printf("%12.6f ", matrix[row][column]);
        }

        printf("\n");
    }
}

void print_matrix_3x4(const double matrix[3][4])
{
    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            printf("%12.6f ", matrix[row][column]);
        }

        printf("\n");
    }
}

/* ============================================================
 * Forward kinematics
 * ============================================================ */

/*
 * Computes the homogeneous transformation matrix T04.
 *
 * Input:
 *   q1, q2, q3: revolute-joint angles in radians
 *   q4:         prismatic-joint displacement
 *   d1, a2:     constant robot parameters
 *
 * Output:
 *   T04[4][4]
 */
void compute_T04(double q1,
                 double q2,
                 double q3,
                 double q4,
                 double d1,
                 double a2,
                 double T04[4][4])
{
    const double c1 = cos(q1);
    const double s1 = sin(q1);

    const double c2 = cos(q2);
    const double s2 = sin(q2);

    const double q23 = q2 + q3;
    const double c23 = cos(q23);
    const double s23 = sin(q23);

    const double radial_distance =
        a2 * c2 +
        q4 * s23;

    const double px =
        c1 * radial_distance;

    const double py =
        s1 * radial_distance;

    const double pz =
        d1 -
        a2 * s2 +
        q4 * c23;

    T04[0][0] = c1 * c23;
    T04[0][1] = -s1;
    T04[0][2] = c1 * s23;
    T04[0][3] = px;

    T04[1][0] = s1 * c23;
    T04[1][1] = c1;
    T04[1][2] = s1 * s23;
    T04[1][3] = py;

    T04[2][0] = -s23;
    T04[2][1] = 0.0;
    T04[2][2] = c23;
    T04[2][3] = pz;

    T04[3][0] = 0.0;
    T04[3][1] = 0.0;
    T04[3][2] = 0.0;
    T04[3][3] = 1.0;
}

/*
 * Computes only the end-effector Cartesian position.
 */
void compute_forward_position(double q1,
                              double q2,
                              double q3,
                              double q4,
                              double d1,
                              double a2,
                              double *x,
                              double *y,
                              double *z)
{
    const double q23 = q2 + q3;

    const double radial_distance =
        a2 * cos(q2) +
        q4 * sin(q23);

    *x =
        cos(q1) * radial_distance;

    *y =
        sin(q1) * radial_distance;

    *z =
        d1 -
        a2 * sin(q2) +
        q4 * cos(q23);
}

/* ============================================================
 * Inverse kinematics
 * ============================================================ */

/*
 * Adds a solution only if an equivalent solution is not already
 * present in the array.
 */
void add_unique_solution(JointConfiguration solutions[],
                         int *number_of_solutions,
                         double q1,
                         double q2,
                         double q3,
                         double q4)
{
    q1 = normalize_angle(q1);
    q2 = normalize_angle(q2);
    q3 = normalize_angle(q3);

    for (int index = 0; index < *number_of_solutions; index++)
    {
        if (angular_distance(solutions[index].q1, q1) < IK_TOLERANCE &&
            angular_distance(solutions[index].q2, q2) < IK_TOLERANCE &&
            angular_distance(solutions[index].q3, q3) < IK_TOLERANCE &&
            fabs(solutions[index].q4 - q4) < IK_TOLERANCE)
        {
            return;
        }
    }

    if (*number_of_solutions < MAX_IK_SOLUTIONS)
    {
        solutions[*number_of_solutions].q1 = q1;
        solutions[*number_of_solutions].q2 = q2;
        solutions[*number_of_solutions].q3 = q3;
        solutions[*number_of_solutions].q4 = q4;

        (*number_of_solutions)++;
    }
}

/*
 * Computes q2 and q3 solutions for:
 *
 *   radial = a2*cos(q2) + q4*sin(q2 + q3)
 *   height = -a2*sin(q2) + q4*cos(q2 + q3)
 *
 * q4 must previously be selected because the position-only inverse
 * kinematics problem is redundant when q4 is also free.
 */
void compute_q2_q3_solutions(double q1,
                             double radial,
                             double height,
                             double a2,
                             double q4,
                             JointConfiguration solutions[],
                             int *number_of_solutions)
{
    double sin_q3 =
        (
            radial * radial +
            height * height -
            a2 * a2 -
            q4 * q4
        ) /
        (
            2.0 * a2 * q4
        );

    /*
     * Correct small floating-point errors near workspace boundaries.
     */
    if (sin_q3 > 1.0 &&
        sin_q3 < 1.0 + IK_TOLERANCE)
    {
        sin_q3 = 1.0;
    }

    if (sin_q3 < -1.0 &&
        sin_q3 > -1.0 - IK_TOLERANCE)
    {
        sin_q3 = -1.0;
    }

    if (sin_q3 < -1.0 ||
        sin_q3 > 1.0)
    {
        return;
    }

    const double first_q3 =
        asin(sin_q3);

    /*
     * Since:
     *
     *   sin(q3) = sin(pi - q3)
     *
     * two q3 candidates may exist.
     */
    const double q3_candidates[2] =
    {
        first_q3,
        M_PI - first_q3
    };

    for (int index = 0; index < 2; index++)
    {
        const double q3 =
            normalize_angle(q3_candidates[index]);

        const double A =
            a2 + q4 * sin(q3);

        const double B =
            q4 * cos(q3);

        const double q2 =
            atan2(
                B * radial -
                A * height,

                A * radial +
                B * height
            );

        add_unique_solution(
            solutions,
            number_of_solutions,
            q1,
            q2,
            q3,
            q4
        );
    }
}

/*
 * Computes inverse-kinematics solutions for a selected q4 value.
 *
 * Input:
 *   x, y, z: desired Cartesian position
 *   d1, a2:  constant robot parameters
 *   q4:      selected prismatic-joint displacement
 *
 * Output:
 *   solutions[]: possible joint configurations
 *
 * Important:
 *   q4 is received as an input because position alone does not uniquely
 *   determine all four joint variables.
 */
IKStatus compute_inverse_kinematics_for_q4(
    double x,
    double y,
    double z,
    double d1,
    double a2,
    double q4,
    JointConfiguration solutions[],
    int *number_of_solutions)
{
    *number_of_solutions = 0;

    if (fabs(a2) < IK_TOLERANCE ||
        fabs(q4) < IK_TOLERANCE)
    {
        return IK_INVALID_GEOMETRY;
    }

    const double radius =
        hypot(x, y);

    const double height =
        z - d1;

    /*
     * If x = y = 0, q1 cannot be determined from position alone.
     * Representative solutions with q1 = 0 are returned.
     */
    if (radius < IK_TOLERANCE)
    {
        compute_q2_q3_solutions(
            0.0,
            0.0,
            height,
            a2,
            q4,
            solutions,
            number_of_solutions
        );

        if (*number_of_solutions == 0)
        {
            return IK_UNREACHABLE_POSITION;
        }

        return IK_SINGULAR_Q1;
    }

    /*
     * First family:
     * positive signed radial distance.
     */
    const double q1_first =
        atan2(y, x);

    compute_q2_q3_solutions(
        q1_first,
        radius,
        height,
        a2,
        q4,
        solutions,
        number_of_solutions
    );

    /*
     * Second family:
     * q1 differs by pi and radial distance is negative.
     */
    const double q1_second =
        normalize_angle(q1_first + M_PI);

    compute_q2_q3_solutions(
        q1_second,
        -radius,
        height,
        a2,
        q4,
        solutions,
        number_of_solutions
    );

    if (*number_of_solutions == 0)
    {
        return IK_UNREACHABLE_POSITION;
    }

    return IK_SUCCESS;
}

void print_ik_solutions(const JointConfiguration solutions[],
                        int number_of_solutions,
                        double d1,
                        double a2)
{
    printf("Number of IK solutions: %d\n\n",
           number_of_solutions);

    for (int index = 0;
         index < number_of_solutions;
         index++)
    {
        double x_check;
        double y_check;
        double z_check;

        compute_forward_position(
            solutions[index].q1,
            solutions[index].q2,
            solutions[index].q3,
            solutions[index].q4,
            d1,
            a2,
            &x_check,
            &y_check,
            &z_check
        );

        printf("Solution %d:\n", index + 1);

        printf("  q1 = %10.6f rad = %10.4f deg\n",
               solutions[index].q1,
               RAD_TO_DEG(solutions[index].q1));

        printf("  q2 = %10.6f rad = %10.4f deg\n",
               solutions[index].q2,
               RAD_TO_DEG(solutions[index].q2));

        printf("  q3 = %10.6f rad = %10.4f deg\n",
               solutions[index].q3,
               RAD_TO_DEG(solutions[index].q3));

        printf("  q4 = %10.6f\n",
               solutions[index].q4);

        printf("  FK verification:\n");
        printf("    x = %.6f\n", x_check);
        printf("    y = %.6f\n", y_check);
        printf("    z = %.6f\n", z_check);

        printf("\n");
    }
}

/* ============================================================
 * Jacobian
 * ============================================================ */

/*
 * Computes the complete 6x4 geometric Jacobian:
 *
 *   [ vx     ]
 *   [ vy     ]
 *   [ vz     ]       [ q1_dot ]
 *   [ omega_x] = J * [ q2_dot ]
 *   [ omega_y]       [ q3_dot ]
 *   [ omega_z]       [ q4_dot ]
 *
 * Rows 0 to 2: linear velocity
 * Rows 3 to 5: angular velocity
 */
void compute_geometric_jacobian(double q1,
                                double q2,
                                double q3,
                                double q4,
                                double a2,
                                double J[6][4])
{
    const double c1 = cos(q1);
    const double s1 = sin(q1);

    const double c2 = cos(q2);
    const double s2 = sin(q2);

    const double q23 = q2 + q3;
    const double c23 = cos(q23);
    const double s23 = sin(q23);

    const double radial_distance =
        a2 * c2 +
        q4 * s23;

    const double radial_derivative_q2 =
        -a2 * s2 +
        q4 * c23;

    /*
     * Translational Jacobian Jv.
     */
    J[0][0] = -s1 * radial_distance;
    J[0][1] =  c1 * radial_derivative_q2;
    J[0][2] =  c1 * q4 * c23;
    J[0][3] =  c1 * s23;

    J[1][0] =  c1 * radial_distance;
    J[1][1] =  s1 * radial_derivative_q2;
    J[1][2] =  s1 * q4 * c23;
    J[1][3] =  s1 * s23;

    J[2][0] =  0.0;
    J[2][1] = -radial_distance;
    J[2][2] = -q4 * s23;
    J[2][3] =  c23;

    /*
     * Angular Jacobian Jw.
     *
     * The fourth column is zero because q4 is a prismatic joint.
     */
    J[3][0] =  0.0;
    J[3][1] = -s1;
    J[3][2] = -s1;
    J[3][3] =  0.0;

    J[4][0] =  0.0;
    J[4][1] =  c1;
    J[4][2] =  c1;
    J[4][3] =  0.0;

    J[5][0] =  1.0;
    J[5][1] =  0.0;
    J[5][2] =  0.0;
    J[5][3] =  0.0;
}

/*
 * Extracts the translational 3x4 Jacobian from the complete Jacobian.
 */
void extract_translational_jacobian(const double J[6][4],
                                    double Jv[3][4])
{
    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            Jv[row][column] =
                J[row][column];
        }
    }
}

/* ============================================================
 * Matrix operations for damped least-squares pseudoinverse
 * ============================================================ */

/*
 * Computes the inverse of a 3x3 matrix.
 *
 * Returns:
 *   1 on success
 *   0 if the matrix is singular or nearly singular
 */
int invert_matrix_3x3(const double matrix[3][3],
                      double inverse[3][3])
{
    const double a = matrix[0][0];
    const double b = matrix[0][1];
    const double c = matrix[0][2];

    const double d = matrix[1][0];
    const double e = matrix[1][1];
    const double f = matrix[1][2];

    const double g = matrix[2][0];
    const double h = matrix[2][1];
    const double i = matrix[2][2];

    const double determinant =
        a * (e * i - f * h) -
        b * (d * i - f * g) +
        c * (d * h - e * g);

    if (fabs(determinant) < MATRIX_TOLERANCE)
    {
        return 0;
    }

    inverse[0][0] =  (e * i - f * h) / determinant;
    inverse[0][1] = -(b * i - c * h) / determinant;
    inverse[0][2] =  (b * f - c * e) / determinant;

    inverse[1][0] = -(d * i - f * g) / determinant;
    inverse[1][1] =  (a * i - c * g) / determinant;
    inverse[1][2] = -(a * f - c * d) / determinant;

    inverse[2][0] =  (d * h - e * g) / determinant;
    inverse[2][1] = -(a * h - b * g) / determinant;
    inverse[2][2] =  (a * e - b * d) / determinant;

    return 1;
}

/*
 * Estimates joint velocities using the damped least-squares
 * pseudoinverse:
 *
 *   q_dot =
 *       Jv^T *
 *       inverse(Jv * Jv^T + lambda^2 * I) *
 *       cartesian_velocity
 *
 * Input:
 *   Jv:     translational Jacobian, size 3x4
 *   vx, vy, vz:
 *           desired end-effector linear velocity
 *   lambda: damping coefficient
 *
 * Output:
 *   q_dot[4]
 *
 * Returns:
 *   1 on success
 *   0 if matrix inversion fails
 */
int estimate_joint_velocities(const double Jv[3][4],
                              double vx,
                              double vy,
                              double vz,
                              double lambda,
                              double q_dot[4])
{
    double J_J_transpose[3][3] =
    {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    double regularized_matrix[3][3];
    double inverse_regularized_matrix[3][3];

    const double cartesian_velocity[3] =
    {
        vx,
        vy,
        vz
    };

    double auxiliary_vector[3] =
    {
        0.0,
        0.0,
        0.0
    };

    /*
     * Compute:
     *
     *   Jv * Jv^T
     *
     * Resulting matrix size:
     *
     *   3x4 * 4x3 = 3x3
     */
    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            for (int index = 0; index < 4; index++)
            {
                J_J_transpose[row][column] +=
                    Jv[row][index] *
                    Jv[column][index];
            }
        }
    }

    /*
     * Add damping:
     *
     *   Jv * Jv^T + lambda^2 * I
     */
    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            regularized_matrix[row][column] =
                J_J_transpose[row][column];

            if (row == column)
            {
                regularized_matrix[row][column] +=
                    lambda * lambda;
            }
        }
    }

    if (!invert_matrix_3x3(
            regularized_matrix,
            inverse_regularized_matrix))
    {
        return 0;
    }

    /*
     * Compute:
     *
     *   auxiliary_vector =
     *       inverse(Jv * Jv^T + lambda^2 * I) *
     *       cartesian_velocity
     */
    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            auxiliary_vector[row] +=
                inverse_regularized_matrix[row][column] *
                cartesian_velocity[column];
        }
    }

    /*
     * Compute:
     *
     *   q_dot = Jv^T * auxiliary_vector
     */
    for (int joint = 0; joint < 4; joint++)
    {
        q_dot[joint] = 0.0;

        for (int row = 0; row < 3; row++)
        {
            q_dot[joint] +=
                Jv[row][joint] *
                auxiliary_vector[row];
        }
    }

    return 1;
}

/*
 * Computes the Cartesian velocity produced by the estimated
 * joint velocities:
 *
 *   v_check = Jv * q_dot
 */
void compute_cartesian_velocity_from_joint_velocities(
    const double Jv[3][4],
    const double q_dot[4],
    double *vx,
    double *vy,
    double *vz)
{
    double cartesian_velocity[3] =
    {
        0.0,
        0.0,
        0.0
    };

    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            cartesian_velocity[row] +=
                Jv[row][column] *
                q_dot[column];
        }
    }

    *vx = cartesian_velocity[0];
    *vy = cartesian_velocity[1];
    *vz = cartesian_velocity[2];
}

/* ============================================================
 * Command-line interface
 * ============================================================ */

void print_usage(const char *program_name)
{
    printf("Robot kinematics tool\n\n");

    printf("Available modes:\n\n");

    printf("1. Forward kinematics:\n");
    printf("   %s fk q1_deg q2_deg q3_deg q4 d1 a2\n\n",
           program_name);

    printf("   Example:\n");
    printf("   %s fk 30 45 -20 0.25 0.50 0.40\n\n",
           program_name);

    printf("2. Inverse kinematics for a selected q4:\n");
    printf("   %s ik x y z d1 a2 q4\n\n",
           program_name);

    printf("   Example:\n");
    printf("   %s ik 0.40 0.20 0.60 0.50 0.40 0.25\n\n",
           program_name);

    printf("3. Geometric Jacobian:\n");
    printf("   %s jac q1_deg q2_deg q3_deg q4 a2\n\n",
           program_name);

    printf("   Example:\n");
    printf("   %s jac 30 45 -20 0.25 0.40\n\n",
           program_name);

    printf("4. Joint velocities from a desired Cartesian velocity:\n");
    printf("   %s vel q1_deg q2_deg q3_deg q4 a2 vx vy vz [lambda]\n\n",
           program_name);

    printf("   Example:\n");
    printf("   %s vel 30 45 -20 0.25 0.40 0.10 0.00 -0.05\n\n",
           program_name);

    printf("   Example with custom damping coefficient:\n");
    printf("   %s vel 30 45 -20 0.25 0.40 0.10 0.00 -0.05 0.001\n",
           program_name);
}

/* ============================================================
 * Main
 * ============================================================ */

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    /* --------------------------------------------------------
     * Mode: forward kinematics
     * -------------------------------------------------------- */
    if (strcmp(argv[1], "fk") == 0)
    {
        if (argc != 8)
        {
            fprintf(stderr,
                    "Usage: %s fk q1_deg q2_deg q3_deg q4 d1 a2\n",
                    argv[0]);

            return EXIT_FAILURE;
        }

        double q1_degrees;
        double q2_degrees;
        double q3_degrees;

        double q4;
        double d1;
        double a2;

        if (!parse_double(argv[2], &q1_degrees) ||
            !parse_double(argv[3], &q2_degrees) ||
            !parse_double(argv[4], &q3_degrees) ||
            !parse_double(argv[5], &q4) ||
            !parse_double(argv[6], &d1) ||
            !parse_double(argv[7], &a2))
        {
            fprintf(stderr,
                    "Error: all arguments must be numerical values.\n");

            return EXIT_FAILURE;
        }

        const double q1 =
            DEG_TO_RAD(q1_degrees);

        const double q2 =
            DEG_TO_RAD(q2_degrees);

        const double q3 =
            DEG_TO_RAD(q3_degrees);

        double T04[4][4];

        compute_T04(
            q1,
            q2,
            q3,
            q4,
            d1,
            a2,
            T04
        );

        printf("Joint configuration:\n");
        printf("  q1 = %.6f deg\n", q1_degrees);
        printf("  q2 = %.6f deg\n", q2_degrees);
        printf("  q3 = %.6f deg\n", q3_degrees);
        printf("  q4 = %.6f\n\n", q4);

        printf("Robot geometry:\n");
        printf("  d1 = %.6f\n", d1);
        printf("  a2 = %.6f\n\n", a2);

        printf("Homogeneous transformation matrix T04:\n\n");
        print_matrix_4x4(T04);

        printf("\nEnd-effector position:\n");
        printf("  x = %.6f\n", T04[0][3]);
        printf("  y = %.6f\n", T04[1][3]);
        printf("  z = %.6f\n", T04[2][3]);

        return EXIT_SUCCESS;
    }

    /* --------------------------------------------------------
     * Mode: inverse kinematics
     * -------------------------------------------------------- */
    if (strcmp(argv[1], "ik") == 0)
    {
        if (argc != 8)
        {
            fprintf(stderr,
                    "Usage: %s ik x y z d1 a2 q4\n",
                    argv[0]);

            return EXIT_FAILURE;
        }

        double x;
        double y;
        double z;

        double d1;
        double a2;
        double q4;

        if (!parse_double(argv[2], &x) ||
            !parse_double(argv[3], &y) ||
            !parse_double(argv[4], &z) ||
            !parse_double(argv[5], &d1) ||
            !parse_double(argv[6], &a2) ||
            !parse_double(argv[7], &q4))
        {
            fprintf(stderr,
                    "Error: all arguments must be numerical values.\n");

            return EXIT_FAILURE;
        }

        JointConfiguration solutions[MAX_IK_SOLUTIONS];
        int number_of_solutions;

        printf("Desired Cartesian position:\n");
        printf("  x = %.6f\n", x);
        printf("  y = %.6f\n", y);
        printf("  z = %.6f\n\n", z);

        printf("Robot parameters:\n");
        printf("  d1 = %.6f\n", d1);
        printf("  a2 = %.6f\n", a2);
        printf("  q4 = %.6f  selected prismatic displacement\n\n",
               q4);

        const IKStatus status =
            compute_inverse_kinematics_for_q4(
                x,
                y,
                z,
                d1,
                a2,
                q4,
                solutions,
                &number_of_solutions
            );

        if (status == IK_INVALID_GEOMETRY)
        {
            fprintf(stderr,
                    "Error: a2 and q4 must be different from zero.\n");

            return EXIT_FAILURE;
        }

        if (status == IK_UNREACHABLE_POSITION)
        {
            fprintf(stderr,
                    "Error: the requested position cannot be reached "
                    "with the selected q4 value.\n");

            return EXIT_FAILURE;
        }

        if (status == IK_SINGULAR_Q1)
        {
            printf("Warning: x = y = 0.\n");
            printf("q1 is arbitrary. Representative solutions "
                   "with q1 = 0 are shown.\n\n");
        }

        print_ik_solutions(
            solutions,
            number_of_solutions,
            d1,
            a2
        );

        return EXIT_SUCCESS;
    }

    /* --------------------------------------------------------
     * Mode: Jacobian
     * -------------------------------------------------------- */
    if (strcmp(argv[1], "jac") == 0)
    {
        if (argc != 7)
        {
            fprintf(stderr,
                    "Usage: %s jac q1_deg q2_deg q3_deg q4 a2\n",
                    argv[0]);

            return EXIT_FAILURE;
        }

        double q1_degrees;
        double q2_degrees;
        double q3_degrees;

        double q4;
        double a2;

        if (!parse_double(argv[2], &q1_degrees) ||
            !parse_double(argv[3], &q2_degrees) ||
            !parse_double(argv[4], &q3_degrees) ||
            !parse_double(argv[5], &q4) ||
            !parse_double(argv[6], &a2))
        {
            fprintf(stderr,
                    "Error: all arguments must be numerical values.\n");

            return EXIT_FAILURE;
        }

        const double q1 =
            DEG_TO_RAD(q1_degrees);

        const double q2 =
            DEG_TO_RAD(q2_degrees);

        const double q3 =
            DEG_TO_RAD(q3_degrees);

        double J[6][4];
        double Jv[3][4];

        compute_geometric_jacobian(
            q1,
            q2,
            q3,
            q4,
            a2,
            J
        );

        extract_translational_jacobian(
            J,
            Jv
        );

        printf("Joint configuration:\n");
        printf("  q1 = %.6f deg\n", q1_degrees);
        printf("  q2 = %.6f deg\n", q2_degrees);
        printf("  q3 = %.6f deg\n", q3_degrees);
        printf("  q4 = %.6f\n", q4);
        printf("  a2 = %.6f\n\n", a2);

        printf("Complete geometric Jacobian J, size 6x4:\n\n");
        print_matrix_6x4(J);

        printf("\nTranslational Jacobian Jv, size 3x4:\n\n");
        print_matrix_3x4(Jv);

        return EXIT_SUCCESS;
    }

    /* --------------------------------------------------------
     * Mode: estimate joint velocities
     * -------------------------------------------------------- */
    if (strcmp(argv[1], "vel") == 0)
    {
        if (argc != 10 &&
            argc != 11)
        {
            fprintf(stderr,
                    "Usage: %s vel q1_deg q2_deg q3_deg "
                    "q4 a2 vx vy vz [lambda]\n",
                    argv[0]);

            return EXIT_FAILURE;
        }

        double q1_degrees;
        double q2_degrees;
        double q3_degrees;

        double q4;
        double a2;

        double vx;
        double vy;
        double vz;

        double lambda =
            DEFAULT_DAMPING;

        if (!parse_double(argv[2], &q1_degrees) ||
            !parse_double(argv[3], &q2_degrees) ||
            !parse_double(argv[4], &q3_degrees) ||
            !parse_double(argv[5], &q4) ||
            !parse_double(argv[6], &a2) ||
            !parse_double(argv[7], &vx) ||
            !parse_double(argv[8], &vy) ||
            !parse_double(argv[9], &vz))
        {
            fprintf(stderr,
                    "Error: all arguments must be numerical values.\n");

            return EXIT_FAILURE;
        }

        if (argc == 11)
        {
            if (!parse_double(argv[10], &lambda))
            {
                fprintf(stderr,
                        "Error: lambda must be a numerical value.\n");

                return EXIT_FAILURE;
            }
        }

        if (lambda < 0.0)
        {
            fprintf(stderr,
                    "Error: lambda must be non-negative.\n");

            return EXIT_FAILURE;
        }

        const double q1 =
            DEG_TO_RAD(q1_degrees);

        const double q2 =
            DEG_TO_RAD(q2_degrees);

        const double q3 =
            DEG_TO_RAD(q3_degrees);

        double J[6][4];
        double Jv[3][4];

        double q_dot[4];

        compute_geometric_jacobian(
            q1,
            q2,
            q3,
            q4,
            a2,
            J
        );

        extract_translational_jacobian(
            J,
            Jv
        );

        if (!estimate_joint_velocities(
                Jv,
                vx,
                vy,
                vz,
                lambda,
                q_dot))
        {
            fprintf(stderr,
                    "Error: unable to estimate joint velocities.\n"
                    "Try using a positive damping coefficient, "
                    "for example lambda = 0.001.\n");

            return EXIT_FAILURE;
        }

        double vx_check;
        double vy_check;
        double vz_check;

        compute_cartesian_velocity_from_joint_velocities(
            Jv,
            q_dot,
            &vx_check,
            &vy_check,
            &vz_check
        );

        printf("Joint configuration:\n");
        printf("  q1 = %.6f deg\n", q1_degrees);
        printf("  q2 = %.6f deg\n", q2_degrees);
        printf("  q3 = %.6f deg\n", q3_degrees);
        printf("  q4 = %.6f\n", q4);
        printf("  a2 = %.6f\n\n", a2);

        printf("Desired Cartesian velocity:\n");
        printf("  vx = %.6f\n", vx);
        printf("  vy = %.6f\n", vy);
        printf("  vz = %.6f\n\n", vz);

        printf("Damping coefficient:\n");
        printf("  lambda = %.9f\n\n", lambda);

        printf("Translational Jacobian Jv:\n\n");
        print_matrix_3x4(Jv);

        printf("\nEstimated joint velocities:\n");
        printf("  q1_dot = %12.6f rad/s = %12.6f deg/s\n",
               q_dot[0],
               RAD_TO_DEG(q_dot[0]));

        printf("  q2_dot = %12.6f rad/s = %12.6f deg/s\n",
               q_dot[1],
               RAD_TO_DEG(q_dot[1]));

        printf("  q3_dot = %12.6f rad/s = %12.6f deg/s\n",
               q_dot[2],
               RAD_TO_DEG(q_dot[2]));

        printf("  q4_dot = %12.6f displacement-units/s\n",
               q_dot[3]);

        printf("\nCartesian velocity verification:\n");
        printf("  vx = %.6f\n", vx_check);
        printf("  vy = %.6f\n", vy_check);
        printf("  vz = %.6f\n", vz_check);

        printf("\nVelocity error:\n");
        printf("  ex = %.9f\n", vx - vx_check);
        printf("  ey = %.9f\n", vy - vy_check);
        printf("  ez = %.9f\n", vz - vz_check);

        return EXIT_SUCCESS;
    }

    fprintf(stderr,
            "Error: unknown mode '%s'.\n\n",
            argv[1]);

    print_usage(argv[0]);

    return EXIT_FAILURE;
}
