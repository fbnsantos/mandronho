#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_IK_SOLUTIONS 4
#define IK_TOLERANCE 1e-9

typedef struct
{
    double q1;
    double q2;
    double q3;
    double q4;
} IKSolution;

typedef enum
{
    IK_SUCCESS = 0,
    IK_UNREACHABLE_POSITION,
    IK_INVALID_GEOMETRY,
    IK_SINGULAR_Q1
} IKStatus;

/*
 * Converts text to double.
 */
int parse_double(const char *text, double *value)
{
    char *end_pointer;

    errno = 0;
    *value = strtod(text, &end_pointer);

    if (text == end_pointer || *end_pointer != '\0' || errno == ERANGE)
    {
        return 0;
    }

    return 1;
}

/*
 * Normalizes an angle to [-pi, pi].
 */
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

double rad_to_deg(double angle)
{
    return angle * 180.0 / M_PI;
}

double angular_distance(double angle1, double angle2)
{
    return fabs(normalize_angle(angle1 - angle2));
}

/*
 * Adds a new solution only if it is distinct from previous solutions.
 */
void add_unique_solution(IKSolution solutions[],
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
 * Forward kinematics: computes only the Cartesian position.
 *
 * q4 is the displacement of the prismatic joint.
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

    *x = cos(q1) * radial_distance;
    *y = sin(q1) * radial_distance;
    *z = d1 - a2 * sin(q2) + q4 * cos(q23);
}

/*
 * Computes q2 and q3 solutions for a specified q4.
 */
void compute_q2_q3_solutions(double q1,
                             double radial,
                             double height,
                             double a2,
                             double q4,
                             IKSolution solutions[],
                             int *number_of_solutions)
{
    double sin_q3 =
        (radial * radial +
         height * height -
         a2 * a2 -
         q4 * q4) /
        (2.0 * a2 * q4);

    /*
     * Correct minor floating-point errors near workspace boundaries.
     */
    if (sin_q3 > 1.0 && sin_q3 < 1.0 + IK_TOLERANCE)
    {
        sin_q3 = 1.0;
    }

    if (sin_q3 < -1.0 && sin_q3 > -1.0 - IK_TOLERANCE)
    {
        sin_q3 = -1.0;
    }

    if (sin_q3 < -1.0 || sin_q3 > 1.0)
    {
        return;
    }

    const double first_q3 = asin(sin_q3);

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
            atan2(B * radial - A * height,
                  A * radial + B * height);

        add_unique_solution(solutions,
                            number_of_solutions,
                            q1,
                            q2,
                            q3,
                            q4);
    }
}

/*
 * Computes inverse-kinematics solutions for a specified value of q4.
 *
 * Since q4 is variable, this function receives q4 as an input.
 */
IKStatus compute_inverse_kinematics_for_q4(double x,
                                           double y,
                                           double z,
                                           double d1,
                                           double a2,
                                           double q4,
                                           IKSolution solutions[],
                                           int *number_of_solutions)
{
    *number_of_solutions = 0;

    if (fabs(a2) < IK_TOLERANCE || fabs(q4) < IK_TOLERANCE)
    {
        return IK_INVALID_GEOMETRY;
    }

    const double radius = hypot(x, y);
    const double height = z - d1;

    /*
     * Singular case: q1 is arbitrary if x = y = 0.
     */
    if (radius < IK_TOLERANCE)
    {
        compute_q2_q3_solutions(0.0,
                                0.0,
                                height,
                                a2,
                                q4,
                                solutions,
                                number_of_solutions);

        if (*number_of_solutions == 0)
        {
            return IK_UNREACHABLE_POSITION;
        }

        return IK_SINGULAR_Q1;
    }

    /*
     * First family: positive signed radial distance.
     */
    const double q1_first =
        atan2(y, x);

    compute_q2_q3_solutions(q1_first,
                            radius,
                            height,
                            a2,
                            q4,
                            solutions,
                            number_of_solutions);

    /*
     * Second family: negative signed radial distance.
     */
    const double q1_second =
        normalize_angle(q1_first + M_PI);

    compute_q2_q3_solutions(q1_second,
                            -radius,
                            height,
                            a2,
                            q4,
                            solutions,
                            number_of_solutions);

    if (*number_of_solutions == 0)
    {
        return IK_UNREACHABLE_POSITION;
    }

    return IK_SUCCESS;
}

void print_ik_solutions(const IKSolution solutions[],
                        int number_of_solutions,
                        double d1,
                        double a2)
{
    printf("Number of IK solutions: %d\n\n", number_of_solutions);

    for (int index = 0; index < number_of_solutions; index++)
    {
        double x_check;
        double y_check;
        double z_check;

        compute_forward_position(solutions[index].q1,
                                 solutions[index].q2,
                                 solutions[index].q3,
                                 solutions[index].q4,
                                 d1,
                                 a2,
                                 &x_check,
                                 &y_check,
                                 &z_check);

        printf("Solution %d:\n", index + 1);

        printf("  q1 = %10.6f rad = %10.4f deg\n",
               solutions[index].q1,
               rad_to_deg(solutions[index].q1));

        printf("  q2 = %10.6f rad = %10.4f deg\n",
               solutions[index].q2,
               rad_to_deg(solutions[index].q2));

        printf("  q3 = %10.6f rad = %10.4f deg\n",
               solutions[index].q3,
               rad_to_deg(solutions[index].q3));

        printf("  q4 = %10.6f\n",
               solutions[index].q4);

        printf("  FK verification:\n");
        printf("    x = %.6f\n", x_check);
        printf("    y = %.6f\n", y_check);
        printf("    z = %.6f\n", z_check);

        printf("\n");
    }
}

int main(int argc, char *argv[])
{
    double x;
    double y;
    double z;

    double d1;
    double a2;
    double q4;

    IKSolution solutions[MAX_IK_SOLUTIONS];
    int number_of_solutions;

    /*
     * argv[0] = program name
     * argv[1] = x
     * argv[2] = y
     * argv[3] = z
     * argv[4] = d1
     * argv[5] = a2
     * argv[6] = q4
     */
    if (argc != 7)
    {
        fprintf(stderr,
                "Usage: %s x y z d1 a2 q4\n"
                "\n"
                "Example:\n"
                "  %s 0.40 0.20 0.60 0.50 0.40 0.25\n",
                argv[0],
                argv[0]);

        return EXIT_FAILURE;
    }

    if (!parse_double(argv[1], &x) ||
        !parse_double(argv[2], &y) ||
        !parse_double(argv[3], &z) ||
        !parse_double(argv[4], &d1) ||
        !parse_double(argv[5], &a2) ||
        !parse_double(argv[6], &q4))
    {
        fprintf(stderr,
                "Error: all arguments must be valid numerical values.\n");

        return EXIT_FAILURE;
    }

    printf("Desired end-effector position:\n");
    printf("  x  = %.6f\n", x);
    printf("  y  = %.6f\n", y);
    printf("  z  = %.6f\n\n", z);

    printf("Robot parameters:\n");
    printf("  d1 = %.6f\n", d1);
    printf("  a2 = %.6f\n", a2);
    printf("  q4 = %.6f  (selected prismatic-joint displacement)\n\n", q4);

    const IKStatus status =
        compute_inverse_kinematics_for_q4(x,
                                          y,
                                          z,
                                          d1,
                                          a2,
                                          q4,
                                          solutions,
                                          &number_of_solutions);

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
                "with the specified q4 value.\n");

        return EXIT_FAILURE;
    }

    if (status == IK_SINGULAR_Q1)
    {
        printf("Warning: singular position detected.\n");
        printf("Since x = y = 0, q1 is arbitrary.\n");
        printf("Representative solutions with q1 = 0 are shown.\n\n");
    }

    print_ik_solutions(solutions,
                       number_of_solutions,
                       d1,
                       a2);

    return EXIT_SUCCESS;
}
