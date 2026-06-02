#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

/*
 * Converts a text argument to double.
 * Returns 1 on success and 0 on error.
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
 * Computes the 6x4 geometric Jacobian.
 *
 * Joint variables:
 *   q1, q2, q3 : revolute-joint angles in radians
 *   q4         : prismatic-joint displacement
 *
 * Constant geometric parameter:
 *   a2         : link length
 *
 * Output:
 *   J[6][4]
 *
 * Rows 0 to 2: linear velocity component
 * Rows 3 to 5: angular velocity component
 */
void compute_jacobian(double q1,
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

    /*
     * Auxiliary values:
     *
     * r = radial distance of the end-effector from the z0 axis
     *
     * k = partial derivative of r with respect to q2
     */
    const double r =
        a2 * c2 +
        q4 * s23;

    const double k =
        -a2 * s2 +
        q4 * c23;

    /*
     * Linear-velocity Jacobian Jv
     */
    J[0][0] = -s1 * r;
    J[0][1] =  c1 * k;
    J[0][2] =  c1 * q4 * c23;
    J[0][3] =  c1 * s23;

    J[1][0] =  c1 * r;
    J[1][1] =  s1 * k;
    J[1][2] =  s1 * q4 * c23;
    J[1][3] =  s1 * s23;

    J[2][0] =  0.0;
    J[2][1] = -r;
    J[2][2] = -q4 * s23;
    J[2][3] =  c23;

    /*
     * Angular-velocity Jacobian Jw
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
 * Prints a 6x4 matrix.
 */
void print_jacobian(const double J[6][4])
{
    printf("Geometric Jacobian J =\n\n");

    for (int row = 0; row < 6; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            printf("%12.6f ", J[row][column]);
        }

        printf("\n");
    }
}

int main(int argc, char *argv[])
{
    double q1_degrees;
    double q2_degrees;
    double q3_degrees;

    double q4;
    double a2;

    double J[6][4];

    /*
     * argv[0] = program name
     * argv[1] = q1 in degrees
     * argv[2] = q2 in degrees
     * argv[3] = q3 in degrees
     * argv[4] = q4 prismatic displacement
     * argv[5] = a2 link length
     */
    if (argc != 6)
    {
        fprintf(stderr,
                "Usage: %s q1_deg q2_deg q3_deg q4 a2\n"
                "\n"
                "Example:\n"
                "  %s 30 45 -20 0.25 0.40\n",
                argv[0],
                argv[0]);

        return EXIT_FAILURE;
    }

    if (!parse_double(argv[1], &q1_degrees) ||
        !parse_double(argv[2], &q2_degrees) ||
        !parse_double(argv[3], &q3_degrees) ||
        !parse_double(argv[4], &q4) ||
        !parse_double(argv[5], &a2))
    {
        fprintf(stderr,
                "Error: all arguments must be valid numerical values.\n");

        return EXIT_FAILURE;
    }

    const double q1 = DEG_TO_RAD(q1_degrees);
    const double q2 = DEG_TO_RAD(q2_degrees);
    const double q3 = DEG_TO_RAD(q3_degrees);

    printf("Joint variables:\n");
    printf("  q1 = %.6f deg = %.6f rad\n", q1_degrees, q1);
    printf("  q2 = %.6f deg = %.6f rad\n", q2_degrees, q2);
    printf("  q3 = %.6f deg = %.6f rad\n", q3_degrees, q3);
    printf("  q4 = %.6f\n\n", q4);

    printf("Robot parameter:\n");
    printf("  a2 = %.6f\n\n", a2);

    compute_jacobian(q1,
                     q2,
                     q3,
                     q4,
                     a2,
                     J);

    print_jacobian(J);

    return EXIT_SUCCESS;
}
