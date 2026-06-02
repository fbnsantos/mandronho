#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

/*
 * Converts a command-line argument to a double.
 * Returns 1 on success and 0 on error.
 */
int parse_double(const char *text, double *value)
{
    char *end_pointer;

    errno = 0;
    *value = strtod(text, &end_pointer);

    /*
     * Detect:
     * - an argument that does not start with a number;
     * - additional invalid characters after the number;
     * - values outside the representable range.
     */
    if (text == end_pointer || *end_pointer != '\0' || errno == ERANGE)
    {
        return 0;
    }

    return 1;
}

/*
 * Computes the homogeneous transformation matrix T04
 * for the DH table:
 *
 *   i | theta | d  | a  | alpha
 *  ---|-------|----|----|------
 *   1 | q1    | d1 | 0  | -90°
 *   2 | q2    | 0  | a2 | 0°
 *   3 | q3    | 0  | 0  | 90°
 *   4 | 0     | d4 | 0  | 0°
 *
 * Angles q1, q2 and q3 must be given in radians.
 */
void compute_T04(double q1,
                 double q2,
                 double q3,
                 double d1,
                 double a2,
                 double d4,
                 double T04[4][4])
{
    const double c1 = cos(q1);
    const double s1 = sin(q1);

    const double c2 = cos(q2);
    const double s2 = sin(q2);

    const double q23 = q2 + q3;
    const double c23 = cos(q23);
    const double s23 = sin(q23);

    const double radial_distance = a2 * c2 + d4 * s23;

    const double px = c1 * radial_distance;
    const double py = s1 * radial_distance;
    const double pz = d1 - a2 * s2 + d4 * c23;

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

int main(int argc, char *argv[])
{
    double T04[4][4];

    double q1_degrees;
    double q2_degrees;
    double q3_degrees;

    double d1;
    double a2;
    double d4;

    /*
     * argv[0] contains the name of the program.
     * The six values provided by the user are stored in argv[1] to argv[6].
     */
    if (argc != 7)
    {
        fprintf(stderr,
                "Usage: %s q1_deg q2_deg q3_deg d1 a2 d4\n"
                "Example: %s 30 45 -20 0.50 0.40 0.25\n",
                argv[0],
                argv[0]);

        return EXIT_FAILURE;
    }

    if (!parse_double(argv[1], &q1_degrees) ||
        !parse_double(argv[2], &q2_degrees) ||
        !parse_double(argv[3], &q3_degrees) ||
        !parse_double(argv[4], &d1) ||
        !parse_double(argv[5], &a2) ||
        !parse_double(argv[6], &d4))
    {
        fprintf(stderr, "Error: all arguments must be valid numerical values.\n");
        return EXIT_FAILURE;
    }

    const double q1 = DEG_TO_RAD(q1_degrees);
    const double q2 = DEG_TO_RAD(q2_degrees);
    const double q3 = DEG_TO_RAD(q3_degrees);

    compute_T04(q1, q2, q3, d1, a2, d4, T04);

    printf("Input values:\n");
    printf("q1 = %.3f deg\n", q1_degrees);
    printf("q2 = %.3f deg\n", q2_degrees);
    printf("q3 = %.3f deg\n", q3_degrees);
    printf("d1 = %.3f\n", d1);
    printf("a2 = %.3f\n", a2);
    printf("d4 = %.3f\n\n", d4);

    printf("T04 =\n");
    print_matrix_4x4(T04);

    printf("\nEnd-effector position:\n");
    printf("x = %.6f\n", T04[0][3]);
    printf("y = %.6f\n", T04[1][3]);
    printf("z = %.6f\n", T04[2][3]);

    return EXIT_SUCCESS;
}
