#include <math.h>
#include <stdio.h>

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

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

    /*
     * Position of the end-effector
     */
    const double radial_distance = a2 * c2 + d4 * s23;

    const double px = c1 * radial_distance;
    const double py = s1 * radial_distance;
    const double pz = d1 - a2 * s2 + d4 * c23;

    /*
     * Rotation matrix
     */
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
            printf("%10.5f ", matrix[row][column]);
        }

        printf("\n");
    }
}

int main(void)
{
    double T04[4][4];

    /*
     * Example values:
     * q1 = 30 degrees
     * q2 = 45 degrees
     * q3 = -20 degrees
     * d1 = 0.50 m
     * a2 = 0.40 m
     * d4 = 0.25 m
     */
    const double q1 = DEG_TO_RAD(30.0);
    const double q2 = DEG_TO_RAD(45.0);
    const double q3 = DEG_TO_RAD(-20.0);

    const double d1 = 0.50;
    const double a2 = 0.40;
    const double d4 = 0.25;

    compute_T04(q1, q2, q3, d1, a2, d4, T04);

    printf("T04 =\n");
    print_matrix_4x4(T04);

    return 0;
}
