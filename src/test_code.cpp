#include <stdio.h>
#include <math.h>

int main(int argc, char* argv[])
{
    int x_1, x_2, y_1, y_2;
    sscanf(argv[1], "%d", &x_1);
    sscanf(argv[2], "%d", &y_1);
    sscanf(argv[3], "%d", &x_2);
    sscanf(argv[4], "%d", &y_2);

    float angle_rad = atan2(y_2 - y_1, x_2 - x_1);
    printf("(%d, %d) -> (%d, %d)", x_1, y_1, x_2, y_2);
    if(angle_rad >= 0)
    {
        printf("%f\n", angle_rad* 180 / M_PI);
    }
    else
    {
        printf("%f\n", ((2*M_PI + angle_rad)*180/M_PI));
    }
}