#include "yesense_driver.h"

int main(int argc, char **argv)
{
    yesense::YesenseDriver yesense_dirver;

    while (1)
    {
        yesense_dirver.run();

        std::cout << "rpy: " << yesense_dirver.rpy[0] << " || " << yesense_dirver.rpy[1] << " || " << yesense_dirver.rpy[2] << std::endl;

        std::cout << "omege: " << yesense_dirver.gyro[0] * M_PI / 180 << " || " << yesense_dirver.gyro[1] * M_PI / 180 << " || " << yesense_dirver.gyro[2] * M_PI / 180 << std::endl;
    }

    return 0;
}