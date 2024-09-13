#include "src/Logitech_controller.h"

using namespace std;

int main()
{
    Logitech gamepad;

    gamepad.init();

    while (1)
    {
        int x = gamepad.listen_input();
        (void)x;

        gamepad.print_key_state();
        usleep(2000);
    }

    return 0;
}
