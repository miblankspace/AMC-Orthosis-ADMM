#include "microphone.h"
#include "edge_impulse.h"

extern "C" void app_main()
{
    init_i2s();

    edge_impulse_init();


    while(1)
    {
        edge_impulse_run();
    }
}