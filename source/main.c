#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC55S28.h"
#include "fsl_debug_console.h"

#include "ams.h"

int main(void) {

	AMS_DEVICE dev;
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("Hello World\n");

    ams_init();
    ams_read_info_register(&dev);
    ams_print_device(&dev);

    return 0 ;
}
