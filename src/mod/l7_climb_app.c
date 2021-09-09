/*
===============================================================================
 Name        : l7_climb_app.c
 Author      : Robert
 Created on	 : 07.09.2021
===============================================================================
*/
#include "l7_climb_app.h"

#include <string.h>
#include "l2_debug_com.h"


// Prototypes
void app_processCmd(int argc, char *argv[]);

void app_init (void *dummy) {

}

void app_main (void) {
	// Debug Command Polling (direct from L2 CLI Module)
	DEB_L2_CMD_T cmd;
	if ( deb_getCommandIfAvailable(&cmd) ) {
		app_processCmd(cmd.parCnt, cmd.pars);
	}

	// handle event - queue ....
}

void app_processCmd(int argc, char *argv[]) {
	// Dummy processing. Send pars in reverse order to L2 Debug (one frame per par)
	for (int i = argc; i>=0; i--) {
		if (!deb_sendFrame((uint8_t *)&argv[i], strlen(argv[i]))) {
			// Error !? buffer full or something else
			// Signal event here !? Or rely on event from lower layer itself !?
			break;
		}
	}
}
