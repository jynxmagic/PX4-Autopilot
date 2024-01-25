#include "ai_controller.h"
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>

px4::AppState AIController::appState;

int AIController::main()
{
	appState.setRunning(true);

	int i = 0;

	while (!appState.exitRequested() && i < 5) {
		px4_sleep(2);

		printf("Doint work...\n");

		++i;
	}

	return 0;
}
