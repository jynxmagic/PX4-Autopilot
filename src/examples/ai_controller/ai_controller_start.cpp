#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/tasks.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>
#include "ai_controller.h"
#include "ai_controller_main.cpp"

static int daemon_task;

extern "C" __EXPORT int ai_controller_main(int argc, char *argv[]);
int ai_controller_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: ai_controller {start|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (AIController::appState.isRunning()) {
			PX4_INFO("ai controller is already running\n");
			/* this is not an error */
			return 0;
		}

		daemon_task = px4_task_spawn_cmd("ai_controller",
						 1,
						 5,
						 6000,
						 ai_controller_app_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		AIController::appState.requestExit();
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (AIController::appState.isRunning()) {
			PX4_INFO("ai controller is running\n");

		} else {
			PX4_INFO("ai controller not started\n");
		}

		return 0;
	}


	PX4_WARN("usage: ai_controller {start|stop|status}\n");
	return 1;
};
