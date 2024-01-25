#pragma once

#include <px4_platform_common/app.h>

class AIController
{
public:
	AIController() {}
	~AIController() {}

	int main();

	static px4::AppState appState;
};
