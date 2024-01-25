############################################################################
#
#   Copyright (c) 2023 Loughborough University. All rights reserved.
#
############################################################################


px4_add_module(
	MODULE examples__ai_controller
	MAIN ai_controller
	STACK_MAIN 6000
	SRCS
		ai_controller_main.cpp
		ai_controller_start.cpp
		ai_controller.cpp
	DEPENDS
)
