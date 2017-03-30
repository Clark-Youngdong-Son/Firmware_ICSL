px4_add_module(
	MODULE modules__attitude_control
	MAIN attitude_control
	STACK_MAIN 1200
	STACK_MAX 3500
	COMPILE_FLAGS
	SRCS
		attitude_control_main.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
