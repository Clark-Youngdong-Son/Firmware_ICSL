px4_add_module(
	MODULE modules__position_control
	MAIN position_control
	COMPILE_FLAGS
	STACK_MAIN 1200
	SRCS
		position_control_main.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
