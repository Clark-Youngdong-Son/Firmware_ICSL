#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/battery_status.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/mavlink_log.h>

#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

extern "C" __EXPORT int attitude_control_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

class AttitudeControl
{
public:
	
	AttitudeControl();

	~AttitudeControl();

	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */
	orb_advert_t 	_mavlink_log_pub;

	/** subscribers **/
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int 	_motor_limits_sub;		/**< motor limits subscription */
	int 	_battery_status_sub;	/**< battery status subscription */

	/** publishers **/
	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */

	orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct control_state_s				_ctrl_state;		/**< control state */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct multirotor_motor_limits_s	_motor_limits;		/**< motor limits */
	struct mc_att_ctrl_status_s 		_controller_status; /**< controller status */
	struct battery_status_s				_battery_status;	/**< battery status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Matrix<3, 3>  _I;				/**< identity matrix */

	math::Matrix<6, 6>  _A;
	math::Matrix<6, 3>  _B;
	math::Vector<6>		_p;
	math::Vector<6> 	_q;
	float 				_Lambda;
	float				_G;
	float				_cmd2torque;
	float 				_yaw_init;

	struct 
	{
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t roll_rate_ff;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t tpa_breakpoint;
		param_t tpa_slope;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;
		param_t yaw_auto_max;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t rattitude_thres;

		param_t vtol_type;
		param_t roll_tc;
		param_t pitch_tc;
		param_t vtol_opt_recovery_enabled;
		param_t vtol_wv_yaw_rate_scale;

		param_t bat_scale_en;

	}		_params_handles;		/**< handles for interesting parameters */

	struct 
	{
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
		float yaw_ff;						/**< yaw control feed-forward */

		float tpa_breakpoint;				/**< Throttle PID Attenuation breakpoint */
		float tpa_slope;					/**< Throttle PID Attenuation slope */

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		float yaw_auto_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */
		math::Vector<3> auto_rate_max;		/**< attitude rate limits in auto modes */
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */
		float rattitude_thres;
		int vtol_type;						/**< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
		bool vtol_opt_recovery_enabled;
		float vtol_wv_yaw_rate_scale;			/**< Scale value [0, 1] for yaw rate setpoint  */

		int bat_scale_en;
	}		_params;

	void		control(float dt);
	int			parameters_update();
	void		parameter_update_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_attitude_setpoint_poll();
	void		vehicle_rates_setpoint_poll();
	void		arming_status_poll();
	void		vehicle_status_poll();
	void		vehicle_motor_limits_poll();
	void		battery_status_poll();
	float 		calc_yaw(float yaw);

	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main();
};

namespace attitude_control
{

AttitudeControl	*g_control;
}

AttitudeControl::AttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),
	_mavlink_log_pub(nullptr),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_vehicle_status_sub(-1),

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
	_rates_sp_id(0),
	_actuators_id(0),

	_actuators_0_circuit_breaker_enabled(false),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "attitude_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

{
	memset(&_ctrl_state, 0, sizeof(_ctrl_state));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
	memset(&_motor_limits, 0, sizeof(_motor_limits));
	memset(&_controller_status, 0, sizeof(_controller_status));
	_vehicle_status.is_rotary_wing = true;

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.auto_rate_max.zero();
	_params.acro_rate_max.zero();
	_params.rattitude_thres = 1.0f;
	_params.vtol_opt_recovery_enabled = false;
	_params.vtol_wv_yaw_rate_scale = 1.0f;
	_params.bat_scale_en = 0;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();
	_A.zero();
	_B.zero();
	_p.zero();
	_q.zero();
	_Lambda = 0.1;
	_G = 100;
	_cmd2torque = 10;
	_yaw_init = 0;

	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff	= 	param_find("MC_ROLLRATE_FF");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 	= 	param_find("MC_PITCHRATE_FF");
	_params_handles.tpa_breakpoint 	= 	param_find("MC_TPA_BREAK");
	_params_handles.tpa_slope	 	= 	param_find("MC_TPA_SLOPE");
	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	_params_handles.roll_rate_max	= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max	= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX");
	_params_handles.yaw_auto_max	= 	param_find("MC_YAWRAUTO_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.rattitude_thres = 	param_find("MC_RATT_TH");
	_params_handles.vtol_type 		= 	param_find("VT_TYPE");
	_params_handles.roll_tc			= 	param_find("MC_ROLL_TC");
	_params_handles.pitch_tc		= 	param_find("MC_PITCH_TC");
	_params_handles.vtol_opt_recovery_enabled	= param_find("VT_OPT_RECOV_EN");
	_params_handles.vtol_wv_yaw_rate_scale		= param_find("VT_WV_YAWR_SCL");
	_params_handles.bat_scale_en		= param_find("MC_BAT_SCALE_EN");

	/* fetch initial parameter values */
	parameters_update();
}

AttitudeControl::~AttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	attitude_control::g_control = nullptr;
}

int AttitudeControl::parameters_update()
{
	float v;

	float roll_tc, pitch_tc;

	param_get(_params_handles.roll_tc, &roll_tc);
	param_get(_params_handles.pitch_tc, &pitch_tc);

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	param_get(_params_handles.tpa_breakpoint, &v);
	_params.tpa_breakpoint = v;
	param_get(_params_handles.tpa_slope, &v);
	_params.tpa_slope = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* auto angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
	_params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

	/* manual rate control scale and auto mode roll/pitch rate limits */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	/* stick deflection needed in rattitude mode to control rates not angles */
	param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

	param_get(_params_handles.vtol_type, &_params.vtol_type);

	int tmp;
	param_get(_params_handles.vtol_opt_recovery_enabled, &tmp);
	_params.vtol_opt_recovery_enabled = (bool)tmp;

	param_get(_params_handles.vtol_wv_yaw_rate_scale, &_params.vtol_wv_yaw_rate_scale);

	param_get(_params_handles.bat_scale_en, &_params.bat_scale_en);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);


	//// DOB settings
	float a0 = _params.rate_i(0); // roll rate i (1.0)
	float a1 = _params.rate_i(1); // pitch rate i (2.0)
	float tau = _params.rate_i(2); // yaw rate i (0.09)

	_A(0,1) = 1.0; _A(2,3) = 1.0; _A(4,5) = 1.0;
	_A(1,0) = -a0/(tau*tau); _A(1,1) = -a1/tau;
	_A(3,2) = -a0/(tau*tau); _A(3,3) = -a1/tau;
	_A(5,4) = -a0/(tau*tau); _A(5,5) = -a1/tau;

	_B(1,0) = a0/(tau*tau); 
	_B(3,1) = a0/(tau*tau);
	_B(5,2) = a0/(tau*tau);

	return OK;
}

void AttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void AttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void AttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void AttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void AttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void AttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void AttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void AttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
	}
}

void AttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

float AttitudeControl::calc_yaw(float yaw)
{
	float yaw_new;

	if(_yaw_init >= 0)
	{
		if(yaw >= (-3.14159265f + _yaw_init) && yaw <= 3.14159265f)
			yaw_new = yaw - _yaw_init;
		else
			yaw_new = yaw - _yaw_init + 2.0f * 3.14159265f; 
	}
	else
	{
		if(yaw >= -3.14159265f && yaw <= (3.14159265f + _yaw_init))
			yaw_new = yaw - _yaw_init;
		else
			yaw_new = yaw - _yaw_init - 2.0f * 3.14159265f; 
	}

	return yaw_new;
}

void AttitudeControl::control(float dt)
{
	/* INPUT : poll setpoint attitude from "mc_pos_control" */
	vehicle_attitude_setpoint_poll(); 	// saves information in _v_att_sp
	_thrust_sp = _v_att_sp.thrust; 		// thrust setpoint from "mc_pos_control"

	/* construct attitude setpoint rotation matrix */
	math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
	math::Vector<3> rpy_sp = q_sp.to_euler();
	/* rotation setpoint to Euler setpoint with box constraints */
	for (int i=0; i<2; i++)
		rpy_sp(i) = math::constrain(rpy_sp(i),math::radians(-20.0f),math::radians(20.0f));

//	mavlink_log_info(&_mavlink_log_pub, 
//		"[sp] roll %2.4f, pitch %2.4f, yaw %2.4f",
//		(double)math::degrees(rpy_sp(0)),
//		(double)math::degrees(rpy_sp(1)),
//		(double)math::degrees(rpy_sp(2)));
	
	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Vector<3> rpy = q_att.to_euler();

	/* check for arming and thrust */
	if (!_armed.armed || _thrust_sp < MIN_TAKEOFF_THRUST) {
		/* P, Q filter initialization */
		_p.zero(); 
		_q.zero();
		_q(0) = rpy(0); _q(2) = rpy(1); _q(4) = rpy(2); 
		_yaw_init = rpy(2);
	}

	rpy(2) = calc_yaw(rpy(2)); /* conventional way to compute yaw in px4? */
	rpy_sp(2) = calc_yaw(rpy_sp(2)); /* conventional way to compute yaw in px4? */

	math::Vector<3> att_err = rpy_sp - rpy;

	/* current Euler angle rates */
	math::Vector<3> rates;
	rates(0) = _ctrl_state.roll_rate;
	rates(1) = _ctrl_state.pitch_rate;
	rates(2) = _ctrl_state.yaw_rate;

	/* DOB structure */
	math::Vector<3> tau0 = _params.att_p.emult(rpy_sp - rpy) + _params.rate_p.emult(-rates);
	math::Vector<3> tau1 = tau0*(_cmd2torque*_Lambda*_G);
	math::Vector<6> dq = _A*_q + _B*rpy;
	math::Vector<3> dq2; dq2(0) = dq(1); dq2(1) = dq(3); dq2(2) = dq(5);
	math::Vector<3> p1; p1(0) = _p(0); p1(1) = _p(2); p1(2) = _p(4);
	math::Vector<3> U = p1 - dq2*_Lambda;
	U(0) = math::constrain(U(0), -5.0f, 5.0f);
	U(1) = math::constrain(U(1), -5.0f, 5.0f);
	U(2) = math::constrain(U(2), -2.0f, 2.0f);
	math::Vector<3> tau2 = tau1 + U;

	/* OUTPUT : att_control vector including torque */
	_att_control = (tau1 + U)/(_cmd2torque*_Lambda*_G);
	_att_control(0) *= 0.1f;
	_att_control(1) *= 0.1f;
	_att_control(2) *= 0.1f;

	/* P, Q filter update */
	math::Vector<6> dp = _A*_p + _B*tau2;
	_p = _p + dp * dt;
	_q = _q + dq * dt;
}

void AttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	attitude_control::g_control->task_main();
}

void AttitudeControl::task_main()
{
	/* do subscriptions */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: vehicle attitude */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _ctrl_state_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) 
	{
		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) 
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) 
		{
			warn("attitude_control: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		//// main loop!!!
		if (fds[0].revents & POLLIN) 
		{	
			
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) 
			{
				dt = 0.002f;
			} 
			else if (dt > 0.02f) 
			{
				dt = 0.02f;
			}

			/* copy attitude and control state topics */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();
			battery_status_poll();

//			if( _armed.armed )
//			{
				// hss : control loop is only executed when vehicle is armed

				if (_v_control_mode.flag_control_rates_enabled) 
				{
					control(dt);

					/* publish actuator controls */
					_actuators.control[0]=(PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
					_actuators.control[1]=(PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
					_actuators.control[2]=(PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
					_actuators.control[3]=(PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
					_actuators.control[7]=_v_att_sp.landing_gear;
					_actuators.timestamp = hrt_absolute_time();
					_actuators.timestamp_sample = _ctrl_state.timestamp;
				
//					mavlink_log_info(&_mavlink_log_pub, 
//							"[att] r %2.4f, p %2.4f, y %2.4f, F %2.4f",
//							(double)_actuators.control[0],
//							(double)_actuators.control[1],
//							(double)_actuators.control[2],
//							(double)_actuators.control[3]);

					_controller_status.roll_rate_integ = _rates_int(0);
					_controller_status.pitch_rate_integ = _rates_int(1);
					_controller_status.yaw_rate_integ = _rates_int(2);
					_controller_status.timestamp = hrt_absolute_time();

					if (!_actuators_0_circuit_breaker_enabled) 
					{
						if (_actuators_0_pub != nullptr) 
						{
							orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
							perf_end(_controller_latency_perf);
						} 
						else if (_actuators_id) 
						{
							_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
						}
					}
	
					/* publish controller status */
					if (_controller_status_pub != nullptr) 
					{
						orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);
					} 
					else 
					{
						_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
					}
				}
//			}
		}
		perf_end(_loop_perf);
	} 

	_control_task = -1;
	return;
}

int
AttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("attitude_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&AttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int attitude_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: attitude_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (attitude_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		attitude_control::g_control = new AttitudeControl;

		if (attitude_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != attitude_control::g_control->start()) {
			delete attitude_control::g_control;
			attitude_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (attitude_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete attitude_control::g_control;
		attitude_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (attitude_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
