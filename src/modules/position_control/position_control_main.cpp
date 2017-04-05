#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/home_position.h>

#include <float.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <platforms/px4_defines.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ONE_G	9.8066f

extern "C" __EXPORT int position_control_main(int argc, char *argv[]);

class PositionControl : public control::SuperBlock
{
public:
	
	PositionControl();

	~PositionControl();
	
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task should exit */
	bool	_gear_state_initialized;	///< true if the gear state has been initialized
	int		_control_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */
	int		_home_pos_sub; 			/**< home position */
	
	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */

	orb_id_t _attitude_setpoint_id;

	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct control_state_s				_ctrl_state;		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */
	struct home_position_s				_home_pos; 				/**< home position */

	control::BlockParamFloat _manual_thr_min;
	control::BlockParamFloat _manual_thr_max;
	control::BlockParamFloat _manual_land_alt;

	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	struct 
	{
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t alt_ctl_dz;
		param_t alt_ctl_dy;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tko_speed;
		param_t tilt_max_land;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_xy_dz;
		param_t hold_z_dz;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t acc_hor_max;
		param_t acc_up_max;
		param_t acc_down_max;
		param_t alt_mode;
		param_t opt_recover;
		param_t xy_vel_man_expo;
		// hss added
		param_t pwm_max;
		param_t pwm_min;
	}		_params_handles;		/**< handles for interesting parameters */

	struct 
	{
		float thr_min;
		float thr_max;
		float thr_hover;
		float alt_ctl_dz;
		float alt_ctl_dy;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_xy_dz;
		float hold_z_dz;
		float hold_max_xy;
		float hold_max_z;
		float acc_hor_max;
		float acc_up_max;
		float acc_down_max;
		float vel_max_up;
		float vel_max_down;
		float xy_vel_man_expo;
		uint32_t alt_mode;

		int opt_recover;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<2> vel_cruise;

		// hss added
		int pwm_max;
		int pwm_min;
	}		_params;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _do_reset_alt_pos_flag;
	bool _mode_auto;
	bool _pos_hold_engaged;
	bool _alt_hold_engaged;
	bool _run_pos_control; // XXX flag for generate setpoint velocity x,y from setpoint position
	bool _run_alt_control; // XXX flag for generate setppint velocity z from setpoint position

	bool _reset_int_z = true;
	bool _reset_int_xy = true;
	bool _reset_int_z_manual = false;
	bool _reset_yaw_sp = true;

	bool _hold_offboard_xy = false;
	bool _hold_offboard_z = false;

	math::Vector<3> _thrust_int;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity */

	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	float _roll;
	float _pitch;
	float _yaw;				/**< yaw angle (euler) */
	float _yaw_takeoff;	/**< home yaw angle present when vehicle was taking off (euler) */
	bool _in_landing;	/**< the vehicle is in the landing descent */
	bool _lnd_reached_ground; /**< controller assumes the vehicle has reached the ground after landing */
	bool _takeoff_jumped;
	float _vel_z_lp;
	float _acc_z_lp;
	float _takeoff_thrust_sp;

	// counters for reset events on position and velocity states
	// they are used to identify a reset event
	uint8_t _z_reset_counter;
	uint8_t _xy_reset_counter;
	uint8_t _vz_reset_counter;
	uint8_t _vxy_reset_counter;
	uint8_t _heading_reset_counter;

	matrix::Dcmf _R_setpoint;

	//// DOB params definition
	float _a0; 					// filter params
	float _a1; 					// filter params
	float _eps; 				// filter time constant
	math::Matrix<2,2> _A; 		// filter system matrix
	math::Matrix<2,1> _B; 		// filter input matrix
	math::Matrix<2,1> _px; 		// p-filter state
	math::Matrix<2,1> _qx;		// q-filter state
	math::Matrix<2,1> _py;		// p-filter state
	math::Matrix<2,1> _qy;		// q-filter state
	math::Matrix<2,1> _pz;		// p-filter state
	math::Matrix<2,1> _qz;		// q-filter state
	float _mass;				// quad mass [kg]
	bool _filter_initialized; 	// filter initialization flag
	float _max_force_N;			// maximum force
	float _p1;					// polynomial parameter
	float _p2;					// polynomial parameter
	float _p3;					// polynomial parameter

	int		parameters_update(bool force);
	void		poll_subscriptions();
	static float    throttle_curve(float ctl, float ctr); // used in gen_att_set
	void		update_ref();
	
	void		reset_pos_sp();
	void		reset_alt_sp();

	void		control_offboard(float dt);
	void		setpoint_generation(float dt);
	void 		control_position(float dt); 

	void		do_control(float dt);

	void		generate_attitude_setpoint(float dt);

	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main();

	//// DOB functions
	void 		reset_PQ_filter();
	void 		control_position2(float dt); 

};

namespace position_control
{

PositionControl	*g_control;
}

PositionControl::PositionControl() :
	SuperBlock(nullptr, "MPC"),
	_task_should_exit(false),
	_gear_state_initialized(false),
	_control_task(-1),
	_mavlink_log_pub(nullptr),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_global_vel_sp_sub(-1),
	_home_pos_sub(-1),

	/* publications */
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_attitude_setpoint_id(nullptr),

	_vehicle_status{},
	_vehicle_land_detected{},
	_ctrl_state{},
	_att_sp{},
	_manual{},
	_control_mode{},
	_arming{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_global_vel_sp{},
	_home_pos{},
	_manual_thr_min(this, "MANTHR_MIN"),
	_manual_thr_max(this, "MANTHR_MAX"),
	_manual_land_alt(this, "MIS_LTRMIN_ALT", false),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_ref_alt(0.0f),
	_ref_timestamp(0),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
	_do_reset_alt_pos_flag(true),
	_mode_auto(false),
	_pos_hold_engaged(false),
	_alt_hold_engaged(false),
	_run_pos_control(true),
	_run_alt_control(true),
	_roll(0.0f),
	_pitch(0.0f),
	_yaw(0.0f),
	_yaw_takeoff(0.0f),
	_in_landing(false),
	_lnd_reached_ground(false),
	_takeoff_jumped(false),
	_vel_z_lp(0),
	_acc_z_lp(0),
	_takeoff_thrust_sp(0.0f),
	_z_reset_counter(0),
	_xy_reset_counter(0),
	_vz_reset_counter(0),
	_vxy_reset_counter(0),
	_heading_reset_counter(0)
{
	// Make the quaternion valid for control state
	_ctrl_state.q[0] = 1.0f;

	memset(&_ref_pos, 0, sizeof(_ref_pos));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_cruise.zero();
	_params.vel_ff.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_vel_sp_prev.zero();
	_vel_err_d.zero();

	_R.identity();

	_R_setpoint.identity();

	_thrust_int.zero();

	//// DOB params construction
	_a0 = 0.0;
	_a1 = 0.0;
	_eps = 0.1;
	_A.zero();
	_B.zero();
	_px.zero();
	_qx.zero();
	_py.zero();
	_qy.zero();
	_pz.zero();
	_qz.zero();
	_mass = 1.5;
	_filter_initialized = false;
	_max_force_N = 0.0;
	_p1 = 1.359e-5;
	_p2 = 0.01536;
	_p3 = 0.4755;

	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.thr_hover	= param_find("MPC_THR_HOVER");
	_params_handles.alt_ctl_dz	= param_find("MPC_ALTCTL_DZ");
	_params_handles.alt_ctl_dy	= param_find("MPC_ALTCTL_DY");
	_params_handles.z_p		= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max_up	= param_find("MPC_Z_VEL_MAX_UP");
	_params_handles.z_vel_max_down	= param_find("MPC_Z_VEL_MAX_DN");
	_params_handles.z_ff		= param_find("MPC_Z_FF");
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_vel_cruise	= param_find("MPC_XY_CRUISE");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.tko_speed	= param_find("MPC_TKO_SPEED");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND");
	_params_handles.man_roll_max = param_find("MPC_MAN_R_MAX");
	_params_handles.man_pitch_max = param_find("MPC_MAN_P_MAX");
	_params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX");
	_params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
	_params_handles.mc_att_yaw_p = param_find("MC_YAW_P");
	_params_handles.hold_xy_dz = param_find("MPC_HOLD_XY_DZ");
	_params_handles.hold_z_dz = param_find("MPC_HOLD_Z_DZ");
	_params_handles.hold_max_xy = param_find("MPC_HOLD_MAX_XY");
	_params_handles.hold_max_z = param_find("MPC_HOLD_MAX_Z");
	_params_handles.acc_hor_max = param_find("MPC_ACC_HOR_MAX");
	_params_handles.acc_up_max = param_find("MPC_ACC_UP_MAX");
	_params_handles.acc_down_max = param_find("MPC_ACC_DOWN_MAX");
	_params_handles.alt_mode = param_find("MPC_ALT_MODE");
	_params_handles.opt_recover = param_find("VT_OPT_RECOV_EN");
	_params_handles.xy_vel_man_expo = param_find("MPC_XY_MAN_EXPO");

	// hss added
	_params_handles.pwm_max = param_find("PWM_MAX");
	_params_handles.pwm_min = param_find("PWM_MIN");

	/* fetch initial parameter values */
	parameters_update(true);
}

PositionControl::~PositionControl()
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

	position_control::g_control = nullptr;
}

int PositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		/* update C++ param system */
		updateParams();

		/* update legacy C interface params */
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.thr_hover, &_params.thr_hover);
		_params.thr_hover = math::constrain(_params.thr_hover, _params.thr_min, _params.thr_max);
		param_get(_params_handles.alt_ctl_dz, &_params.alt_ctl_dz);
		param_get(_params_handles.alt_ctl_dy, &_params.alt_ctl_dy);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.tko_speed, &_params.tko_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land);

		float v;
		uint32_t v_i;
		param_get(_params_handles.xy_p, &v);
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.xy_vel_p, &v);
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v);	//// a0
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v); 	//// a1
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v);
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_max_up = v;
		_params.vel_max(2) = v;
		param_get(_params_handles.z_vel_max_down, &v);
		_params.vel_max_down = v;
		param_get(_params_handles.xy_vel_cruise, &v);
		_params.vel_cruise(0) = v;
		_params.vel_cruise(1) = v;
		param_get(_params_handles.xy_ff, &v);		//// eps
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		_params.vel_ff(2) = v;
		param_get(_params_handles.hold_xy_dz, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.hold_xy_dz = v;
		param_get(_params_handles.hold_z_dz, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.hold_z_dz = v;
		param_get(_params_handles.hold_max_xy, &v);
		_params.hold_max_xy = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.hold_max_z, &v);
		_params.hold_max_z = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.acc_hor_max, &v);
		_params.acc_hor_max = v;
		param_get(_params_handles.acc_up_max, &v);
		_params.acc_up_max = v;
		param_get(_params_handles.acc_down_max, &v);
		_params.acc_down_max = v;
		param_get(_params_handles.xy_vel_man_expo, &v);
		_params.xy_vel_man_expo = v;

		int ppp;
		param_get(_params_handles.pwm_max, &ppp);
		_params.pwm_max = ppp;
		param_get(_params_handles.pwm_min, &ppp);
		_params.pwm_min = ppp;

		//// DOB params update
		_a0 = _params.vel_i(0); // xy_vel_i
		_a1 = _params.vel_i(2); // z_vel_i
		_eps = _params.vel_ff(0); // xy_ff
		_A(0,1) = 1.0;	_A(1,0) = -_a0/(_eps*_eps); _A(1,1) = -_a1/_eps;
		_B(1,0) = _a0/(_eps*_eps);

		/*
		 * increase the maximum horizontal acceleration such that stopping
		 * within 1 s from full speed is feasible
		 */
		_params.acc_hor_max = math::max(_params.vel_cruise(0), _params.acc_hor_max);
		param_get(_params_handles.alt_mode, &v_i);
		_params.alt_mode = v_i;

		int i;
		param_get(_params_handles.opt_recover, &i);
		_params.opt_recover = i;

		/* manual control scale */
		param_get(_params_handles.man_roll_max, &_params.man_roll_max);
		param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
		param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
		param_get(_params_handles.global_yaw_max, &_params.global_yaw_max);
		_params.man_roll_max = math::radians(_params.man_roll_max);
		_params.man_pitch_max = math::radians(_params.man_pitch_max);
		_params.man_yaw_max = math::radians(_params.man_yaw_max);
		_params.global_yaw_max = math::radians(_params.global_yaw_max);

		param_get(_params_handles.mc_att_yaw_p, &v);
		_params.mc_att_yaw_p = v;

		/* takeoff and land velocities should not exceed maximum */
		_params.tko_speed = fminf(_params.tko_speed, _params.vel_max_up);
		_params.land_speed = fminf(_params.land_speed, _params.vel_max_down);
	}

	return OK;
}

void PositionControl::poll_subscriptions()
{

	bool updated;

	// vehicle_status
	orb_check(_vehicle_status_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_attitude_setpoint_id) 
		{
			if (_vehicle_status.is_vtol) 
			{
				_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);
			} 
			else 
			{
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}

	// land detected
	orb_check(_vehicle_land_detected_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, 
				&_vehicle_land_detected);
	}

	// control state (important)
	orb_check(_ctrl_state_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

		/* get current rotation matrix and euler angles from control state quaternions */
		math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], 
				_ctrl_state.q[2], _ctrl_state.q[3]);
		_R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _R.to_euler();
		_roll = euler_angles(0);
		_pitch = euler_angles(1);
		_yaw = euler_angles(2);

		if (_control_mode.flag_control_manual_enabled) 
		{
			if (_heading_reset_counter != _ctrl_state.quat_reset_counter) 
			{
				_heading_reset_counter = _ctrl_state.quat_reset_counter;
				math::Quaternion delta_q(_ctrl_state.delta_q_reset[0], 
						_ctrl_state.delta_q_reset[1], 
						_ctrl_state.delta_q_reset[2], 
						_ctrl_state.delta_q_reset[3]);

				// we only extract the heading change from the delta quaternion
				math::Vector<3> delta_euler = delta_q.to_euler();
				_att_sp.yaw_body += delta_euler(2);
			}
		}
	}

	// attitude setppint
	orb_check(_att_sp_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	// control mode
	orb_check(_control_mode_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	// manual control setpoint
	orb_check(_manual_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	// arming status
	orb_check(_arming_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}

	// local position
	orb_check(_local_pos_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

		// hss : update _pos and _vel here!
		_pos(0) = _local_pos.x;
		_pos(1) = _local_pos.y;
		_pos(2) = _local_pos.z;

		_vel(0) = _local_pos.vx;
		_vel(1) = _local_pos.vy;
		_vel(2) = _local_pos.vz;

		// check if a reset event has happened
		// if the vehicle is in manual mode we will shift the setpoints of the
		// states which were reset. In auto mode we do not shift the setpoints
		// since we want the vehicle to track the original state.
		if (_control_mode.flag_control_manual_enabled) 
		{
			if (_z_reset_counter != _local_pos.z_reset_counter) 
			{
				_pos_sp(2) += _local_pos.delta_z;
			}

			if (_xy_reset_counter != _local_pos.xy_reset_counter) 
			{
				_pos_sp(0) += _local_pos.delta_xy[0];
				_pos_sp(1) += _local_pos.delta_xy[1];
			}

			if (_vz_reset_counter != _local_pos.vz_reset_counter) 
			{
				_vel_sp(2) += _local_pos.delta_vz;
			}

			if (_vxy_reset_counter != _local_pos.vxy_reset_counter) 
			{
				_vel_sp(0) += _local_pos.delta_vxy[0];
				_vel_sp(1) += _local_pos.delta_vxy[1];
			}
		}

		// update the reset counters in any case
		_z_reset_counter = _local_pos.z_reset_counter;
		_xy_reset_counter = _local_pos.xy_reset_counter;
		_vz_reset_counter = _local_pos.vz_reset_counter;
		_vxy_reset_counter = _local_pos.vxy_reset_counter;
	}

	// position setpoint triplet
	orb_check(_pos_sp_triplet_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		//Make sure that the position setpoint is valid
		if (!PX4_ISFINITE(_pos_sp_triplet.current.lat) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.lon) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.alt)) 
		{
			_pos_sp_triplet.current.valid = false;
		}
	}

	// home position
	orb_check(_home_pos_sub, &updated);
	if (updated) 
	{
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}
}

float PositionControl::throttle_curve(float ctl, float ctr)
{
	/* piecewise linear mapping: 0:ctr -> 0:0.5
	 * and ctr:1 -> 0.5:1 */
	if (ctl < 0.5f) 
	{
		return 2 * ctl * ctr;
	} 
	else 
	{
		return ctr + 2 * (ctl - 0.5f) * (1.0f - ctr);
	}
}

void PositionControl::task_main_trampoline(int argc, char *argv[])
{
	position_control::g_control->task_main();
}

void PositionControl::update_ref()
{
	if (_local_pos.ref_timestamp != _ref_timestamp) 
	{
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) 
		{
			/* calculate current position setpoint in global frame */
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2);
		}

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_ref_timestamp != 0) 
		{
			/* reproject position setpoint to new reference */
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

void PositionControl::reset_pos_sp()
{
	if (_reset_pos_sp) 
	{
		_reset_pos_sp = false;
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
	}
}

void PositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) 
	{
		_reset_alt_sp = false;
		_pos_sp(2) = _pos(2);
	}
}

void PositionControl::control_offboard(float dt)
{
	/* weather-vane mode for vtol: disable yaw control */
	// XXX is this necessary?
	// this is room for yaw control... so remain
	if (_vehicle_status.is_vtol) 
	{
		_att_sp.disable_mc_yaw_control = _pos_sp_triplet.current.disable_mc_yaw_control;
	} 
	else 
	{
		_att_sp.disable_mc_yaw_control = false;
	}

	// for safety issue, this is somewhat good.
	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) 
	{
		/* idle state, don't run controller and set zero thrust */
		_R_setpoint.identity();

		matrix::Quatf qd = _R_setpoint;
		memcpy(&_att_sp.q_d[0], qd.data(), sizeof(_att_sp.q_d));
		_att_sp.q_d_valid = true;

		_att_sp.roll_body = 0.0f;
		_att_sp.pitch_body = 0.0f;
		_att_sp.yaw_body = _yaw;
		_att_sp.thrust = 0.0f;

		_att_sp.timestamp = hrt_absolute_time();
	} 
	else 
	{
		//control_position(dt); //////// true controller
		control_position2(dt); //////// true controller
	}
}

void PositionControl::setpoint_generation(float dt)
{
	// hss : not much to do
	if (_pos_sp_triplet.current.valid) 
	{
		/** XY **/
		if (_control_mode.flag_control_position_enabled && 
				_pos_sp_triplet.current.position_valid)
		{
			/* control position */
			_pos_sp(0) = _pos_sp_triplet.current.x;
			_pos_sp(1) = _pos_sp_triplet.current.y;
			_run_pos_control = true;
			_hold_offboard_xy = false;
		} 
		else if (_control_mode.flag_control_velocity_enabled && 
				_pos_sp_triplet.current.velocity_valid) 
		{
			/* control velocity */
			/* reset position setpoint to current position if needed */
			reset_pos_sp();

			if (fabsf(_pos_sp_triplet.current.vx) <= FLT_EPSILON &&
			    fabsf(_pos_sp_triplet.current.vy) <= FLT_EPSILON &&
			    _local_pos.xy_valid) 
			{
				// hss : for small velocity, position hold is performed
				if (!_hold_offboard_xy)
				{
					_pos_sp(0) = _pos(0);
					_pos_sp(1) = _pos(1);
					_hold_offboard_xy = true;
				}
				_run_pos_control = true;

			} 
			else 
			{
				// hss : velocity is assigned per frame.
				if (_pos_sp_triplet.current.velocity_frame == 
						position_setpoint_s::VELOCITY_FRAME_LOCAL_NED) 
				{
					_vel_sp(0) = _pos_sp_triplet.current.vx;
					_vel_sp(1) = _pos_sp_triplet.current.vy;

				} 
				else if (_pos_sp_triplet.current.velocity_frame == 
						position_setpoint_s::VELOCITY_FRAME_BODY_NED) 
				{
					// Transform velocity command from body frame to NED frame
					_vel_sp(0) = cosf(_yaw) * _pos_sp_triplet.current.vx - 
						sinf(_yaw) * _pos_sp_triplet.current.vy;
					_vel_sp(1) = sinf(_yaw) * _pos_sp_triplet.current.vx + 
						cosf(_yaw) * _pos_sp_triplet.current.vy;

				} 
				else 
				{
					PX4_WARN("Unknown velocity offboard coordinate frame");
				}
				_run_pos_control = false;
				_hold_offboard_xy = false;
			}

		}

		/** Z **/
		if (_control_mode.flag_control_altitude_enabled && 
				_pos_sp_triplet.current.alt_valid) 
		{
			/* control altitude as it is enabled */
			_pos_sp(2) = _pos_sp_triplet.current.z;
			_run_alt_control = true;
			_hold_offboard_z = false;

		} 
		else if (_control_mode.flag_control_climb_rate_enabled && 
				_pos_sp_triplet.current.velocity_valid) 
		{
			/* reset alt setpoint to current altitude if needed */
			reset_alt_sp();

			if (fabsf(_pos_sp_triplet.current.vz) <= FLT_EPSILON &&
			    _local_pos.z_valid) 
			{
				if (!_hold_offboard_z) 
				{
					_pos_sp(2) = _pos(2);
					_hold_offboard_z = true;
				}
				_run_alt_control = true;

			} 
			else 
			{
				/* set position setpoint move rate */
				_vel_sp(2) = _pos_sp_triplet.current.vz;
				_run_alt_control = false;
				_hold_offboard_z = false;
			}
		}

		/** yaw **/
		if (_pos_sp_triplet.current.yaw_valid) 
		{
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		} 
		else if (_pos_sp_triplet.current.yawspeed_valid) 
		{
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;
		}

	} 
	else 
	{
		_hold_offboard_xy = false;
		_hold_offboard_z = false;
		reset_pos_sp();
		reset_alt_sp();
	}
}

void PositionControl::do_control(float dt)
{


}

void PositionControl::control_position(float dt)
{
	// this is main control loop
	/* run position & altitude controllers, if enabled 
	   (otherwise use already computed velocity setpoints) */
	if (_run_pos_control) 
	{
		_vel_sp(0) = (_pos_sp(0) - _pos(0)) * _params.pos_p(0);
		_vel_sp(1) = (_pos_sp(1) - _pos(1)) * _params.pos_p(1);
	}

	if (_run_alt_control) 
	{
		_vel_sp(2) = (_pos_sp(2) - _pos(2)) * _params.pos_p(2);
	}

	// hss : reset counter
	if (!_control_mode.flag_control_position_enabled) 
	{
		_reset_pos_sp = true;
	}
	if (!_control_mode.flag_control_altitude_enabled) 
	{
		_reset_alt_sp = true;
	}
	if (!_control_mode.flag_control_velocity_enabled) 
	{
		_vel_sp(0) = 0.0f;
		_vel_sp(1) = 0.0f;
	}
	if (!_control_mode.flag_control_climb_rate_enabled) 
	{
		_vel_sp(2) = 0.0f;
	}


	//// control loop starts
	if (_control_mode.flag_control_climb_rate_enabled || 
			_control_mode.flag_control_velocity_enabled ||
	    	_control_mode.flag_control_acceleration_enabled) 
	{
		// hss : this is performed in offboard mode
		
		/* velocity error */
		math::Vector<3> vel_err = _vel_sp - _vel;

		/* thrust vector in NED frame */
		math::Vector<3> thrust_sp;

//		thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d) 
//			+ _thrust_int - math::Vector<3>(0.0f, 0.0f, _params.thr_hover);
		// hss : thrust vector is generated by pure velocity errors
		thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d); 

		// hss : this process is IMPORTANT!!
		/* Calculate desired total thrust amount in body z direction. */
		/* To compensate for excess thrust during attitude tracking errors we
		 * project the desired thrust force vector F onto the real vehicle's thrust axis in NED:
		 * body thrust axis [0,0,-1]' rotated by R is: R*[0,0,-1]' = -R_z */
		matrix::Vector3f R_z(_R(0, 2), _R(1, 2), _R(2, 2));
		matrix::Vector3f F(thrust_sp.data);
		// hss : thrust_body_z will be used for _att_sp.thrust
		float thrust_body_z = F.dot(-R_z); /* recalculate because it might have changed */
		// hss : maybe negative if ascending

		// hss : I changed this line
//		_att_sp.thrust = math::max(thrust_body_z, thr_min);
		_att_sp.thrust = thrust_body_z;

		// TODO : this geometric generation of attitude setpoint is good
		// but for our purpose, it is not desirable now
		// I have to modify this process with conventional way
		/* calculate attitude setpoint from thrust vector */
		math::Vector<3> body_x;
		math::Vector<3> body_y;
		math::Vector<3> body_z;

		if (thrust_sp.length() > SIGMA) 
		{
			body_z = -thrust_sp.normalized();
		} 
		else 
		{
			/* no thrust, set Z axis to safe value */
			body_z.zero();
			body_z(2) = 1.0f;
		}

		/* vector of desired yaw direction in XY plane, rotated by PI/2 */
		math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

		if (fabsf(body_z(2)) > SIGMA) 
		{
			/* desired body_x axis, orthogonal to body_z */
			body_x = y_C % body_z;

			/* keep nose to front while inverted upside down */
			if (body_z(2) < 0.0f) 
			{
				body_x = -body_x;
			}

			body_x.normalize();

		} 
		else 
		{
			/* desired thrust is in XY plane, set X downside to construct correct matrix,
			 * but yaw component will not be used actually */
			body_x.zero();
			body_x(2) = 1.0f;
		}

		/* desired body_y axis */
		body_y = body_z % body_x;

		/* fill rotation matrix */
		for (int i = 0; i < 3; i++) 
		{
			_R_setpoint(i, 0) = body_x(i);
			_R_setpoint(i, 1) = body_y(i);
			_R_setpoint(i, 2) = body_z(i);
		}

		matrix::Quatf q_sp = _R_setpoint;
		matrix::Eulerf euler = _R_setpoint;
	
		memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
		_att_sp.q_d_valid = true;		
		_att_sp.roll_body = euler(0);
		_att_sp.pitch_body = euler(1);

		_att_sp.timestamp = hrt_absolute_time();
	} 
	else 
	{
		_reset_int_z = true;
	}
}


void PositionControl::generate_attitude_setpoint(float dt)
{
	// hss : this must be PRESERVED!!
	/* reset yaw setpoint to current position if needed */
	if (_reset_yaw_sp) 
	{
		_reset_yaw_sp = false;
		_att_sp.yaw_body = _yaw;
	}
	else if (!_vehicle_land_detected.landed &&
		 	!(!_control_mode.flag_control_altitude_enabled && _manual.z < 0.1f)) 
	{
		/* we want to know the real constraint, and global overrides manual */
		const float yaw_rate_max = (_params.man_yaw_max < _params.global_yaw_max) ? 
			_params.man_yaw_max : _params.global_yaw_max;
		const float yaw_offset_max = yaw_rate_max / _params.mc_att_yaw_p;

		_att_sp.yaw_sp_move_rate = _manual.r * yaw_rate_max;
		float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * dt);
		float yaw_offs = _wrap_pi(yaw_target - _yaw);

		// If the yaw offset became too big for the system to track stop
		// shifting it, only allow if it would make the offset smaller again.
		if (fabsf(yaw_offs) < yaw_offset_max ||
		    (_att_sp.yaw_sp_move_rate > 0 && yaw_offs < 0) ||
		    (_att_sp.yaw_sp_move_rate < 0 && yaw_offs > 0)) 
		{
			_att_sp.yaw_body = yaw_target;
		}
	}

	/* control throttle directly if no climb rate controller is active */
	if (!_control_mode.flag_control_climb_rate_enabled) 
	{
		float thr_val = throttle_curve(_manual.z, _params.thr_hover);
		_att_sp.thrust = math::min(thr_val, _manual_thr_max.get());

		/* enforce minimum throttle if not landed */
		if (!_vehicle_land_detected.landed) 
		{
			_att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
		}
	}

	/* control roll and pitch directly if no aiding velocity controller is active */
	if (!_control_mode.flag_control_velocity_enabled) 
	{
		_att_sp.roll_body = _manual.y * _params.man_roll_max;
		_att_sp.pitch_body = -_manual.x * _params.man_pitch_max;

		/* only if optimal recovery is not used, modify roll/pitch */
		if (_params.opt_recover <= 0) 
		{
			// construct attitude setpoint rotation matrix. modify the setpoints for roll
			// and pitch such that they reflect the user's intention even if a yaw error
			// (yaw_sp - yaw) is present. In the presence of a yaw error constructing 
			// a rotation matrix from the pure euler angle setpoints will lead to 
			// unexpected attitude behaviour from the user's view as the euler angle sequence 
			// uses the yaw setpoint and not the current heading of the vehicle.

			// calculate our current yaw error
			float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);

			// compute the vector obtained by rotating a z unit vector by the rotation
			// given by the roll and pitch commands of the user
			math::Vector<3> zB = {0, 0, 1};
			math::Matrix<3, 3> R_sp_roll_pitch;
			R_sp_roll_pitch.from_euler(_att_sp.roll_body, _att_sp.pitch_body, 0);
			math::Vector<3> z_roll_pitch_sp = R_sp_roll_pitch * zB;

			// transform the vector into a new frame which is rotated around the z axis
			// by the current yaw error. this vector defines the desired tilt when we look
			// into the direction of the desired heading
			math::Matrix<3, 3> R_yaw_correction;
			R_yaw_correction.from_euler(0.0f, 0.0f, -yaw_error);
			z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

			// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
			// R_tilt is computed from_euler; only true if cos(roll) not equal zero
			// -> valid if roll is not +-pi/2;
			_att_sp.roll_body = -asinf(z_roll_pitch_sp(1));
			_att_sp.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
		}

		/* copy quaternion setpoint to attitude setpoint topic */
		matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, 
				_att_sp.pitch_body, _att_sp.yaw_body);
		memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
		_att_sp.q_d_valid = true;
	}

	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && 
		_gear_state_initialized &&
	    !_vehicle_land_detected.landed) 
	{
		_att_sp.landing_gear = 1.0f;
	} 
	else if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) 
	{
		_att_sp.landing_gear = -1.0f;
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	_att_sp.timestamp = hrt_absolute_time();
}

//// DOB functions
void PositionControl::reset_PQ_filter()
{
	_px.zero();
	_qx.zero();
	_py.zero();
	_qy.zero();
	_pz.zero();
	_qz.zero();

	// X and Y
	_qx(0,0) = _pos(0); // x
	_qy(0,0) = _pos(1); // y

	// Z
	_qz(0,0) = _pos(2);

	float P1 = _params.pos_p(2)*( _pos_sp(2) - _pos(2) ); // kp*(z_d - z)
	float P2 = _params.vel_p(2)*( _vel_sp(2) - _vel(2) ); // kd*(zdot_d - zdot)
	float tau0_z = _mass*( -9.81f + P1 + P2);
	_pz(0,0) = (1.0f / sqrtf(_mass))*tau0_z;

	mavlink_log_info(&_mavlink_log_pub, "PQ filter initialized");
}

void PositionControl::control_position2(float dt)
{
	// this is main control loop

	//// control loop starts
	if (_control_mode.flag_control_climb_rate_enabled || 
			_control_mode.flag_control_velocity_enabled ||
	    	_control_mode.flag_control_acceleration_enabled) 
	{
		// hss : error vectors
		// for now, _vel_sp is not used
		// so we have to modify "setpoint_generation" in order to use
		// _pos_sp and _vel_sp simultaneously.
		math::Vector<3> pos_err = _pos_sp - _pos;
		math::Vector<3> vel_err = _vel_sp - _vel;

		// 1. X and Y
		// outer control for X, Y (PD)
		math::Vector<2> tau0; 
		tau0(0) = _params.pos_p(0)*pos_err(0) + _params.vel_p(0)*vel_err(0);
		tau0(1) = _params.pos_p(1)*pos_err(1) + _params.vel_p(1)*vel_err(1);

		math::Matrix<2,1> dqx = _A*_qx + _B*_pos(0);
		math::Matrix<2,1> dqy = _A*_qy + _B*_pos(1);

		math::Vector<2> W; 
		W(0) = math::constrain(_px(0,0) - dqx(1,0), -1.5f, 1.5f);
		W(1) = math::constrain(_py(0,0) - dqy(1,0), -1.5f, 1.5f);

		math::Vector<2> tau1;
		tau1 = tau0 + W;

		math::Vector<2> tau2; // yaw compensation required
		tau2(0) = -0.1019f*( cosf(_yaw)*tau1(0) + sinf(_yaw)*tau1(1) );
		tau2(1) = -0.1019f*( sinf(_yaw)*tau1(0) - cosf(_yaw)*tau1(1) );
		// XXX I have to test another yaw compensation
		// I'm confusing with this...

		
		math::Vector<2> chad;
		chad(0) = cosf(_roll)*sinf(_pitch);
		chad(1) = sinf(_roll);

		math::Vector<2> U; // yaw compensation required
		U(0) = -9.81f*( cosf(_yaw)*chad(0) + sinf(_yaw)*chad(1) );
		U(1) = -9.81f*( sinf(_yaw)*chad(0) - cosf(_yaw)*chad(1) );
		// XXX I have to test another yaw compensation
		// I'm confusing with this...

		math::Matrix<2,1> dpx = _A*_px + _B*U(0);
		math::Matrix<2,1> dpy = _A*_py + _B*U(1);
		
		// 2. Z
		// outer control for Z (PD)
		float tau0_z = _mass*( _params.pos_p(2)*pos_err(2) + _params.vel_p(2)*vel_err(2) - 9.81f);
		
		math::Matrix<2,1> dqz = _A*_qz + _B*_pos(2);
		
		float W_z = math::constrain(_pz(0,0) + sqrtf(_mass)*( 9.81f - dqz(1,0) ),
									-30.0f, 30.0f);
		
		float tau1_z = (1.0f/sqrtf(_mass))*tau0_z + W_z;
		//float tau1_z = (1.0f/sqrtf(_mass))*tau0_z;
		float tau2_z = sqrtf(_mass)*tau1_z;

		math::Matrix<2,1> dpz = _A*_pz + _B*tau1_z;

		// 3. filter update
		_qx = _qx + dqx*dt;
		_qy = _qy + dqy*dt;
		_qz = _qz + dqz*dt;
		_px = _px + dpx*dt;
		_py = _py + dpy*dt;
		_pz = _pz + dpz*dt;

		// 4. attitude setpoint publication
		float roll_sp = tau2(1);
		float pitch_sp = tau2(0)*(1.0f/cosf(roll_sp));
		matrix::Eulerf euler_sp(roll_sp, pitch_sp, _att_sp.yaw_body);
		matrix::Quatf q_sp = euler_sp;
		_R_setpoint = euler_sp;
		memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
		_att_sp.roll_body = euler_sp(0);
		_att_sp.pitch_body = euler_sp(1);
	
		// 5. special care for thrust
		// since tau2_z is real thrust force, we have to convert it into 0~1 scaled value
		// my choice is 0 -> 0N / 1 -> full thrust per motor X 4
		float thrust_sp = (-1.0f*tau2_z)/(_max_force_N);
		_att_sp.thrust = thrust_sp;
		
		_att_sp.timestamp = hrt_absolute_time();
	} 
	else 
	{
		_reset_int_z = true;
	}
}

void PositionControl::task_main()
{
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));

	parameters_update(true);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	/* We really need to know from the beginning if we're landed or in-air. */
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);

	bool was_armed = false;

	hrt_abstime t_prev = 0;

	// Let's be safe and have the landing gear down by default
	_att_sp.landing_gear = -1.0f;

	/* wakeup source */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	// hss, computes maximum force
	_max_force_N = (_p1*(_params.pwm_max-1200)*(_params.pwm_max-1200) + 
				    _p2*(_params.pwm_max-1200) + _p3)*4.0f;

	while (!_task_should_exit) 
	{
		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) 
		{
			// Go through the loop anyway to copy manual input at 50 Hz.
		}
		/* this is undesirable but not much we can do */
		if (pret < 0) 
		{
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		// hss : full state is updated here
		poll_subscriptions();

		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
		t_prev = t;

		// set dt for control blocks
		setDt(dt);

		if (_control_mode.flag_armed && !was_armed) 
		{
			/* reset setpoints and integrals on arming */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_do_reset_alt_pos_flag = true;
			_reset_int_z = true;
			_reset_int_xy = true;
			_reset_yaw_sp = true;
			_yaw_takeoff = _yaw;
		}

		/* reset setpoints and integrators VTOL in FW mode */
		// XXX : is this reauired?
		if (_vehicle_status.is_vtol && !_vehicle_status.is_rotary_wing) 
		{
			_reset_alt_sp = true;
			_reset_int_xy = true;
			_reset_int_z = true;
			_reset_pos_sp = true;
			_reset_yaw_sp = true;
		}

		//Update previous arming state
		was_armed = _control_mode.flag_armed;

		// hss : reference is updated here ? 
		// temporally don't do this
		update_ref();

		// reset the horizontal and vertical position hold flags for non-manual modes
		// or if position / altitude is not controlled
		// hss : this is always true
//		if (!_control_mode.flag_control_position_enabled || 
//			!_control_mode.flag_control_manual_enabled) 
//		{
//			_pos_hold_engaged = false;
//		}

//		if (!_control_mode.flag_control_altitude_enabled || 
//			!_control_mode.flag_control_manual_enabled) 
//		{
//			_alt_hold_engaged = false;
//		}

		/** MAIN CONTROL LOOP **/
		bool offboard_flag = _control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled ||
		    _control_mode.flag_control_acceleration_enabled;

		if (offboard_flag) 
		{
			// hss : OFFBOARD
			// hss : reference is updated here ? 
			setpoint_generation(dt);

			if(!_filter_initialized)
			{
				reset_PQ_filter();
				_filter_initialized = true;
			}

			_run_pos_control = true;
			_run_alt_control = true;

			control_offboard(dt);

			_reset_yaw_sp = true;
			_att_sp.yaw_sp_move_rate = 0.0f;
		} 
		else 
		{
			// hss : MANUAL
			_filter_initialized = false;

			/* position controller disabled, reset setpoints */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_do_reset_alt_pos_flag = true;
			_mode_auto = false;
			_reset_int_z = true;
			_reset_int_xy = true;

			// hss : instead, manual control mode generates _att_sp here
			generate_attitude_setpoint(dt);
		}

		if (!(_control_mode.flag_control_offboard_enabled &&
		    !(_control_mode.flag_control_position_enabled ||
			_control_mode.flag_control_velocity_enabled ||
			_control_mode.flag_control_acceleration_enabled))) 
		{
			// hss : this is always published, IMPORTANT!!
			if (_att_sp_pub != nullptr) 
			{
				orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);
			} 
			else if (_attitude_setpoint_id) 
			{
				_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
			}
		}
		
//		mavlink_log_info(&_mavlink_log_pub, 
//				"[pos] r %2.4f, p %2.4f, y %2.4f, F %2.4f",
//				(double)_att_sp.roll_body,
//				(double)_att_sp.pitch_body,
//				(double)_att_sp.yaw_body,
//				(double)_att_sp.thrust);

//		mavlink_log_info(&_mavlink_log_pub,
//				"[pos] max force : %2.4f",
//				(double)_max_force_N);
	}

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
}

int PositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("position_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1900,
					   (px4_main_t)&PositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) 
	{
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int position_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: position_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (position_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		position_control::g_control = new PositionControl;

		if (position_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != position_control::g_control->start()) {
			delete position_control::g_control;
			position_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (position_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete position_control::g_control;
		position_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (position_control::g_control) {
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
