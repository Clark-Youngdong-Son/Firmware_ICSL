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

//	bool	cross_sphere_line(const math::Vector<3> &sphere_c, const float sphere_r,
//			const math::Vector<3> &line_a, const math::Vector<3> &line_b, math::Vector<3> &res);

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

	int		parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

//	static float	scale_control(float ctl, float end, float dz, float dy);
	static float    throttle_curve(float ctl, float ctr); // used in gen_att_set

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();
	
	/**
	 * Reset position setpoint to current position.
	 *
	 * This reset will only occur if the _reset_pos_sp flag has been set.
	 * The general logic is to first "activate" the flag in the flight
	 * regime where a switch to a position control mode should hold the
	 * very last position. Once switching to a position control mode
	 * the last position is stored once.
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude.
	 *
	 * This reset will only occur if the _reset_alt_sp flag has been set.
	 * The general logic follows the reset_pos_sp() architecture.
	 */
	void		reset_alt_sp();
	
	//void		control_manual(float dt);  
	//void		control_auto(float dt); 

	void		control_non_manual(float dt);
	void		control_offboard(float dt);
	void 		control_position(float dt); 

//	void 		vel_sp_slewrate(float dt);
//	void		update_velocity_derivative(); // hss : I think additional filter is unnecessary

	void		do_control(float dt);

	void		generate_attitude_setpoint(float dt);

	/**
	 * limit altitude based on several conditions
	 */
//	void 		limit_altitude();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
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
		param_get(_params_handles.xy_vel_i, &v);
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
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
		param_get(_params_handles.xy_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
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
				_vel_sp_prev(2) +=  _local_pos.delta_vz;
			}

			if (_vxy_reset_counter != _local_pos.vxy_reset_counter) 
			{
				_vel_sp(0) += _local_pos.delta_vxy[0];
				_vel_sp(1) += _local_pos.delta_vxy[1];
				_vel_sp_prev(0) += _local_pos.delta_vxy[0];
				_vel_sp_prev(1) += _local_pos.delta_vxy[1];
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

void PositionControl::control_non_manual(float dt)
{
	/* select control source */
//	if (_control_mode.flag_control_offboard_enabled) 
//	{
		/* offboard control */
		control_offboard(dt);
		_mode_auto = false;
//	} 
//	else 
//	{
		//_hold_offboard_xy = false;
		//_hold_offboard_z = false;
		/* AUTO */
		//control_auto(dt); // XXX remove this!!
//	}

	// hss : AS of control_offboard 

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

	// guard against any bad velocity values
//	bool velocity_valid = PX4_ISFINITE(_pos_sp_triplet.current.vx) &&
//			      PX4_ISFINITE(_pos_sp_triplet.current.vy) &&
//			      _pos_sp_triplet.current.velocity_valid;

	// do not go slower than the follow target velocity 
	// when position tracking is active (set to valid)
	// XXX is this necessary? 
	// may be not..
//	if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
//	    velocity_valid &&
//	    _pos_sp_triplet.current.position_valid) 
//	{
//		math::Vector<3> ft_vel(_pos_sp_triplet.current.vx, _pos_sp_triplet.current.vy, 0);
//	
//		float cos_ratio = (ft_vel * _vel_sp) / (ft_vel.length() * _vel_sp.length());
//	
//		// only override velocity set points 
//		// when uav is traveling in same direction as target and 
//		//vector component is greater than calculated position set point velocity component
//		if (cos_ratio > 0) 
//		{
//			ft_vel *= (cos_ratio);
//			// min speed a little faster than target vel
//			ft_vel += ft_vel.normalized() * 1.5f;
//		} 
//		else 
//		{
//			ft_vel.zero();
//		}
//		_vel_sp(0) = fabsf(ft_vel(0)) > fabsf(_vel_sp(0)) ? ft_vel(0) : _vel_sp(0);
//		_vel_sp(1) = fabsf(ft_vel(1)) > fabsf(_vel_sp(1)) ? ft_vel(1) : _vel_sp(1);
//	
//		// track target using velocity only
//	
//	} 
//	else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
//		   velocity_valid) 
//	{
//		_vel_sp(0) = _pos_sp_triplet.current.vx;
//		_vel_sp(1) = _pos_sp_triplet.current.vy;
//	}

	/* use constant descend rate when landing, ignore altitude setpoint */
	// XXX is this necessary?
	// DEFINITELY not
//	if (_pos_sp_triplet.current.valid
//	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) 
//	{
//		_vel_sp(2) = _params.land_speed;
//		_run_alt_control = false;
//	}

	/* special thrust setpoint generation for takeoff from ground */
	// XXX is this necessary?
	// DEFINITELY not
//	if (_pos_sp_triplet.current.valid
//	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
//	    && _control_mode.flag_armed) 
//	{
//		// check if we are not already in air.
//		// if yes then we don't need a jumped takeoff anymore
//		if (!_takeoff_jumped && !_vehicle_land_detected.landed && 
//				fabsf(_takeoff_thrust_sp) < FLT_EPSILON) 
//		{
//			_takeoff_jumped = true;
//		}
//	
//		if (!_takeoff_jumped) 
//		{
//			// ramp thrust setpoint up
//			if (_vel(2) > -(_params.tko_speed / 2.0f)) 
//			{
//				_takeoff_thrust_sp += 0.5f * dt;
//				_vel_sp.zero();
//				_vel_prev.zero();
//			} 
//			else 
//			{
//				// copter has reached our takeoff speed. split the thrust setpoint up
//				// into an integral part and into a P part
//				_thrust_int(2) = _takeoff_thrust_sp - _params.vel_p(2) * fabsf(_vel(2));
//				_thrust_int(2) = -math::constrain(_thrust_int(2), _params.thr_min,_params.thr_max);
//				_vel_sp_prev(2) = -_params.tko_speed;
//				_takeoff_jumped = true;
//				_reset_int_z = false;
//			}
//		}
//	
//		if (_takeoff_jumped) 
//		{
//			_vel_sp(2) = -_params.tko_speed;
//		}
//	
//	} 
//	else 
//	{
//		_takeoff_jumped = false;
//		_takeoff_thrust_sp = 0.0f;
//	}

	// XXX is this necessary?
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
		control_position(dt); //////// true controller
	}
}

void PositionControl::control_offboard(float dt)
{
	// hss : not much to do
	//mavlink_log_info(&_mavlink_log_pub, "control_offboard");
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

//void PositionControl::vel_sp_slewrate(float dt)
//{
//	// XXX is this necessary?, maybe not
//	math::Vector<3> acc = (_vel_sp - _vel_sp_prev) / dt;
//	float acc_xy_mag = sqrtf(acc(0) * acc(0) + acc(1) * acc(1));
//
//	/* limit total horizontal acceleration */
//	if (acc_xy_mag > _params.acc_hor_max) 
//	{
//		_vel_sp(0) = _params.acc_hor_max * acc(0) / acc_xy_mag * dt + _vel_sp_prev(0);
//		_vel_sp(1) = _params.acc_hor_max * acc(1) / acc_xy_mag * dt + _vel_sp_prev(1);
//	}
//
//	/* limit vertical acceleration */
//	float max_acc_z = acc(2) < 0.0f ? -_params.acc_up_max : _params.acc_down_max;
//
//	if (fabsf(acc(2)) > fabsf(max_acc_z)) 
//	{
//		_vel_sp(2) = max_acc_z * dt + _vel_sp_prev(2);
//	}
//}

//void PositionControl::update_velocity_derivative()
//{
//
//	/* Update velocity derivative,
//	 * independent of the current flight mode
//	 */
//	if (_local_pos.timestamp == 0) 
//	{
//		return;
//	}
//
//	// TODO: this logic should be in the estimator, not the controller!
//
//	if (PX4_ISFINITE(_local_pos.x) &&
//	    PX4_ISFINITE(_local_pos.y) &&
//	    PX4_ISFINITE(_local_pos.z)) 
//	{
//		_pos(0) = _local_pos.x;
//		_pos(1) = _local_pos.y;
//
//		if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) 
//		{
//			_pos(2) = -_local_pos.dist_bottom;
//		} 
//		else 
//		{
//			_pos(2) = _local_pos.z;
//		}
//	}
//
//	if (PX4_ISFINITE(_local_pos.vx) &&
//	    PX4_ISFINITE(_local_pos.vy) &&
//	    PX4_ISFINITE(_local_pos.vz)) 
//	{
//		_vel(0) = _local_pos.vx;
//		_vel(1) = _local_pos.vy;
//
//		if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) 
//		{
//			_vel(2) = -_local_pos.dist_bottom_rate;
//		} 
//		else 
//		{
//			_vel(2) = _local_pos.vz;
//		}
//	}
//
//	_vel_err_d(0) = _vel_x_deriv.update(-_vel(0));
//	_vel_err_d(1) = _vel_y_deriv.update(-_vel(1));
//	_vel_err_d(2) = _vel_z_deriv.update(-_vel(2));
//}

void PositionControl::do_control(float dt)
{

	_vel_ff.zero();

	/* by default, run position/altitude controller. the control_* functions
	 * can disable this and run velocity controllers directly in this cycle */
	_run_pos_control = true;
	_run_alt_control = true;

//	if (_control_mode.flag_control_manual_enabled) 
//	{
//		/* manual control */
//		control_manual(dt);
//		_mode_auto = false;
//		_hold_offboard_xy = false;
//		_hold_offboard_z = false;
//	} 
//	else 
//	{
		control_non_manual(dt);
//	}

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

//	limit_altitude();
	if (_run_alt_control) 
	{
		_vel_sp(2) = (_pos_sp(2) - _pos(2)) * _params.pos_p(2);
	}

	/* make sure velocity setpoint is saturated in xy*/
	// XXX is this necessary?, remove this
//	float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) + _vel_sp(1) * _vel_sp(1));
//	if (vel_norm_xy > _params.vel_max(0)) 
//	{
//		/* note assumes vel_max(0) == vel_max(1) */
//		_vel_sp(0) = _vel_sp(0) * _params.vel_max(0) / vel_norm_xy;
//		_vel_sp(1) = _vel_sp(1) * _params.vel_max(1) / vel_norm_xy;
//	}

	/* make sure velocity setpoint is saturated in z*/
	// XXX is this necessary?, remove this
//	if (_vel_sp(2) < -1.0f * _params.vel_max_up) 
//	{
//		_vel_sp(2) = -1.0f * _params.vel_max_up;
//	}

	/*
	 * Make sure downward velocity (positive Z) is limited close to ground.
	 * for now we use the home altitude and assume that our Z coordinate
	 * is initialized close to home.
	 XXX is this necessary?, remove this
	 */
//	bool close_to_ground = (-_pos(2) + _home_pos.z)  < _manual_land_alt.get();
//
//	if (close_to_ground && (_vel_sp(2) > _params.land_speed)) 
//	{
//		_vel_sp(2) = _params.land_speed;
//	} 
//	else if (_vel_sp(2) >  _params.vel_max_down) 
//	{
//		_vel_sp(2) = _params.vel_max_down;
//	}

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
		_vel_sp_prev(0) = _vel(0);
		_vel_sp_prev(1) = _vel(1);
		_vel_sp(0) = 0.0f;
		_vel_sp(1) = 0.0f;
	}

	if (!_control_mode.flag_control_climb_rate_enabled) 
	{
		_vel_sp(2) = 0.0f;
	}

	/* TODO: remove this is a pathetic leftover, it's here just to make sure that
	 * _takeoff_jumped flags are reset 
	 XXX let's remove this */
//	if (_control_mode.flag_control_manual_enabled || !_pos_sp_triplet.current.valid
//	    || _pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF
//	    || !_control_mode.flag_armed) 
//	{
//		_takeoff_jumped = false;
//		_takeoff_thrust_sp = 0.0f;
//	}

//	vel_sp_slewrate(dt);
	_vel_sp_prev = _vel_sp;

//	_global_vel_sp.timestamp = hrt_absolute_time();
//	_global_vel_sp.vx = _vel_sp(0);
//	_global_vel_sp.vy = _vel_sp(1);
//	_global_vel_sp.vz = _vel_sp(2);
//
//	/* publish velocity setpoint */
//	if (_global_vel_sp_pub != nullptr) 
//	{
//		// this is published when offboard control mode
//		// XXX this may be unnecessary, yes
//		//mavlink_log_info(&_mavlink_log_pub, "global publication?");
//		orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);
//	} else 
//	{
//		_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), 
//				&_global_vel_sp);
//	}

	//// control loop starts??
	if (_control_mode.flag_control_climb_rate_enabled || 
			_control_mode.flag_control_velocity_enabled ||
	    	_control_mode.flag_control_acceleration_enabled) 
	{
		// hss : this is performed in offboard mode
		
		// hss : do not use thrust integral
//		/* reset integrals in Z if needed */
//		if (_control_mode.flag_control_climb_rate_enabled) 
//		{
//			if (_reset_int_z) 
//			{
//				_reset_int_z = false;
//				_thrust_int(2) = 0.0f;
//			}
//		} 
//		else 
//		{
//			_reset_int_z = true;
//		}

//		/* reset integrals in XY if needed */
//		if (_control_mode.flag_control_velocity_enabled)
//		{
//			if (_reset_int_xy) 
//			{
//				_reset_int_xy = false;
//				_thrust_int(0) = 0.0f;
//				_thrust_int(1) = 0.0f;
//			}
//
//		} 
//		else 
//		{
//			_reset_int_xy = true;
//		}

		/* velocity error */
		math::Vector<3> vel_err = _vel_sp - _vel;

		/* thrust vector in NED frame */
		math::Vector<3> thrust_sp;

		// hss : acceleration is not used
//		if (_control_mode.flag_control_acceleration_enabled && 
//				_pos_sp_triplet.current.acceleration_valid) 
//		{
//			//// generate thrust_sp with current acceleration
//			thrust_sp = math::Vector<3>(_pos_sp_triplet.current.a_x, 
//					_pos_sp_triplet.current.a_y, 
//					_pos_sp_triplet.current.a_z);
//
//		} 
//		else 
//		{
			//// generate thrust sp with vel_err and vel_err_d
			// hss : this is IMPORTANT
			thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d)
				    + _thrust_int - math::Vector<3>(0.0f, 0.0f, _params.thr_hover);
//		}

		// XXX this is not necessary
//		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF && 
//				!_takeoff_jumped && 
//				!_control_mode.flag_control_manual_enabled) 
//		{
//			// for jumped takeoffs use special thrust setpoint calculated above
//			thrust_sp.zero();
//			thrust_sp(2) = -_takeoff_thrust_sp;
//		}

		// hss : this is not the case of offboard control
//		if (!_control_mode.flag_control_velocity_enabled && 
//				!_control_mode.flag_control_acceleration_enabled) 
//		{
//			thrust_sp(0) = 0.0f;
//			thrust_sp(1) = 0.0f;
//		}

		/* if still or already on ground command zero xy velcoity and 
		   zero xy thrust_sp in body frame to consider uneven ground 
		 XXX is this necessary? remove this*/
//		if (_vehicle_land_detected.ground_contact) 
//		{
//			/* thrust setpoint in body frame*/
//			math::Vector<3> thrust_sp_body = _R.transposed() * thrust_sp;
//
//			/* we dont want to make any correction in body x and y*/
//			thrust_sp_body(0) = 0.0f;
//			thrust_sp_body(1) = 0.0f;
//
//			/* make sure z component of thrust_sp_body is larger than 0 
//			   (positive thrust is downward) */
//			thrust_sp_body(2) = thrust_sp(2) > 0.0f ? thrust_sp(2) : 0.0f;
//
//			/* convert back to local frame (NED) */
//			thrust_sp = _R * thrust_sp_body;
//
//			/* set velocity setpoint to zero and reset position */
//			_vel_sp(0) = 0.0f;
//			_vel_sp(1) = 0.0f;
//			_pos_sp(0) = _pos(0);
//			_pos_sp(1) = _pos(1);
//		}

		// hss : this is not the case of offboard mode
//		if (!_control_mode.flag_control_climb_rate_enabled && 
//				!_control_mode.flag_control_acceleration_enabled) 
//		{
//			thrust_sp(2) = 0.0f;
//		}

//		/* limit thrust vector and check for saturation */
//		bool saturation_xy = false;
//		bool saturation_z = false;

//		/* limit min lift */
//		float thr_min = _params.thr_min;

		// hss : this is not the case for offboard control
//		if (!_control_mode.flag_control_velocity_enabled && thr_min < 0.0f) 
//		{
//			/* don't allow downside thrust direction in manual attitude mode */
//			thr_min = 0.0f;
//		}

//		float tilt_max = _params.tilt_max_air;
//		float thr_max = _params.thr_max;
//		// XXX : why this filter needed???, remove
//		/* filter vel_z over 1/8sec */
//		_vel_z_lp = _vel_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * _vel(2);
//		/* filter vel_z change over 1/8sec */
//		float vel_z_change = (_vel(2) - _vel_prev(2)) / dt;
//		_acc_z_lp = _acc_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * vel_z_change;

		// We can only run the control if we're already in-air, have a takeoff setpoint,
		// or if we're in offboard control.
		// Otherwise, we should just bail out
		// XXX is this necessary?, maybe not
//		const bool got_takeoff_setpoint = (_pos_sp_triplet.current.valid &&
//				_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) ||
//				_control_mode.flag_control_offboard_enabled;
//
//		if (_vehicle_land_detected.landed && !got_takeoff_setpoint) 
//		{
//			// hss : is this useful?
//			// Keep throttle low while still on ground.
//			thr_max = 0.0f;
//		} 
//		else if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
//			   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) 
//		{
//			/* adjust limits for landing mode */
//			/* limit max tilt and min lift when landing */
//			tilt_max = _params.tilt_max_land;
//
//			if (thr_min < 0.0f) 
//			{
//				thr_min = 0.0f;
//			}
//
//			/* descend stabilized, we're landing */
//			if (!_in_landing && !_lnd_reached_ground
//			    && (float)fabsf(_acc_z_lp) < 0.1f
//			    && _vel_z_lp > 0.6f * _params.land_speed) 
//			{
//				_in_landing = true;
//			}
//
//			float land_z_threshold = 0.1f;
//
//			/* assume ground, cut thrust */
//			if (_in_landing
//			    && _vel_z_lp < land_z_threshold) 
//			{
//				thr_max = 0.0f;
//				_in_landing = false;
//				_lnd_reached_ground = true;
//
//			} 
//			else if (_in_landing && 
//					_vel_z_lp < math::min(0.3f * _params.land_speed, 2.5f * land_z_threshold)) 
//			{
//				/* not on ground but with ground contact, stop position and velocity control */
//				thrust_sp(0) = 0.0f;
//				thrust_sp(1) = 0.0f;
//				_vel_sp(0) = _vel(0);
//				_vel_sp(1) = _vel(1);
//				_pos_sp(0) = _pos(0);
//				_pos_sp(1) = _pos(1);
//			}
//
//			/* once we assumed to have reached the ground always cut the thrust.
//				Only free fall detection below can revoke this	*/
//			if (!_in_landing && _lnd_reached_ground) 
//			{
//				thr_max = 0.0f;
//			}
//
//			/* if we suddenly fall, reset landing logic and remove thrust limit */
//			if (_lnd_reached_ground
//			    && (_acc_z_lp > 4.0f
//				|| _vel_z_lp > 2.0f * _params.land_speed)) 
//			{
//				thr_max = _params.thr_max;
//				_in_landing = true;
//				_lnd_reached_ground = false;
//			}
//
//		} 
//		else 
//		{
//			_in_landing = false;
//			_lnd_reached_ground = false;
//		}

		/* limit min lift */
//		if (-thrust_sp(2) < thr_min) 
//		{
//			thrust_sp(2) = -thr_min;
//			/* Don't freeze altitude integral if it wants to throttle up */
//			saturation_z = vel_err(2) > 0.0f ? true : saturation_z;
//		}

//		if (_control_mode.flag_control_velocity_enabled || 
//			_control_mode.flag_control_acceleration_enabled) 
//		{
//			// hss : this is true, but is it necessary?
//			/* limit max tilt */
//			if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) 
//			{
//				/* absolute horizontal thrust */
//				float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
//
//				if (thrust_sp_xy_len > 0.01f) 
//				{
//					/* max horizontal thrust for given vertical thrust*/
//					float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);
//
//					if (thrust_sp_xy_len > thrust_xy_max) 
//					{
//						float k = thrust_xy_max / thrust_sp_xy_len;
//						thrust_sp(0) *= k;
//						thrust_sp(1) *= k;
//						/* Don't freeze x,y integrals if they both want to throttle down */
//						saturation_xy = ((vel_err(0) * _vel_sp(0) < 0.0f) && 
//								(vel_err(1) * _vel_sp(1) < 0.0f)) ? saturation_xy : true;
//					}
//				}
//			}
//		}

//		if (_control_mode.flag_control_climb_rate_enabled && 
//			!_control_mode.flag_control_velocity_enabled) 
//		{
//			// hss : this is false
//			/* thrust compensation when vertical velocity 
//			   but not horizontal velocity is controlled */
//			float att_comp;
//
//			if (_R(2, 2) > TILT_COS_MAX) 
//			{
//				att_comp = 1.0f / _R(2, 2);
//			} 
//			else if (_R(2, 2) > 0.0f) 
//			{
//				att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _R(2, 2) + 1.0f;
//				saturation_z = true;
//			} 
//			else 
//			{
//				att_comp = 1.0f;
//				saturation_z = true;
//			}
//			thrust_sp(2) *= att_comp;
//		}

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

		// hss : remove this limitation
//		/* limit max thrust */
//		if (fabsf(thrust_body_z) > thr_max) 
//		{
//			if (thrust_sp(2) < 0.0f) 
//			{
//				if (-thrust_sp(2) > thr_max) 
//				{
//					/* thrust Z component is too large, limit it */
//					thrust_sp(0) = 0.0f;
//					thrust_sp(1) = 0.0f;
//					thrust_sp(2) = -thr_max;
//					saturation_xy = true;
//					/* Don't freeze altitude integral if it wants to throttle down */
//					saturation_z = vel_err(2) < 0.0f ? true : saturation_z;
//
//				} 
//				else 
//				{
//					/* preserve thrust Z component and lower XY, 
//					   keeping altitude is more important than position */
//					float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp(2) * thrust_sp(2));
//					float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
//					float k = thrust_xy_max / thrust_xy_abs;
//					thrust_sp(0) *= k;
//					thrust_sp(1) *= k;
//					/* Don't freeze x,y integrals if they both want to throttle down */
//					saturation_xy = ((vel_err(0) * _vel_sp(0) < 0.0f) && 
//							(vel_err(1) * _vel_sp(1) < 0.0f)) ? saturation_xy : true;
//				}
//
//			} 
//			else 
//			{
//				/* Z component is positive, going down (Z is positive down in NED), 
//				   simply limit thrust vector */
//				float k = thr_max / fabsf(thrust_body_z);
//				thrust_sp *= k;
//				saturation_xy = true;
//				saturation_z = true;
//			}
//			thrust_body_z = thr_max;
//		}

		// hss : I changed this line
//		_att_sp.thrust = math::max(thrust_body_z, thr_min);
		_att_sp.thrust = thrust_body_z;

		// hss : do not use thrust integral
//		/* update integrals */
//		if (_control_mode.flag_control_velocity_enabled && !saturation_xy) 
//		{
//			_thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
//			_thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
//		}
//
//		if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) 
//		{
//			_thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;
//		}

		// TODO : this geometric generation of attitude setpoint is good
		// but for our purpose, it is not desirable now
		// I have to modify this process with conventional way
		/* calculate attitude setpoint from thrust vector */
//		if (_control_mode.flag_control_velocity_enabled || 
//			_control_mode.flag_control_acceleration_enabled) 
//		{
			// hss : this is performed in offboard mode
			/* desired body_z axis = -normalize(thrust_vector) */
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

			/* copy quaternion setpoint to attitude setpoint topic */
			matrix::Quatf q_sp = _R_setpoint;
			memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
			_att_sp.q_d_valid = true;

			/* calculate euler angles, for logging only, must not be used for control */
			matrix::Eulerf euler = _R_setpoint;
			_att_sp.roll_body = euler(0);
			_att_sp.pitch_body = euler(1);

//		} 
//		else if (!_control_mode.flag_control_manual_enabled) 
//		{
//			// hss : this is not performed
//			/* autonomous altitude control without position control (failsafe landing),
//			 * force level attitude, don't change yaw */
//			_R_setpoint = matrix::Eulerf(0.0f, 0.0f, _att_sp.yaw_body);
//
//			/* copy quaternion setpoint to attitude setpoint topic */
//			matrix::Quatf q_sp = _R_setpoint;
//			memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
//			_att_sp.q_d_valid = true;
//
//			_att_sp.roll_body = 0.0f;
//			_att_sp.pitch_body = 0.0f;
//		}

		// hss : remove this
//		/* save thrust setpoint for logging */
//		_local_pos_sp.acc_x = thrust_sp(0) * ONE_G;
//		_local_pos_sp.acc_y = thrust_sp(1) * ONE_G;
//		_local_pos_sp.acc_z = thrust_sp(2) * ONE_G;

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
			_vel_sp_prev.zero();
			_reset_int_z = true;
			_reset_int_xy = true;
			_reset_yaw_sp = true;
			_yaw_takeoff = _yaw;
		}

		/* reset setpoints and integrators VTOL in FW mode */
		if (_vehicle_status.is_vtol && !_vehicle_status.is_rotary_wing) 
		{
			_reset_alt_sp = true;
			_reset_int_xy = true;
			_reset_int_z = true;
			_reset_pos_sp = true;
			_reset_yaw_sp = true;
			_vel_sp_prev = _vel;
		}

		//Update previous arming state
		was_armed = _control_mode.flag_armed;

		update_ref();
//		update_velocity_derivative();

		// reset the horizontal and vertical position hold flags for non-manual modes
		// or if position / altitude is not controlled
		if (!_control_mode.flag_control_position_enabled || 
			!_control_mode.flag_control_manual_enabled) 
		{
			_pos_hold_engaged = false;
		}

		if (!_control_mode.flag_control_altitude_enabled || 
			!_control_mode.flag_control_manual_enabled) 
		{
			_alt_hold_engaged = false;
		}

		if (_control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled ||
		    _control_mode.flag_control_acceleration_enabled) 
		{
			// hss : offboard control
			do_control(dt);

			//// XXX this would be unnecessary... remove
//			/* fill local position, velocity and thrust setpoint */
//			_local_pos_sp.timestamp = hrt_absolute_time();
//			_local_pos_sp.x = _pos_sp(0);
//			_local_pos_sp.y = _pos_sp(1);
//			_local_pos_sp.z = _pos_sp(2);
//			_local_pos_sp.yaw = _att_sp.yaw_body;
//			_local_pos_sp.vx = _vel_sp(0);
//			_local_pos_sp.vy = _vel_sp(1);
//			_local_pos_sp.vz = _vel_sp(2);
//
//			/* publish local position setpoint */
//			if (_local_pos_sp_pub != nullptr) 
//			{
//				orb_publish(ORB_ID(vehicle_local_position_setpoint), 
//						_local_pos_sp_pub, &_local_pos_sp);
//			} 
//			else 
//			{
//				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), 
//						&_local_pos_sp);
//			}

		} 
		else 
		{
			//// hss : manual control mode!
			//mavlink_log_info(&_mavlink_log_pub, "manual mode?");

			/* position controller disabled, reset setpoints */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_do_reset_alt_pos_flag = true;
			_mode_auto = false;
			_reset_int_z = true;
			_reset_int_xy = true;

			/* store last velocity in case a mode switch to position control occurs */
			_vel_sp_prev = _vel;
		}

		/* generate attitude setpoint from manual controls */
		if (_control_mode.flag_control_manual_enabled && 
			_control_mode.flag_control_attitude_enabled) 
		{
			//// hss : instead, manual control mode generates _att_sp here
			generate_attitude_setpoint(dt);
		} 
		else 
		{
			_reset_yaw_sp = true;
			_att_sp.yaw_sp_move_rate = 0.0f;
		}

		/* update previous velocity for velocity controller D part */
		_vel_prev = _vel;

		//// hss : always publish _att_sp_pub
		//// hss : _run_pos_control and _run_alt_control is TRUE
		//// hss : when offboard mode, "control_offboard" is called
		//// hss : vehicle control mode
		////	manual :	10011000000000
		////	offboard :	00111000111100
		//// 	attitude, and rates are always controlled
		////	in manual, position, velocity, altitude, climb_rate is controlled
		////				(x,y)	  (vx,vy)     (z)       (vz)

		//bool flag = !(_control_mode.flag_control_offboard_enabled &&
		//    !(_control_mode.flag_control_position_enabled ||
		//	_control_mode.flag_control_velocity_enabled ||
		//	_control_mode.flag_control_acceleration_enabled));
		//mavlink_log_info(&_mavlink_log_pub, "att_sp publish flag : %d", flag);
		//mavlink_log_info(&_mavlink_log_pub, "pos control : %d, alt control : %d",
		//		_run_pos_control, _run_alt_control);
		//mavlink_log_info(&_mavlink_log_pub, "ctrl flag : %d%d%d%d%d%d%d%d%d%d%d%d%d%d",
		//		_control_mode.flag_control_manual_enabled,
		//		_control_mode.flag_control_auto_enabled,
		//		_control_mode.flag_control_offboard_enabled,
		//		_control_mode.flag_control_rates_enabled,
		//		_control_mode.flag_control_attitude_enabled,
		//		_control_mode.flag_control_rattitude_enabled,
		//		_control_mode.flag_control_force_enabled,
		//		_control_mode.flag_control_acceleration_enabled,
		//		_control_mode.flag_control_velocity_enabled,
		//		_control_mode.flag_control_position_enabled,
		//		_control_mode.flag_control_altitude_enabled,
		//		_control_mode.flag_control_climb_rate_enabled,
		//		_control_mode.flag_control_termination_enabled,
		//		_control_mode.flag_control_fixed_hdg_enabled);

		if (!(_control_mode.flag_control_offboard_enabled &&
		    !(_control_mode.flag_control_position_enabled ||
			_control_mode.flag_control_velocity_enabled ||
			_control_mode.flag_control_acceleration_enabled))) 
		{
			// hss : manual thrust is computed in "position control" and saved in "att_sp"
			mavlink_log_info(&_mavlink_log_pub, 
					"r %2.4f, p %2.4f, y %2.4f, F %2.4f",
					(double)_att_sp.roll_body,
					(double)_att_sp.pitch_body,
					(double)_att_sp.yaw_body,
					(double)_att_sp.thrust);

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

		/* reset altitude controller integral (hovering throttle) to 
		   manual throttle after manual throttle control */
		_reset_int_z_manual = _control_mode.flag_armed && 
							  _control_mode.flag_control_manual_enabled && 
							  !_control_mode.flag_control_climb_rate_enabled;
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
