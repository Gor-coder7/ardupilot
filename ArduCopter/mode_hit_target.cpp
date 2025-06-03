#include "Copter.h"

#if MODE_HITTARGET_ENABLED

static bool guided_pos_terrain_alt; 
static Vector3f guided_accel_target_cmss;
static Vector3f guided_vel_target_cms; 
static uint32_t update_time_ms; 
static Vector3p guided_pos_target_cm; 

bool ModeHitTarget::init(bool ignore_checks)
{
    if (!copter.position_ok() && !ignore_checks) {
        return false;
    }
    return true;
}


void ModeHitTarget::run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    float terr_offset = 0.0f;
    if (guided_pos_terrain_alt && !wp_nav->get_terrain_offset_cm(terr_offset)) {
        copter.failsafe_terrain_on_event();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    guided_accel_target_cmss.zero();
    guided_vel_target_cms.zero();

    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    float pos_offset_z_buffer = 0.0;
    if (guided_pos_terrain_alt) {
        pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin_m() * 100.0, 0.5 * fabsF(guided_pos_target_cm.z));
    }
    pos_control->input_pos_NEU_cm(guided_pos_target_cm, terr_offset, pos_offset_z_buffer);
    
    
    Vector3f curr_pos = inertial_nav.get_position_neu_cm();
    Vector3f target_vec = guided_pos_target_cm.tofloat() - curr_pos;
    Vector3f desired_vel;
    desired_vel = target_vec.normalized() * 2000;
    pos_control->input_vel_accel_NE_cm(desired_vel.xy(), Vector2f(), false);
    pos_control->input_vel_accel_U_cm(desired_vel.z, 0, false);
    
    pos_control->update_NE_controller();
    pos_control->update_U_controller();
    
    attitude_control->input_thrust_vector_heading_cd(pos_control->get_thrust_vector(), auto_yaw.get_heading());

}


#endif 
