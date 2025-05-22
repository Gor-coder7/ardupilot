#include "Copter.h"

PID servo_pid(10, 0, 0, 0);

bool ModeNewMode::init(bool ignore_checks)
{
    hal.console->begin(115200);
    hal.console->printf("Starting test...\n");
    return true;
}

void ModeNewMode::run()
{
    float Roll = AP::ahrs().get_roll() * 180.0f / M_PI;
    hal.console->printf("Roll: %.2f\n",Roll);
    // Deg = 1500 + 10 * attitude_control->get_att_target_euler_rad() * 180.0f / M_PI;
    float pwm = 1500 + servo_pid.get_pid(Roll);
    SRV_Channels::set_output_pwm_chan(6,pwm);
    //hal.scheduler->delay(100);
}
