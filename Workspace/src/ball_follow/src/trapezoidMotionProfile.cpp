#include "trapezoidMotionProfile.h"

TrapezoidMotionProfile::TrapezoidMotionProfile(double dist,double v_max,double a_max,double update_freq)
{
    distance = dist;
    vel_max = v_max;
    accel_max = a_max;
    time_total = (distance / vel_max) + (vel_max / accel_max);
    update_frequency = update_freq;
    time_to_accel = vel_max / accel_max;
    num_of_velocities = (int)((time_total) / update_frequency) + 2;
    generate_profile();
}

double TrapezoidMotionProfile::get_vel_at_time(int index)
{
    double time = index * update_frequency;
    return get_vel_at_time(time);
}

double TrapezoidMotionProfile::get_vel_at_time(double time)
{
    if(time < time_to_accel)
    {
        return accel_max * time;
    }
    else if(time >= time_to_accel && time <= (time_total - time_to_accel))
    {
        return vel_max;
    }
    else if(time > (time_total - time_to_accel) && time < time_total)
    {
        double deceleration_point = time_total - time_to_accel;
        double time_after_deceleration_point = time - deceleration_point;
        return vel_max - (time_after_deceleration_point * accel_max);
    }
    else if(time >= time_total)
    {
        return 0;
    }
}

void TrapezoidMotionProfile::generate_profile()
{
    profile = new double[num_of_velocities];
    for(int i = 0;i < num_of_velocities;i++)
    {
        profile[i] = get_vel_at_time(i);
    }
}