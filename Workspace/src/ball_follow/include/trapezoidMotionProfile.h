//
// Created by Sheikh on 10/17/2018.
//

#ifndef TRAPEZOIDMOTIONPROFILE_TRAPEZOIDMOTIONPROFILE_H
#define TRAPEZOIDMOTIONPROFILE_TRAPEZOIDMOTIONPROFILE_H
class TrapezoidMotionProfile
{
private:
    double distance, vel_max, accel_max, time_total, update_frequency, time_to_accel;
public:
    int num_of_velocities;
    double *profile;
    TrapezoidMotionProfile(double dist,double v_max,double a_max,double update_freq);
    double get_vel_at_time(int index);
    double get_vel_at_time(double time);
    void generate_profile();
};
#endif //TRAPEZOIDMOTIONPROFILE_TRAPEZOIDMOTIONPROFILE_H
