#include "Arduino.h"
#include "kinematics.h"

Kinematics::Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                       float motor_operating_voltage, float motor_power_max_voltage,
                       float wheel_diameter, float wheels_y_distance, float min_pwm,
                       float max_pwm) : base_platform_(robot_base),
                                        wheels_y_distance_(wheels_y_distance),
                                        wheel_circumference_(PI * wheel_diameter),
                                        total_wheels_(getTotalWheels(robot_base)),
                                        min_pwm_(min_pwm),
                                        max_pwm_(max_pwm)
{
    motor_power_max_voltage = constrain(motor_power_max_voltage, 0, motor_operating_voltage);
    max_rpm_ = ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    return calculateRPM(linear_x, linear_y, angular_z);
}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{

    float tangential_vel = angular_z * (wheels_y_distance_ / 2.0);

    // convert m/s to m/min
    float linear_vel_x_mins = linear_x * 60.0;
    float linear_vel_y_mins = linear_y * 60.0;
    // convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel * 60.0;

    float x_rpm = linear_vel_x_mins / wheel_circumference_;
    float y_rpm = linear_vel_y_mins / wheel_circumference_;
    float tan_rpm = tangential_vel_mins / wheel_circumference_;

    float a_x_rpm = fabs(x_rpm);
    float a_y_rpm = fabs(y_rpm);
    float a_tan_rpm = fabs(tan_rpm);

    float xy_sum = a_x_rpm + a_y_rpm;
    float xtan_sum = a_x_rpm + a_tan_rpm;

    // calculate the scale value how much each target velocity
    // must be scaled down in such cases where the total required RPM
    // is more than the motor's max RPM
    // this is to ensure that the required motion is achieved just with slower speed
    if (xy_sum >= max_rpm_ && angular_z == 0)
    {
        float vel_scaler = max_rpm_ / xy_sum;

        x_rpm *= vel_scaler;
        y_rpm *= vel_scaler;
    }

    else if (xtan_sum >= max_rpm_ && linear_y == 0)
    {
        float vel_scaler = max_rpm_ / xtan_sum;

        x_rpm *= vel_scaler;
        tan_rpm *= vel_scaler;
    }

    Kinematics::rpm rpm;

    // calculate for the target encoder RPM and direction
    // Inverse Swerve Drive
    //front-left motor
    rpm.motor1 = sqrt(((x_rpm-(tan_rpm*(wheels_y_distance_/2)))**2) + ((y_rpm+(tan_rpm*(wheels_y_distance_/2)))**2));
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

    //front-right motor
    rpm.motor2 = sqrt(((x_rpm-(tan_rpm*(wheels_y_distance_/2)))**2) + ((y_rpm+(tan_rpm*(wheels_y_distance_/2)))**2));
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

    //back-left motor
    rpm.motor3 = sqrt(((x_rpm-(tan_rpm*(wheels_y_distance_/2)))**2) + ((y_rpm+(tan_rpm*(wheels_y_distance_/2)))**2));
    rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

    //back-right motor
    rpm.motor4 = sqrt(((x_rpm-(tan_rpm*(wheels_y_distance_/2)))**2) + ((y_rpm+(tan_rpm*(wheels_y_distance_/2)))**2));
    rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);

    // Angle robot
    rpm.rot = atan((y_rpm+(tan_rpm*(wheels_y_distance_/2))) / (x_rpm-(tan_rpm*(wheels_y_distance_/2))));

    return rpm;
}

Kinematics::pwm Kinematics::getPWM(float pwm[])
{
    return calculatePWM(pwm[]);
}

Kinematics::pwm Kinematics::calculatePWM(float pwm[])
{
    Kinematics::pwm pwm;

    pwm.motor1 = pwm[0];
    pwm.motor1 = constrain(pwm.motor1, -max_pwm_, max_pwm_);
    pwm.rot1 = pwm[1];
    pwm.rot1 = constrain(pwm.rot1, -max_pwm_, max_pwm_);
    pwm.motor2 = pwm[2];
    pwm.motor2 = constrain(pwm.motor2, -max_pwm_, max_pwm_);
    pwm.rot2 = pwm[3];
    pwm.rot2 = constrain(pwm.rot2, -max_pwm_, max_pwm_);
    pwm.motor3 = pwm[4];
    pwm.motor3 = constrain(pwm.motor3, -max_pwm_, max_pwm_);
    pwm.rot3 = pwm[5];
    pwm.rot3 = constrain(pwm.rot3, -max_pwm_, max_pwm_);
    pwm.motor4 = pwm[6];
    pwm.motor4 = constrain(pwm.motor4, -max_pwm_, max_pwm_);
    pwm.rot4 = pwm[7];
    pwm.rot4 = constrain(pwm.rot4, -max_pwm_, max_pwm_);

    return pwm;
}

// <--change current vel/odom formal input-->
Kinematics::velocities Kinematics::getVelocities(float rpm[])
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    // Forward Swerve Drive
    //OutputSpeeds(vx, vy) for every module, calculated from module speed, angle and wheel rotation direction
    double vel_robot[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double inv_robotState[3][8] = {{0.25, 0, 0.25, 0, 0.25, 0, 0.25, 0}, 
                                  {0.0, 0.25, 0.0, 0.25, 0.0, 0.25, 0.0, 0.25}, 
                                  {-0.1767767,  0.1767767,  0.1767767,  0.1767767, -0.1767767, -0.1767767, 0.1767767, -0.1767767}};
    double VxVys[2];
    // Front Left
    calculateVxVy(VxVys, (rpm[0]* wheel_circumference_)/ 60.0, rpm[1]);
    vel_robot[0] = VxVys[0]; //vx
    vel_robot[1] = VxVys[1]; //vy
    // Front Right
    calculateVxVy(VxVys, (rpm[2]* wheel_circumference_)/ 60.0, rpm[3]);
    vel_robot[2] = VxVys[0];
    vel_robot[3] = VxVys[1];
    // Back Left
    calculateVxVy(VxVys, (rpm[4]* wheel_circumference_)/ 60.0, rpm[5]);
    vel_robot[4] = VxVys[0];
    vel_robot[5] = VxVys[1];
    // Back Right
    calculateVxVy(VxVys, (rpm[6]* wheel_circumference_)/ 60.0, rpm[7]);
    vel_robot[6] = VxVys[0];
    vel_robot[7] = VxVys[1];
    
    vel.linear_x = ((float)(vel_robot[0] + vel_robot[2] + vel_robot[4] + vel_robot[6]) / 4.0) ; // m/s
    vel.linear_y = ((float)(vel_robot[1] + vel_robot[3] + vel_robot[5] + vel_robot[7]) / 4.0) ; // m/s

    double average_vy = (sqrt((rpm[0]* wheel_circumference_)**2-(vel_robot[0]**2)) + sqrt((rpm[2]* wheel_circumference_)**2-(vel_robot[2]**2))
                        + sqrt((rpm[4]* wheel_circumference_)**2-(vel_robot[4]**2)) + sqrt((rpm[6]* wheel_circumference_)**2-(vel_robot[6]**2)) / 4.0);
    average_rps_a = ((vel_robot[1] + vel_robot[3] + vel_robot[5] + vel_robot[7] / 4.0) - average_vy) / 60.0;
    vel.angular_z = (float)average_rps_a / (wheels_y_distance_ / 2.0); //  rad/s

    return vel;
}

//input angle:deg
Kinematics::pwm Kinematics::calculateVxVy(double retVxVy[], double currentSpeed, double currentAngle)
{
    double temp_vx, temp_vy;
    double temp_angle = currentAngle * PI / 180;
    temp_vx = sqrt((currentSpeed * currentSpeed)/(1 + tan(temp_angle) * tan(temp_angle)));
    temp_vy = tan(temp_angle) * temp_vx;
    if(temp_vy < 0) temp_vy *= -1;
    if(currentSpeed >= 0 && currentAngle >= 0) 
    {
        temp_vx = temp_vx;
        temp_vy = temp_vy;
    }
    else if (currentSpeed > 0 && currentAngle < 0)
    {
        temp_vx = temp_vx;
        temp_vy = temp_vy * -1;  //vy
    }
    else if (currentSpeed < 0 && currentAngle < 0)
    {
        temp_vx = temp_vx * -1; //vx
        temp_vy = temp_vy;  //vy
    }
    else
    {
        temp_vx = temp_vx * -1;
        temp_vy = temp_vy * -1;
    }
    retVxVy[0] = temp_vx;
    retVxVy[1] = temp_vy;
}

int Kinematics::getTotalWheels(base robot_base)
{
    switch (robot_base)
    {
    case SWERVE:
        return 4;
    default:
        return 2;
    }
}

float Kinematics::getMaxRPM()
{
    return max_rpm_;
}