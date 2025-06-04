/*
 * @Author: Ryoma Liu -- ROBOLAND
 * @Date: 2021-11-27 15:47:05
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 19:15:40
 */

#include "controller/pid_speed_controller.h"
#include "controller/inverse_kinematics.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
const double PI = 3.141592653589793238463;

using namespace std;

namespace turtle_namespace
{
    namespace control
    {

        void PID_speed_controller::Init()
        {
            t = 0;
        }

        void PID_speed_controller::ComputeControlCommand(turtle &turtle_)
        {
            // if(turtle_.traveler_gui.drag_speed > 1){
            //     righttriangular_gait_params[1].period_down = 0.1 * traveler_.traveler_gui.drag_speed;
            //     righttriangular_gait_params[1].period_hori = 0.1 * traveler_.traveler_gui.drag_speed;
            //     righttriangular_gait_params[1].period_incline = 0.1 * traveler_.traveler_gui.drag_speed;
            // }
            // if(turtle_.turtle_gui.start_flag==1){
            //     t = t + 0.01;
            //     boundingGAIT(turtle_, t, theta1, gamma1, beta1, theta2, gamma2, beta2);
            //     // cout << theta1 <<" "<< gamma1 <<" "<< beta1 <<" "<< theta2 <<" "<< gamma2 <<" "<< beta2 << endl;

            //     turtle_.turtle_control.left_limb_motor.motor_control_position = theta1;
            //     turtle_.turtle_control.left_big_servo_command = gamma1;
            //     turtle_.turtle_control.left_small_servo_command = beta1;

            //     turtle_.turtle_control.right_limb_motor.motor_control_position = theta2;
            //     turtle_.turtle_control.right_big_servo_command = gamma2;
            //     turtle_.turtle_control.right_small_servo_command = beta2;

            //     // update the current state
            //     turtle_.turtle_chassis.left_big_servo_pos = gamma1;
            //     turtle_.turtle_chassis.left_small_servo_pos = beta1;
            //     turtle_.turtle_chassis.right_big_servo_pos = gamma2;
            //     turtle_.turtle_chassis.right_small_servo_pos = beta2;
            // }
            // else{
            //     t = 0;
            //     // boundingGAIT(turtle_, t, theta1, gamma1, beta1, theta2, gamma2, beta2);
            //     // cout << theta1 <<" "<< gamma1 <<" "<< beta1 <<" "<< theta2 <<" "<< gamma2 <<" "<< beta2 << endl;

            //     // turtle_.turtle_control.left_limb_motor.motor_control_position = turtle_.turtle_chassis.left_limb_motor.position + 0.0001*(0.2 - turtle_.turtle_chassis.left_limb_motor.position);
            //     turtle_.turtle_control.left_limb_motor.motor_control_position = 0.12;
            //     turtle_.turtle_control.left_big_servo_command = 100;
            //     turtle_.turtle_control.left_small_servo_command = 0;

            //     // turtle_.turtle_control.right_limb_motor.motor_control_position = turtle_.turtle_chassis.right_limb_motor.position + 0.0001*(1.2 - turtle_.turtle_chassis.right_limb_motor.position);
            //     turtle_.turtle_control.right_limb_motor.motor_control_position = 1.2;
            //     turtle_.turtle_control.right_big_servo_command = 23;
            //     turtle_.turtle_control.right_small_servo_command = 0;

            //     // update the current state
            //     turtle_.turtle_chassis.left_big_servo_pos = 100;
            //     turtle_.turtle_chassis.left_small_servo_pos = 0;
            //     turtle_.turtle_chassis.right_big_servo_pos = 23;
            //     turtle_.turtle_chassis.right_small_servo_pos = 0;
            // }
            


            // t = t + 0.01;
            // int count;
            // Rectangle_Params rectangle_params = {0.3f, .3f, .3f, .3f, 0.17f, 0.17f, 0.5f};
            // float t_mod = fmod(t, (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right));
            // if(t_mod > rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right) {
                
            //     count++;
                
            //     t = t - 0.01 * count;
                
                
            //     boundingGAIT(t, theta1, gamma1, beta1, theta2, gamma2, beta2);
                
                
            //     turtle_.turtle_control.left_big_servo_command = gamma1;
            //     turtle_.turtle_control.left_small_servo_command = beta1;

                
            //     turtle_.turtle_control.right_big_servo_command = gamma2;
            //     turtle_.turtle_control.right_small_servo_command = beta2;

            //     // update the current state
            //     turtle_.turtle_chassis.left_big_servo_pos = gamma1;
            //     turtle_.turtle_chassis.left_small_servo_pos = beta1;

            //     turtle_.turtle_chassis.right_big_servo_pos = gamma2;
            //     turtle_.turtle_chassis.right_small_servo_pos = beta2;

            //     t = t + 0.01 * count;
            //     boundingGAIT(t, theta1, gamma1, beta1, theta2, gamma2, beta2);
                
            //     turtle_.turtle_control.left_limb_motor.motor_control_position = theta1;

            //     turtle_.turtle_control.right_limb_motor.motor_control_position = theta2;
            // }
            // else {
            //     count = 0;
            //     boundingGAIT(t, theta1, gamma1, beta1, theta2, gamma2, beta2);
            //     // cout << theta1 <<" "<< gamma1 <<" "<< beta1 <<" "<< theta2 <<" "<< gamma2 <<" "<< beta2 << endl;

            //     turtle_.turtle_control.left_limb_motor.motor_control_position = theta1;
            //     turtle_.turtle_control.left_big_servo_command = gamma1;
            //     turtle_.turtle_control.left_small_servo_command = beta1;

            //     turtle_.turtle_control.right_limb_motor.motor_control_position = theta2;
            //     turtle_.turtle_control.right_big_servo_command = gamma2;
            //     turtle_.turtle_control.right_small_servo_command = beta2;

            //     // update the current state
            //     turtle_.turtle_chassis.left_big_servo_pos = gamma1;
            //     turtle_.turtle_chassis.left_small_servo_pos = beta1;
            //     turtle_.turtle_chassis.right_big_servo_pos = gamma2;
            //     turtle_.turtle_chassis.right_small_servo_pos = beta2;
            // }

        }

        void PID_speed_controller::Reset() { is_init = false; }

        void PID_speed_controller::Stop()
        {
        }

    } // namespace control
} // namespace turtle_namespace
