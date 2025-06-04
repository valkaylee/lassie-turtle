/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 20:20:11 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 19:01:12
 */

#include "controller/controller_monitor.h"

namespace turtle_namespace{
namespace control{
void ControllerMonitor::Init(){
    pid_controller_ptr_ = new PID_speed_controller();
    pid_controller_ptr_->Init();
}

void ControllerMonitor::ControlCommand(turtle& turtle_){
    pid_controller_ptr_->ComputeControlCommand(turtle_);
}








} //name space control
} //name space turtle_namespace