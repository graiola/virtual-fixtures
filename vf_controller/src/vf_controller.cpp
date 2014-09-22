#include "vf_controller/vf_controller.h"

using namespace vf_controller;

/*PLUGINLIB_DECLARE_CLASS(vf_controller,
                        VFController,
                        vf_controller::VFController,
                        controller_interface::ControllerBase)*/

//PLUGINLIB_EXPORT_CLASS(controller::Controller, controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(vf_controller::VFController, controller_interface::ControllerBase);
