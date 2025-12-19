#include "HammeringTaskNew_Initial.h"
#include <mc_rtc/logging.h>

#include "../HammeringTaskNew.h"

void HammeringTaskNew_Initial::configure(const mc_rtc::Configuration & config)
{
}

void HammeringTaskNew_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

  // Creates a button to start the movement
  ctl.gui()->addElement({}, mc_rtc::gui::Button("Start hammering", [this]() { _positionning_hammer_clicked = true; }));
  ctl.getPostureTask(ctl_.robot().name())->stiffness(100);
  mc_rtc::log::info("Starting Initial State");
  total_time_elapsed = 0.0f;
}

bool HammeringTaskNew_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
  // static_cast<TVMImpulseConstraint *>(constraint_.get())
  // static_cast<TVMImpulseConstraint *>(ctl.impulseConstraint->getConstraint().get())->impFunctionLow()->runUpdateA();
  // ctl.impulseConstraint->getConstraint().get()->impFunctionLow()->runUpdateB();
  // ctl.impulseConstraint->getConstraint().get()->impFunctionHigh()->runUpdateB();
  // ctl.impulseConstraint->getConstraint().get()->impFunctionLow()->runUpdateJacobian();
  // ctl.impulseConstraint->getConstraint().get()->impFunctionHigh()->runUpdateJacobian();
  total_time_elapsed += ctl_.solver().dt();
  if (_positionning_hammer_clicked || total_time_elapsed > 1.5f)
  {
      output("BUTTON_CLICKED");
      return true;
  }
  return false;
}

void HammeringTaskNew_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
  ctl_.gui()->removeElement({}, "Start hammering");
}

EXPORT_SINGLE_STATE("HammeringTaskNew_Initial", HammeringTaskNew_Initial)
