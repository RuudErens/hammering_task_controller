#include "HammeringTaskNew_Initial.h"
#include <mc_rtc/logging.h>

#include "../HammeringTaskNew.h"

// TODO: delete the commented out parts as there are general example lines. Do this once the project works

void HammeringTaskNew_Initial::configure(const mc_rtc::Configuration & config)
{
}

void HammeringTaskNew_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);

  // Creates a button to start the movement
  ctl_.gui()->addElement({}, mc_rtc::gui::Button("Start hammering", [this]() { _positionning_hammer_clicked = true; }));
  ctl_.getPostureTask(ctl_.robot().name())->weight(1);
  mc_rtc::log::info("Starting Initial State");

}

bool HammeringTaskNew_Initial::run(mc_control::fsm::Controller & ctl_)
{
  // auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
  // output("OK");

  if (_positionning_hammer_clicked)
  {
      output("BUTTON_CLICKED");
      return true;
  }
  return false;
}

void HammeringTaskNew_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  // auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
  ctl_.gui()->removeElement({}, "Start hammering");
}

EXPORT_SINGLE_STATE("HammeringTaskNew_Initial", HammeringTaskNew_Initial)
