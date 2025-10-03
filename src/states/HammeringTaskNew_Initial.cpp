#include "HammeringTaskNew_Initial.h"

#include "../HammeringTaskNew.h"

void HammeringTaskNew_Initial::configure(const mc_rtc::Configuration & config)
{
}

void HammeringTaskNew_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
}

bool HammeringTaskNew_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
  output("OK");
  return true;
}

void HammeringTaskNew_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTaskNew &>(ctl_);
}

EXPORT_SINGLE_STATE("HammeringTaskNew_Initial", HammeringTaskNew_Initial)
