#include "moordyn_tether/moordyn_tether_winch.hpp"

#include <algorithm>

namespace moordyn_tether
{

void VirtualWinch::setConfig(const WinchConfig & config)
{
    cfg_ = config;
}

void VirtualWinch::update(MoorDynLine line, const std::array<BodyState, 2> & body_states, double dt)
{
    if (!cfg_.enabled || !line)
    {
        return;
    }

    const double span = Kinematics::distance3(body_states[0].anchor_pos,
                                              body_states[1].anchor_pos);
    double desired = span * cfg_.slack_factor;

    if (desired < cfg_.min_length)
    {
        desired = cfg_.min_length;
    }
    if (desired > cfg_.max_length)
    {
        desired = cfg_.max_length;
    }

    if (dt <= 0.0) {
        // Before the initialisation: the line starts at the desired length
        MoorDyn_SetLineUnstretchedLength(line, desired);
        MoorDyn_SetLineUnstretchedLengthVel(line, 0.0);
        return;
    }
    // The length is paid out through its rate, so that MoorDyn ramps it over its internal substeps: a jump of the
    // length at every step of the node strains a stiff line in steps and rings its axial modes
    double current = 0.0;
    MoorDyn_GetLineUnstretchedLength(line, &current);
    MoorDyn_SetLineUnstretchedLengthVel(line, (desired - current) / dt);
}

}  // namespace moordyn_tether
