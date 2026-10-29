#ifndef MOORDYN_TETHER__MOORDYN_TETHER_WINCH_HPP_
#define MOORDYN_TETHER__MOORDYN_TETHER_WINCH_HPP_

#include <string>
#include <array>
#include "moordyn/MoorDyn2.h"
#include "moordyn_tether/moordyn_tether_kinematics.hpp"

namespace moordyn_tether
{

/// @brief Configuration for the virtual winch, which keeps the length at the straight-line distance times a slack
/// factor, L = d / (1 - eps_0).
struct WinchConfig
{
    bool   enabled{false};
    double slack_factor{1.15};   ///< Multiplier on straight-line distance (>1.0).

    // --- Constraints ---
    double min_length{3.0};      ///< Minimum unstretched length [m].
    double max_length{100.0};    ///< Maximum unstretched length [m].
};

/// @brief Virtual Winch logic for adjusting the tether length.
class VirtualWinch
{
public:
    VirtualWinch() = default;

    /// Update the winch configuration.
    void setConfig(const WinchConfig & config);

    /// Get current configuration.
    const WinchConfig & getConfig() const { return cfg_; }

    /// @brief Computes the desired unstretched length and applies it to MoorDyn.
    void update(MoorDynLine line, const std::array<BodyState, 2> & body_states, double dt);

private:
    WinchConfig cfg_;
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__MOORDYN_TETHER_WINCH_HPP_
