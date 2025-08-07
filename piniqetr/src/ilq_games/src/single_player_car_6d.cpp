//////////////////////////////////////////////////////////////////////////////
//
// Single player dynamics modeling a car. 5 states and 2 control inputs.
// State is [x, y, theta, phi, v, a], control is [omega, j], and dynamics are:
//                     \dot px    = v cos theta
//                     \dot py    = v sin theta
//                     \dot theta = (v / L) * tan phi
//                     \dot phi   = omega
//                     \dot v     = a
//                     \dot a     = j
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/dynamics/single_player_car_6d.h"

namespace e2e_noa::planning {

// Constexprs for state indices.
const Dimension SinglePlayerCar6D::kNumXDims = 6;
const Dimension SinglePlayerCar6D::kPxIdx = 0;
const Dimension SinglePlayerCar6D::kPyIdx = 1;
const Dimension SinglePlayerCar6D::kThetaIdx = 2;
const Dimension SinglePlayerCar6D::kPhiIdx = 3;
const Dimension SinglePlayerCar6D::kVIdx = 4;
const Dimension SinglePlayerCar6D::kAIdx = 5;

// Constexprs for control indices.
const Dimension SinglePlayerCar6D::kNumUDims = 2;
const Dimension SinglePlayerCar6D::kOmegaIdx = 0;
const Dimension SinglePlayerCar6D::kJerkIdx = 1;

}  // namespace e2e_noa::planning
