#pragma once

#include <Eigen/Dense>

namespace drake {
namespace examples {
namespace allegro_hand {

// This is a temporary class that defines the inital position of the mug
class MugSetting{
public:

Eigen::Vector3d IniRotAngles{M_PI / 2, 0, 0};
Eigen::Vector3d IniTransPosition{0.095, 0.062, 0.095};

};



}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
