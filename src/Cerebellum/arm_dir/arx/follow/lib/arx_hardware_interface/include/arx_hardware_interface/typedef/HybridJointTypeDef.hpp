#pragma once

namespace arx {
namespace hw_interface {
typedef struct HybridJointCmd {
  double position;
  double velocity;
  double torque;
  double k_p;
  double k_d;
} HybridJointCmd;

typedef struct HybridJointStatus {
  double position;
  double velocity;
  double torque;
  double current;
} HybridJointStatus;
} // namespace hw_interface
} // namespace arx
