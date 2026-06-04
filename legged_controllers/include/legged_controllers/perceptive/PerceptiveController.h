//
// Perceptive controller entry point.
//

#pragma once

#include <legged_controllers/LeggedController.h>

namespace legged {

class PerceptiveController : public LeggedController {
 protected:
  void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) override;

  void setupMpc() override;
};

}  // namespace legged
