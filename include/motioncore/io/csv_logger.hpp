#pragma once

#include "motioncore/metrics/metrics.hpp"

#include <filesystem>
#include <vector>

namespace motioncore::io {

void write_simulation_csv(const std::filesystem::path& path,
                          const std::vector<metrics::SimulationSample>& samples);

// 简短别名，便于测试和小型示例直接调用。
inline void write_csv(const std::filesystem::path& path,
                      const std::vector<metrics::SimulationSample>& samples) {
    write_simulation_csv(path, samples);
}

}  // namespace motioncore::io
