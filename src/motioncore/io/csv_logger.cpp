#include "motioncore/io/csv_logger.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace motioncore::io {

void write_simulation_csv(const std::filesystem::path& path,
                          const std::vector<metrics::SimulationSample>& samples) {
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("cannot open CSV output: " + path.string());
    }
    output << "time_s,target_position_mm,actual_position_mm,target_velocity_mm_s,"
              "actual_velocity_mm_s,actual_acceleration_mm_s2,drive_force_N,"
              "load_force_N,position_error_mm\n";
    output << std::setprecision(12);
    for (const auto& sample : samples) {
        output << sample.time_s << ','
               << sample.target_position_m * 1000.0 << ','
               << sample.actual_position_m * 1000.0 << ','
               << sample.target_velocity_mps * 1000.0 << ','
               << sample.actual_velocity_mps * 1000.0 << ','
               << sample.actual_acceleration_mps2 * 1000.0 << ','
               << sample.drive_force_n << ','
               << sample.load_force_n << ','
               << sample.position_error_m * 1000.0 << '\n';
    }
    if (!output) {
        throw std::runtime_error("failed while writing CSV: " + path.string());
    }
}

}  // namespace motioncore::io
