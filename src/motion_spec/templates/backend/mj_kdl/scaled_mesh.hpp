// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace motion_spec {

inline bool write_scaled_mesh(
    const std::filesystem::path &source, const std::filesystem::path &target,
    const std::vector<double> &sizes) {
    if (sizes.size() != 3) return false;
    for (double value : sizes) {
        if (!std::isfinite(value) || value <= 0.0) return false;
    }

    std::ifstream input(source);
    std::ofstream output(target);
    if (!input || !output) return false;
    output << std::setprecision(17);
    std::string line;
    while (std::getline(input, line)) {
        if (line.rfind("v ", 0) == 0) {
            std::istringstream row(line.substr(2));
            std::array<double, 3> point{};
            if (!(row >> point[0] >> point[1] >> point[2])) return false;
            for (int i = 0; i < 3; ++i) point[i] *= sizes[i];
            if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) return false;
            output << "v " << point[0] << ' ' << point[1] << ' ' << point[2] << '\n';
        } else {
            output << line << '\n';
        }
    }
    return input.eof() && output.good();
}

} // namespace motion_spec
