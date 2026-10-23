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

inline bool write_affine_mesh(
    const std::filesystem::path &source, const std::filesystem::path &target,
    const std::string &kind, const std::vector<double> &sizes,
    const std::array<double, 9> &matrix) {
    std::array<double, 3> factors{};
    if (kind == "sphere" && sizes.size() == 1) {
        factors = {sizes[0], sizes[0], sizes[0]};
    } else if (kind == "cylinder" && sizes.size() == 2) {
        factors = {sizes[0], sizes[0], sizes[1]};
    } else if (sizes.size() == 3) {
        factors = {sizes[0], sizes[1], sizes[2]};
    } else {
        return false;
    }
    for (double value : factors) {
        if (!std::isfinite(value) || value <= 0.0) return false;
    }

    std::ifstream input(source);
    std::ofstream output(target);
    if (!input || !output) return false;
    output << std::setprecision(17);
    const double determinant =
        matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
        matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
        matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
    if (!std::isfinite(determinant) || determinant == 0.0) return false;
    std::string line;
    while (std::getline(input, line)) {
        if (line.rfind("v ", 0) == 0) {
            std::istringstream row(line.substr(2));
            std::array<double, 3> point{};
            if (!(row >> point[0] >> point[1] >> point[2])) return false;
            for (int i = 0; i < 3; ++i) point[i] *= factors[i];
            const std::array<double, 3> mapped = {
                matrix[0] * point[0] + matrix[1] * point[1] + matrix[2] * point[2],
                matrix[3] * point[0] + matrix[4] * point[1] + matrix[5] * point[2],
                matrix[6] * point[0] + matrix[7] * point[1] + matrix[8] * point[2],
            };
            if (!std::isfinite(mapped[0]) || !std::isfinite(mapped[1]) || !std::isfinite(mapped[2])) return false;
            output << "v " << mapped[0] << ' ' << mapped[1] << ' ' << mapped[2] << '\n';
        } else if (determinant < 0.0 && line.rfind("f ", 0) == 0) {
            std::istringstream row(line.substr(2));
            std::array<std::string, 3> face;
            if (!(row >> face[0] >> face[1] >> face[2])) return false;
            output << "f " << face[0] << ' ' << face[2] << ' ' << face[1] << '\n';
        } else {
            output << line << '\n';
        }
    }
    return input.eof() && output.good();
}

} // namespace motion_spec
