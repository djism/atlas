#pragma once
#include "atlas/types.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace atlas {
// 8-bit grayscale image, row-major, row 0 at the TOP. 255 = white.
struct GrayImage {
    std::size_t width = 0;
    std::size_t height = 0;
    std::vector<std::uint8_t> pixels;  // width * height
    std::uint8_t at(std::size_t row, std::size_t column) const { return pixels[row * width + column]; }
};

// Top-down view of points, ignoring z. The window is a square of +/-
// half_extent meters around `center`, in the points' own frame (x forward,
// y left), `resolution` meters per pixel: forward (+x) is UP, left (+y) is LEFT. White background; a pixel darkens with the number of
// points in it (log scale, black at 16 or more). Points outside are skipped.
// Throws std::invalid_argument for nonpositive or nonfinite sizes, or a window
// over 8192 pixels wide.
GrayImage birdsEyeView(const std::vector<Point3D>& points, double half_extent, double resolution,
                       const Point2D& center = {0, 0});

// Writes an uncompressed 24-bit BMP (opens in Preview and browsers).
// Throws std::runtime_error if the file cannot be written.
void writeBmp(const GrayImage& image, const std::string& path);
}  // namespace atlas
