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

// One line of a trajectory plot. Points are positions in a frame with x
// forward and y left (for KITTI: the first scan's Velodyne frame, z dropped).
struct PlotSeries {
    std::string name;
    std::string color;  // an SVG color, e.g. "#1f5fbf"
    std::vector<Point2D> points;
};

// Top-down SVG of trajectories on equal axes, oriented like birdsEyeView:
// +x (forward) is UP, +y (left) is LEFT. Draws a title, a legend, a scale bar
// and a ring at the first series' first point (the start). Series are drawn
// in order, so the last one is on top. Throws std::invalid_argument if there
// are no points or any coordinate is not finite.
std::string trajectorySvg(const std::vector<PlotSeries>& series, const std::string& title);
}  // namespace atlas
