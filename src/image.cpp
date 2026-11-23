#include "atlas/image.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace atlas {
GrayImage birdsEyeView(const std::vector<Point3D>& points, double half_extent, double resolution,
                       const Point2D& center) {
    if (!(half_extent > 0) || !(resolution > 0) || !std::isfinite(half_extent / resolution))
        throw std::invalid_argument("birdsEyeView: half_extent and resolution must be positive");
    const double cells = 2 * half_extent / resolution;
    if (cells > 8192) throw std::invalid_argument("birdsEyeView: window over 8192 pixels");
    const auto size = static_cast<std::size_t>(std::ceil(cells - 1e-9));
    std::vector<unsigned> counts(size * size, 0);
    for (const Point3D& p : points) {
        // Row from the top: forward distance remaining to the window's far edge.
        const double row = std::floor((half_extent - (p.x - center.x)) / resolution);
        const double column = std::floor((half_extent - (p.y - center.y)) / resolution);  // +y (left) -> small column
        if (row < 0 || column < 0 || row >= size || column >= size) continue;
        ++counts[static_cast<std::size_t>(row) * size + static_cast<std::size_t>(column)];
    }
    GrayImage image{size, size, std::vector<std::uint8_t>(size * size, 255)};
    const double saturate = std::log1p(16.0);
    for (std::size_t i = 0; i < counts.size(); ++i) {
        if (!counts[i]) continue;
        const double darkness = std::min(1.0, std::log1p(counts[i]) / saturate);
        image.pixels[i] = static_cast<std::uint8_t>(std::lround(200 * (1 - darkness)));  // even one point shows
    }
    return image;
}

void writeBmp(const GrayImage& image, const std::string& path) {
    const std::size_t row_bytes = (3 * image.width + 3) / 4 * 4;  // rows padded to 4 bytes
    const std::size_t data_bytes = row_bytes * image.height;
    const std::size_t file_bytes = 54 + data_bytes;
    const auto le = [](std::vector<unsigned char>& out, std::uint32_t v, int n) {
        for (int i = 0; i < n; ++i) out.push_back(static_cast<unsigned char>(v >> (8 * i)));
    };
    std::vector<unsigned char> out;
    out.reserve(file_bytes);
    out.push_back('B');
    out.push_back('M');
    le(out, static_cast<std::uint32_t>(file_bytes), 4);
    le(out, 0, 4);   // reserved
    le(out, 54, 4);  // offset to pixels
    le(out, 40, 4);  // BITMAPINFOHEADER size
    le(out, static_cast<std::uint32_t>(image.width), 4);
    le(out, static_cast<std::uint32_t>(image.height), 4);  // positive: rows stored bottom-up
    le(out, 1, 2);   // planes
    le(out, 24, 2);  // bits per pixel
    le(out, 0, 4);   // no compression
    le(out, static_cast<std::uint32_t>(data_bytes), 4);
    le(out, 2835, 4);  // 72 dpi
    le(out, 2835, 4);
    le(out, 0, 4);
    le(out, 0, 4);
    for (std::size_t r = image.height; r-- > 0;) {  // bottom row first
        for (std::size_t c = 0; c < image.width; ++c) {
            const unsigned char v = image.at(r, c);
            out.insert(out.end(), {v, v, v});
        }
        out.resize(out.size() + row_bytes - 3 * image.width, 0);
    }
    std::ofstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("cannot write image: " + path);
    file.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
    if (!file) throw std::runtime_error("cannot write image: " + path);
}
}  // namespace atlas
