#include "atlas/image.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
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

namespace {
std::string escapeXml(const std::string& text) {
    std::string out;
    for (char c : text) {
        if (c == '&') out += "&amp;";
        else if (c == '<') out += "&lt;";
        else if (c == '>') out += "&gt;";
        else if (c == '"') out += "&quot;";
        else out += c;
    }
    return out;
}

// Largest 1, 2 or 5 times a power of ten that is at most a quarter of span.
double scaleBarLength(double span) {
    const double target = span / 4;
    const double power = std::pow(10.0, std::floor(std::log10(target)));
    for (double m : {5.0, 2.0, 1.0})
        if (m * power <= target) return m * power;
    return power;
}

std::string number(double v) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.1f", v);
    return buffer;
}
}  // namespace

std::string trajectorySvg(const std::vector<PlotSeries>& series, const std::string& title) {
    // Plot axes: u = -y (rightward on screen), v = x (upward on screen).
    const double inf = std::numeric_limits<double>::infinity();
    double umin = inf, umax = -inf, vmin = inf, vmax = -inf;
    std::size_t count = 0;
    for (const PlotSeries& s : series)
        for (const Point2D& p : s.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) throw std::invalid_argument("trajectorySvg: nonfinite point");
            umin = std::min(umin, -p.y);
            umax = std::max(umax, -p.y);
            vmin = std::min(vmin, p.x);
            vmax = std::max(vmax, p.x);
            ++count;
        }
    if (!count) throw std::invalid_argument("trajectorySvg: no points");
    const double du = std::max(umax - umin, 1.0);
    const double dv = std::max(vmax - vmin, 1.0);
    double scale = 720 / du;  // pixels per meter, the same on both axes
    if (dv * scale > 900) scale = 900 / dv;
    const double margin = 30;
    const double top = 44 + 22.0 * static_cast<double>(series.size());  // title and legend
    const double bottom = 40;                                            // scale bar
    const double width = std::max(du * scale + 2 * margin, 420.0);
    const double height = top + dv * scale + margin + bottom;
    const auto sx = [&](double y) { return margin + (-y - umin) * scale; };
    const auto sy = [&](double x) { return top + (vmax - x) * scale; };

    std::string svg = "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" + number(width) + "\" height=\"" +
                      number(height) + "\" viewBox=\"0 0 " + number(width) + " " + number(height) +
                      "\" font-family=\"Helvetica, Arial, sans-serif\" font-size=\"13\">\n";
    svg += "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";
    svg += "<text x=\"" + number(margin) + "\" y=\"24\" font-size=\"15\" font-weight=\"bold\" fill=\"#111111\">" +
           escapeXml(title) + "</text>\n";
    for (std::size_t i = 0; i < series.size(); ++i) {
        const double y = 46 + 22.0 * static_cast<double>(i);
        svg += "<line x1=\"" + number(margin) + "\" y1=\"" + number(y - 4) + "\" x2=\"" + number(margin + 24) +
               "\" y2=\"" + number(y - 4) + "\" stroke=\"" + escapeXml(series[i].color) + "\" stroke-width=\"3\"/>\n";
        svg += "<text x=\"" + number(margin + 32) + "\" y=\"" + number(y) + "\" fill=\"#111111\">" +
               escapeXml(series[i].name) + "</text>\n";
    }
    for (const PlotSeries& s : series) {
        if (s.points.empty()) continue;
        svg += "<polyline fill=\"none\" stroke=\"" + escapeXml(s.color) +
               "\" stroke-width=\"2\" stroke-linejoin=\"round\" points=\"";
        for (const Point2D& p : s.points) svg += number(sx(p.y)) + "," + number(sy(p.x)) + " ";
        svg += "\"/>\n";
    }
    for (const PlotSeries& s : series) {
        if (s.points.empty()) continue;
        svg += "<circle cx=\"" + number(sx(s.points.front().y)) + "\" cy=\"" + number(sy(s.points.front().x)) +
               "\" r=\"6\" fill=\"none\" stroke=\"#111111\" stroke-width=\"2\"/>\n";
        svg += "<text x=\"" + number(sx(s.points.front().y) + 10) + "\" y=\"" + number(sy(s.points.front().x) + 4) +
               "\" fill=\"#111111\">start</text>\n";
        break;
    }
    const double bar = scaleBarLength(std::max(du, dv));
    const double bar_y = height - 18;
    char bar_label[32];
    std::snprintf(bar_label, sizeof bar_label, "%g", bar);
    svg += "<line x1=\"" + number(margin) + "\" y1=\"" + number(bar_y) + "\" x2=\"" + number(margin + bar * scale) +
           "\" y2=\"" + number(bar_y) + "\" stroke=\"#111111\" stroke-width=\"2\"/>\n";
    svg += "<text x=\"" + number(margin + bar * scale + 8) + "\" y=\"" + number(bar_y + 4) + "\" fill=\"#111111\">" +
           std::string(bar_label) + " m (equal axes; forward of the first scan is up)</text>\n";
    svg += "</svg>\n";
    return svg;
}
}  // namespace atlas
