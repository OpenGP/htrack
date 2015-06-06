#pragma once

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>

// concatenate strings contained in the input vector
std::string cat(const std::vector<std::string> &v);

// check whether a contains b
bool contains_string(const std::string &a, const std::string &b);

// rgb color
struct ColorRGB
{
	double r; // percent
	double g; // percent
	double b; // percent

	ColorRGB() {}

	ColorRGB(double r, double g, double b)
		: r(r), g(g), b(b) {}
};

// hsv color
struct ColorHSV
{
	double h; // angle in degrees
	double s; // percent
	double v; // percent

	ColorHSV() {}

	ColorHSV(double h, double s, double v)
		: h(h), s(s), v(v) {}
};

// color space conversions
ColorHSV rgb2hsv(const ColorRGB &in);
ColorRGB hsv2rgb(const ColorHSV &in);

// opencv color utility functions
cv::Scalar cvc(const ColorRGB &c, bool bgr);
cv::Scalar cvc(const ColorHSV &c);
ColorRGB cvc(const cv::Scalar &c, bool bgr);
ColorHSV cvc(const cv::Scalar &c);

