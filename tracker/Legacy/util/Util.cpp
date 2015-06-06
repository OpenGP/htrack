#include "Util.h"

#include <iostream>
#include <string>
#include <iterator>

#ifdef OPENMP
    #include <omp.h>
#endif 

#include <Eigen/Core>

//===========================================================================//

std::string cat(const std::vector<std::string> &v)
{
	std::ostringstream s;
	std::copy(v.begin(), v.end(), std::ostream_iterator<std::string>(s));
	return s.str();
}

//===========================================================================//

bool contains_string(const std::string &a, const std::string &b)
{
    std::size_t found = a.find(b);
    return found!=std::string::npos;
}

//===========================================================================//

ColorHSV rgb2hsv(const ColorRGB &in)
{
	ColorHSV    out;
	double      min, max, delta;

	min = in.r < in.g ? in.r : in.g;
	min = min  < in.b ? min  : in.b;

	max = in.r > in.g ? in.r : in.g;
	max = max  > in.b ? max  : in.b;

	out.v = max;                                // v
	delta = max - min;

	if( max > 0.0 )
	{
		out.s = (delta / max);                  // s
	}
	else
	{
		// r = g = b = 0                        // s = 0, v is undefined
		out.s = 0.0;
		out.h = NAN;                            // its now undefined
		return out;
	}

	if( in.r >= max )
	{
		out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
	}
	else if( in.g >= max )
	{
		out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
	}
	else
	{
		out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan
	}

	out.h *= 60.0;                              // degrees

	if( out.h < 0.0 )
	{
		out.h += 360.0;
	}

	return out;
}

//===========================================================================//

ColorRGB hsv2rgb(const ColorHSV &in)
{
	double      hh, p, q, t, ff;
	long        i;
	ColorRGB    out;

	if(in.s <= 0.0)
	{
		out.r = in.v;
		out.g = in.v;
		out.b = in.v;
		return out;
	}

	hh = in.h;

	if(hh >= 360.0)
	{
		hh = 0.0;
	}

	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;

	p = in.v * (1.0 - in.s);
	q = in.v * (1.0 - (in.s * ff));
	t = in.v * (1.0 - (in.s * (1.0 - ff)));

	switch(i)
	{
		case 0:
			out.r = in.v;
			out.g = t;
			out.b = p;
			break;

		case 1:
			out.r = q;
			out.g = in.v;
			out.b = p;
			break;

		case 2:
			out.r = p;
			out.g = in.v;
			out.b = t;
			break;

		case 3:
			out.r = p;
			out.g = q;
			out.b = in.v;
			break;

		case 4:
			out.r = t;
			out.g = p;
			out.b = in.v;
			break;

		case 5:
		default:
			out.r = in.v;
			out.g = p;
			out.b = q;
			break;
	}

	return out;
}

//===========================================================================//

cv::Scalar cvc(const ColorRGB &c, bool bgr)
{
	if(bgr)
		return cv::Scalar(c.b * 255, c.g * 255, c.r * 255);
	else
		return cv::Scalar(c.r * 255, c.g * 255, c.b * 255);
}

//===========================================================================//

cv::Scalar cvc(const ColorHSV &c)
{
	return cv::Scalar(c.h / 360 * 179, c.s * 255, c.v * 255);
}

//===========================================================================//

ColorRGB cvc(const cv::Scalar &c, bool bgr)
{
	if(bgr)
		return ColorRGB(c[2] / 255.0f, c[1] / 255.0f, c[0] / 255.0f);
	else
		return ColorRGB(c[0] / 255.0f, c[1] / 255.0f, c[2] / 255.0f);
}

//===========================================================================//

ColorHSV cvc(const cv::Scalar &c)
{
	return ColorHSV(c[0] / 179.0f * 360.0f, c[1] / 255.0f, c[2] / 255.0f);
}

