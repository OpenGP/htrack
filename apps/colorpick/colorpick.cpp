// based on http://pastebin.com/EHz2a0YP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include "Util.h"

using namespace cv;
using namespace std;

//-----------------------------------------------------------------//

cv::Mat image, sel, src, hsv;
Scalar cMin(179,255,255), cMax(0,0,0);
bool selecting = false;
char w1[20] = "Color picker";
char w2[20] = "Picked color";
char w3[20] = "Segments";

//-----------------------------------------------------------------//

void filter()
{
	// range based color classification

	cv::Mat bw;
	cv::inRange(hsv, cMin, cMax, bw);
	cv::cvtColor(bw, sel, CV_GRAY2BGR);
	cv::imshow(w2, sel);

	// segmentation with opening and contours

	cv::RNG rng(12345);
	cv::Mat output;

	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(4,4));
	cv::morphologyEx(bw, output, MORPH_OPEN, kernel);
	cv::findContours(output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	cv::Mat drawing = cv::Mat::zeros(output.size(), CV_8UC3);

	for(int i = 0; i < contours.size(); ++i)
	{
		Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
		drawContours(drawing, contours, i, color, CV_FILLED);

		Moments m = moments(contours[i]);
		Point2f mp = Point2f(m.m10/m.m00, m.m01/m.m00);
		double a = contourArea(contours[i]);

		if(a > 2000)
		{
			Scalar color = Scalar(255,255,255);
			drawContours(drawing, contours, i, color);
			line(drawing, mp + Point2f(-4,-4), mp + Point2f( 4, 4), color);
			line(drawing, mp + Point2f( 4,-4), mp + Point2f(-4, 4), color);
		}
	}

	imshow(w3, drawing);
}

//-----------------------------------------------------------------//

static void onMouse(int event, int x, int y, int f, void *)
{
    std::cout << "## UPDATED" << std::endl;
    
	image = src.clone();
	Vec3b pix = hsv.at<Vec3b>(y,x);

	int H = pix.val[0];
	int S = pix.val[1];
	int V = pix.val[2];

	//--

	if(event == EVENT_LBUTTONDOWN)
		selecting = true;
	else if(event == EVENT_LBUTTONUP)
		selecting = false;
	if(selecting)
	{
		if(H < cMin[0])
			cMin[0] = H;
		if(S < cMin[1])
			cMin[1] = S;
		if(V < cMin[2])
			cMin[2] = V;

		if(H > cMax[0])
			cMax[0] = H;
		if(S > cMax[1])
			cMax[1] = S;
		if(V > cMax[2])
			cMax[2] = V;

		filter();
	}

	//--

	Scalar bgr = cvc(hsv2rgb(cvc(cv::Scalar(H,S,V))), true);
	rectangle(image, Point(3,5), Point(13,15), bgr, -1);

	stringstream s;
	s << "HSV=(" << H << ", " << S << ", " << V << ")";
	putText(image, s.str().c_str(), Point(17,15), FONT_HERSHEY_SIMPLEX, .6, Scalar(0,0,0), 2);
	putText(image, s.str().c_str(), Point(17,15), FONT_HERSHEY_SIMPLEX, .6, Scalar(255,255,255));

	//--

	bgr = cvc(hsv2rgb(cvc(cMin)), true);
	rectangle(image, Point(3,25), Point(13,35), bgr, -1);

	s.str("");
	s.clear();
	s << "hsv_min=" << cMin[0] << ", " << cMin[1] << ", " << cMin[2];
    std::cout << s.str() << std::endl;
	putText(image, s.str().c_str(), Point(17,35), FONT_HERSHEY_SIMPLEX, .6, Scalar(0,0,0), 2);
	putText(image, s.str().c_str(), Point(17,35), FONT_HERSHEY_SIMPLEX, .6, Scalar(255,255,255));

	//--

	bgr = cvc(hsv2rgb(cvc(cMax)), true);
	rectangle(image, Point(3,45), Point(13,55), bgr, -1);

	s.str("");
	s.clear();
	s << "hsv_max=" << cMax[0] << ", " << cMax[1] << ", " << cMax[2];
	std::cout << s.str() << std::endl;
    putText(image, s.str().c_str(), Point(17,55), FONT_HERSHEY_SIMPLEX, .6, Scalar(0,0,0), 2);
	putText(image, s.str().c_str(), Point(17,55), FONT_HERSHEY_SIMPLEX, .6, Scalar(255,255,255));

	//--

	s.str("");
	s.clear();
	s << "range:";
	putText(image, s.str().c_str(), Point(255,25), FONT_HERSHEY_SIMPLEX, .6, Scalar(0,0,0), 2);
	putText(image, s.str().c_str(), Point(255,25), FONT_HERSHEY_SIMPLEX, .6, Scalar(255,255,255));

	double n = 57.0f;

	for(double i = 0; i < n; ++i)
	{
		// note: red hues wrap around, 1D linear interpolation
		//       doesn't include all possible HSV combinations

		double w = i / n;
		Scalar col = (1.0-w)*cMin + w*cMax;

		bgr = cvc(hsv2rgb(cvc(col)), true);
		rectangle(image, Point(255+i,35), Point(256+i,45), bgr, -1);
	}

	//--

	imshow(w1, image);
	imshow(w2, sel);
}

//-----------------------------------------------------------------//

int main(int argc, char **argv)
{
	// TODO use camera images for input

	cout << "Color picker" << endl
	     << "------------" << endl
	     << "Pass image file as command line argument" << endl
	     << "Click to select colors" << endl
	     << "Press SPACE to reset min/max" << endl
	     << "Press Q or ESC to quit" << endl << endl;

	if(argc < 2)
	{
		cout << "No file specified" << endl;
		return 0;
	}

	src = imread(argv[1]);

	if(!src.data)
	{
		cout << "Could not read file" << endl;
		return 0;
	}

	namedWindow(w1, WINDOW_AUTOSIZE);
	cvtColor(src, hsv, CV_BGR2HSV);
	filter();

	imshow(w1, src);
	imshow(w2, sel);

	setMouseCallback(w1, onMouse, 0);

	for(;;)
	{
		char key = waitKey(20);

		if(key == 0x71 || key == 0x1b)
			break;
		else if(key == 0x20)
		{
			cMin = Scalar(179,255,255);
			cMax = Scalar(0,0,0);
			filter();
		}
	}

	return 0;
}

