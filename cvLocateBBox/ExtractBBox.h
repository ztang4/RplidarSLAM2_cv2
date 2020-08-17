#pragma once
#include <iostream>
#include <cmath>

#include <opencv2\opencv.hpp>
#include <highgui.h> 


#ifndef EXTRACTBBOX_H
#define EXTRACTBBOX_H


using namespace std;
using namespace cv;
//����opencv��ʾ�����С��rplidar��Զ������16m�����Է�����㣬����ȡ6�ı���
const int RadarImageWdith = 1000;
const int RadarImageHeight = 1000;
const int thresh = 50, N = 11;
//const char* wndname = "Square Detection Demo";

class ExtractBBox
{
public:
	ExtractBBox(void);
	~ExtractBBox(void);
	void drawLidarPoints(IplImage* RadarImage);
	void extractBoundingBox(Mat src);

};





#endif