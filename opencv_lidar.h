#pragma once
#include <iostream>
#include <cmath>

#include <opencv2\opencv.hpp>
#include <highgui.h> 

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header


using namespace std;
using namespace cv;
//定义opencv显示界面大小，rplidar最远距离是16m，所以方便计算，画布取6的倍数
static int RadarImageWdith = 1000;
static int RadarImageHeight = 1000;
//测量点数据结构，这个可以参考应用手册
struct scanDot {
	_u8   quality;
	float angle;
	float dist;
};

#define LIDAR_RATIO   1

class LidarImage
{
public:
	LidarImage(void);
	~LidarImage(void);
	
	vector<scanDot> scan_data; //保存每扫描一周的雷达数据
	float scan_speed;
	void scanData(rplidar_response_measurement_node_t *buffer,
				  size_t count, float frequency);

	void draw(IplImage* RadarImage);

	/// Function header
	void boundingBoxImg(Mat src);

};


