#pragma once
#include <iostream>
#include <cmath>

#include <opencv2\opencv.hpp>
#include <highgui.h> 

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header


using namespace std;
using namespace cv;
//����opencv��ʾ�����С��rplidar��Զ������16m�����Է�����㣬����ȡ6�ı���
static int RadarImageWdith = 1000;
static int RadarImageHeight = 1000;
//���������ݽṹ��������Բο�Ӧ���ֲ�
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
	
	vector<scanDot> scan_data; //����ÿɨ��һ�ܵ��״�����
	float scan_speed;
	void scanData(rplidar_response_measurement_node_t *buffer,
				  size_t count, float frequency);

	void draw(IplImage* RadarImage);

	/// Function header
	void boundingBoxImg(Mat src);

};


