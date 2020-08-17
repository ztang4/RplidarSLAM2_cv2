#include "opencv_lidar.h"
#define PI 3.141592653


int thresh = 50, N = 11;
const char* wndname = "Square Detection Demo";



LidarImage::LidarImage(void)
{
}

LidarImage::~LidarImage(void)
{

}


//将扫面的原始数据转化为实际距离和角度，这个可以参考ultra_simple
void LidarImage::scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency)
{
	
    scan_data.clear();
    for (int pos = 0; pos < (int)count; ++pos) {
        scanDot dot;
        if (!buffer[pos].distance_q2) continue;

        dot.quality = (buffer[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        dot.angle = (buffer[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
        dot.dist = buffer[pos].distance_q2 / 4.0f;
        scan_data.push_back(dot);
    }

    scan_speed = frequency;

	
}



/** @function thresh_callback */
void LidarImage::boundingBoxImg(Mat src){


    Mat src_gray;
	int boundryThresh = 100;
	int max_thresh = 255;
	RNG rng(12345);


	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;


	cvtColor(src, src_gray, CV_BGR2GRAY);
	/// Detect edges using Threshold
	threshold(src_gray, threshold_output, boundryThresh, 255, THRESH_BINARY);
	/// Find contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	if (contours.size() == 0) {
		cout << "not find contours" << endl;;
			return;
	
	}

	/// Find the rotated rectangles and ellipses for each contour
	vector<RotatedRect> minRect(contours.size());

	int lastContourIndex = contours.size() - 1;

	// get minimal contour
	minRect[0] = minAreaRect(Mat(contours[lastContourIndex]));
	/*
	for (int i = 0; i < contoursizes; i++){

		minRect[i] = minAreaRect(Mat(contours[i]));
	}
	*/

	/// Draw contours + rotated rects + ellipses
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++)
	{
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		// contour
		drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());

		// rotated rectangle
		Point2f rect_points[4]; minRect[i].points(rect_points);
		for (int j = 0; j < 4; j++)
			line(drawing, rect_points[j], rect_points[(j + 1) % 4], color, 3, 8);
	}


	cout << "Angle: " << minRect[0].angle << endl;
	cout << "Width: " << minRect[0].size.width << endl;
	cout << "height: " << minRect[0].size.height << endl;
	cout << "center point: x:" << minRect[0].center.x <<
		" y:" << minRect[0].center.y << endl;


	// print box angle, width, height
	char s[350];
	sprintf_s(s, "Agngle: %0.2f, Width: %0.2f, Height: %0.2f", minRect[0].angle, 
								minRect[0].size.width, minRect[0].size.height);

	putText(drawing, s, Point(50, 50),1,1,Scalar(0,255,0));//显示扫描的有效点个数和扫描频率

	sprintf_s(s, "Center X: %0.1f, Y: %0.1f", minRect[0].center.x,
											  minRect[0].center.y);

	putText(drawing, s, Point(minRect[0].center.x, minRect[0].center.y), 1, 1, Scalar(0, 255, 0));//显示扫描的有效点个数和扫描频率

	/// Show in a window
	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	imshow("Contours", drawing);
}

//将扫描点映射到画布上
void LidarImage::draw(IplImage* RadarImage)
{
    //RadarImage = cvCreateImage(cvSize(RadarImageWdith,RadarImageHeight),IPL_DEPTH_8U,1);  
    cvZero(RadarImage);
	cvSet(RadarImage, cvScalarAll(255), 0);
    //在中心加上一个圆心  
		//此处为在界面上显示文字使用了CvFont
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 0, 1, 8);

	cvCircle(RadarImage, cvPoint(RadarImageWdith / 2, RadarImageHeight / 2), 30, CV_RGB(0, 255, 255), 1, 8, 0);

    int x0, y0,x1,y1;
    double theta, rho;
    unsigned char * pPixel = 0;
    int halfWidth = RadarImageWdith / 2;
    int halfHeight = RadarImageHeight / 2;



    for (int i = 1; i < scan_data.size(); i++)
    {
        scanDot dot;

		dot = scan_data[i-1];
		theta = dot.angle*PI / 180;
		rho = dot.dist * LIDAR_RATIO;

		x0 = (int)(rho*sin(theta) ) + halfWidth;
		y0 = (int)(-rho * cos(theta)) + halfHeight;

		// get second point
		dot = scan_data[i];
        theta = dot.angle*PI / 180;
        rho = dot.dist* LIDAR_RATIO;

		//cout << "rho: " << rho << endl;

		x1 = (int)(rho*sin(theta)) + halfWidth;
		y1 = (int)(-rho * cos(theta)) + halfHeight;

		cvLine(RadarImage, cvPoint(x0, y0), cvPoint(x1, y1), CV_RGB(0, 0, 0), 2, 8, 0);
		
	    }

	Mat image;
	image = cvarrToMat(RadarImage);
	
	this->boundingBoxImg(image);
	
    char s[350];
    sprintf_s(s, "Value Count: %d, Scan Speed: %0.2f", scan_data.size(), scan_speed);
    cvPutText(RadarImage, s, cvPoint(50, 50), &font, cvScalar(255, 0, 0));//显示扫描的有效点个数和扫描频率
}