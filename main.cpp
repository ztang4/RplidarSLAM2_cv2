#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "opencv_lidar.h"

#include <iostream>  
#include <cmath>  
#include "io.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        }
        else {
            return true;
        }

    }
    else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 256000;
    u_result     op_result;

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

    //creat opencv image
    IplImage* RadarImage = cvCreateImage(cvSize(RadarImageWdith, RadarImageHeight), IPL_DEPTH_8U, 3);

    LidarImage lidarImage;

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }


    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

	drv->startMotor();
    // start scan...
	RplidarScanMode* outUsedScanMode;
    drv->startScan(true,true);
	
	drv->setMotorPWM(500);
    // fetech result and print it out...
    char key;
    cvNamedWindow("Radar", 1);

    while (1) {
		rplidar_response_measurement_node_t nodes[8192];

        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            float frequency = 0;
            //drv->getFrequency(*outUsedScanMode, count, frequency);
            lidarImage.scanData(nodes, count, frequency);
            lidarImage.draw(RadarImage);

            cvShowImage("Radar", RadarImage);
            key = cvWaitKey(30);
            if (key == 27)//esc退出  
            {
                break;
            }       
        }
    }

    cvReleaseImage(&RadarImage);
    cvDestroyWindow("Radar");
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}