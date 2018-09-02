#ifndef _IMAGE_CAPTURE_H
#define _IMAGE_CAPTURE_H

#include <ctime>
#include <cstdio>
#include <cstdint>
#include <string>
#include <iomanip>

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#include <windows.h>
#include "FlyCapture2.h"
#else
#include <unistd.h>
#include <time.h>
//#include <linux/types.h>
#include <sys/stat.h>
#include "../Chameleon_Test_Linux/include/FlyCapture2.h"
#endif


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>     
#include <opencv2/imgproc/imgproc.hpp>  

namespace FC2 = FlyCapture2;

FC2::Error get_image(FC2::Camera &cam, cv::Mat &image)
{
    FC2::Error error;
    FC2::Image rawImage, convertedImageCV;
    uint32_t image_rows, image_cols;
    unsigned char *image_data = NULL;

    // get the images from the camera
    error = cam.RetrieveBuffer(&rawImage);
    if (error != FC2::PGRERROR_OK)
    {
        return error;
    }   
    
    image_cols = rawImage.GetCols();
    image_rows = rawImage.GetRows();
    //rawImage.GetDimensions(&, &, &image_stride);    // , &pixFormat, &btFormat);
    error = rawImage.Convert(FC2::PIXEL_FORMAT_BGR, &convertedImageCV);
    //image_data = convertedImageCV.GetData();
    //image_data = rawImage.GetData();

    cv::Mat tmp_img = cv::Mat(cv::Size(image_cols, image_rows), CV_8UC3, convertedImageCV.GetData(), image_cols *3);
    image = tmp_img.clone();    // do this because the pointer goes out of scope when the function exits

    return error;
}

#endif  // _IMAGE_CAPTURE_H
