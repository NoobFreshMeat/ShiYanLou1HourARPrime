// marker.cpp

#include <opencv2/core/core.hpp>     // openCV core lib
#include <opencv2/imgproc/imgproc.hpp> // opencv image process lib
#include <opencv2/highgui/highgui.hpp> // opencv user interface for video play
#include <opencv2/calib3d/calib3d.hpp> // calibrate camera for posture estimate

#include <iostream>

using namespace std;
using namespace cv;

// Scalar is a 4 elements array like variable to store color channel data
// below is rgb/bgra
Scalar blue(255, 0, 0);
Scalar green(0, 255, 0);
Scalar red(0, 0, 255);

const int marker_width = 200;

int main(int argc, char** argv)
{
    Mat image;  // Mat: matrix, for storing image

    VideoCapture cap("video.mp4"); // capture video object
                                   // parameter as 0 will get camera input
    if (!cap.isOpened())
        return -1;

    while (cap.grab()) // capture next frame from video object
    {
        cap.retrieve(image); //intepret captured frame and stor into image
        cv::imshow("image", image); //create window to display image
                                    //1st parameter as window name
        cv::waitKey(100); // delay 100ms
    }
    return 0;
}

