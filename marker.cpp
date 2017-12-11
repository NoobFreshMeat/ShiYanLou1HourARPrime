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

void drawQuad(Mat image, vector<Point2f> points, Scalar color)
{
    line(image, points[0], points[1], color);
    line(image, points[1], points[2], color);
    line(image, points[2], points[3], color);
    line(image, points[4], points[4], color);
}

void clockwise(vector<Point2f>& square)
{
    Point2f v1 = square[1] - square[0];
    Point2f v2 = square[2] - square[0];

    double o = (v1.x* v2.y) - (v1.y *v2.x);    // cross product
    
    if (o < 0.0)  // in opengl, axis Y is opposite, so o<0.0 means CCW
    {
        std::swap(square[1], square[3]);  // swap point 1 and 3 to change to CW
    }
}

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
        
        Mat grayImage;
        cvtColor(image, grayImage, CV_RGB2GRAY); // get gray image

        Mat blurredImage;
        blur(grayImage, blurredImage, Size(5, 5)); // Fuzzy processing
   
        Mat threshImage;  
        // distinguish backgurond and object
        threshold(blurredImage, threshImage, 128.0, 255.0, THRESH_OTSU); //OTSU
        
        vector<vector<Point> > contours;
        //CV_RETR_LIST: DETECT ALL contours, but do not set up layer relationship
        //CV_VHAIN_APPROX_NONE: save all point in contours to contour vector.
        findContours(threshImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        // Point2f: float ponit 2D
        vector<vector<Point2f> > squares;
        for (int i = 0; i < contours.size(); i++)
        {
            vector<Point> contour = contours[i];
            vector<Point> approx;
            //algorith: Douglas-Peucker 
            //Des: use polygon to fit original contour
            //arcLength: calculate perimeter of contour, parameter 2 as true means 
            //closed contur 
            approxPolyDP(contour, approx, arcLength(Mat(contour), true)*0.02, 
                true);
            // contourArea: calculate area of approx
            //isContourConvex: test the input contour is convex or not
            // approx.size() == 4 : find quadrilateral contour
            if ( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 
                && isContourConvex(Mat(approx)))
            {
                vector<Point2f> square;

                for (int i = 0; i < 4; ++i)
                {
                    square.push_back(Point2f(approx[i].x, approx[i].y));
                }
                squares.push_back(square);
            }
        }
        
        vector<Point2f> square = squares[0];
        drawQuad(image, square, green);
        
        clockwise(square); //change to CW

        Mat marker;
        vector<Point2f> marker_square;
        
        // marker_width = 200
        // get a new viewing plane
        marker_square.push_back(Point(0,0));
        marker_square.push_back(Point(marker_width-1, 0));
        marker_square.push_back(Point(marker_width-1, marker_width-1));
        marker_square.push_back(Point(0, marker_width-1));

        // get perspective tansform matrix
        Mat transform = getPerspectiveTransform(square, marker_square);
        // do perspective transform
        warpPerspective(grayImage, marker, transform, Size(marker_width,
            marker_width));

        threshold(marker, marker, 125, 255, THRESH_BINARY|THRESH_OTSU);      
        
        vector<Point> direction_point = {{50, 50}, {150, 50}, {150, 150}, {50, 150}};
        int direction;

        for (int i = 0; i < 4; ++i)
        {
            Point p = direction_point[i];
            if (countNonZero(marker(Rect(p.x -25, p.y - 25, marker_width /4,
                marker_width / 4))) > 20)
            {
                direction = i;
                break;
            }
        }
       
        for (int i = 0; i < direction; ++i)
        {
            rotate(square.begin(), square.begin() + 1, square.end());
        }
 
        circle(image, square[0], 5, red);
        
        // read camera parameters
        FileStorage fs("calibrate/out_camera_data.xml", FileStorage::READ);
        Mat intrinsics, distortion;

        // extract inner parameters matrix (focal, optical center)
        fs["Camera_Matrix"] >> intrinsics;
        // extract distortion factor
        fs["Distortion_Coefficients"] >> distortion;

        vector<Point3f> objectPoints;  // 3d float point
        objectPoints.push_back(Point3f(-1, 1, 0));
        objectPoints.push_back(Point3f(1, 1, 0));
        objectPoints.push_back(Point3f(1, -1, 0)); 
        objectPoints.push_back(Point3f(-1, -1, 0));

        Mat objectPointsMat(objectPoints);

        Mat rvec;
        Mat tvec;

        solvePnP(objectPointsMat, square, intrinsics, distortion, rvec, tvec);

        cout << "rvec: " << rvec << endl;
        cout << "tvec: " << tvec << endl;

        vector<Point3f> line3dx = {{0, 0, 0}, {1, 0, 0}};
        vector<Point3f> line3dy = {{0, 0, 0}, {0, 1, 0}};
        vector<Point3f> line3dz = {{0, 0, 0}, {0, 0, 1}};

        vector<Point2f> line2dx;
        vector<Point2f> line2dy;
        vector<Point2f> line2dz;
        projectPoints(line3dx, rvec, tvec, intrinsics, distortion, line2dx);
        projectPoints(line3dy, rvec, tvec, intrinsics, distortion, line2dy);
        projectPoints(line3dz, rvec, tvec, intrinsics, distortion, line2dz);
       
        line(image, line2dx[0], line2dx[1], red);
        line(image, line2dy[0], line2dy[1], blue);
        line(image, line2dz[0], line2dz[1], green);

        cv::imshow("image", image); //create window to display image
                                        //1st parameter as window name
        cv::waitKey(100); // delay 100ms
    }
    return 0;
}

