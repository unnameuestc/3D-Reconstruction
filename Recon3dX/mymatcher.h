#ifndef MYMATCHER_H
#define MYMATCHER_H

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <iostream>

using namespace std;

class MyMatcher
{
public:
    MyMatcher();

    static cv::Mat startMatch(cv::Mat leftImg, cv::Mat rightImg);
    static void test();
};

#endif // MYMATCHER_H
