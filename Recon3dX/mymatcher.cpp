#include "mymatcher.h"

MyMatcher::MyMatcher()
{
}

cv::Mat MyMatcher::startMatch(cv::Mat leftImg, cv::Mat rightImg)
{
    int win_w = 11;
    int win_h = 3;

    cv::Mat disImg(leftImg.size(), cv::DataType<uchar>::type);

    return disImg;
}


void MyMatcher::test()
{
    cv::Mat leftImg = cv::imread("rectifyLeft.bmp");
    cv::Mat rightImg = cv::imread("rectifyRight.bmp");

    cv::Mat dis = startMatch(leftImg, rightImg);
    cv::imshow("dis", dis);

    cv::waitKey();
}
