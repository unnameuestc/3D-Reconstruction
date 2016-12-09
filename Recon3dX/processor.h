#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <QObject>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <string>
#include <iostream>

using namespace std;
using namespace pcl;

class Processor : public QObject
{
    Q_OBJECT
public:
    explicit Processor(QObject *parent = 0);

signals:

public slots:
    void readParams(QString fileName);
    void setImages(cv::Mat& left, cv::Mat& right);
    void readImagesFromFile(QString leftFileName, QString rightFileName);
    void doRectify();
    void doMatch();
    void doReconstruct();

    void savePCL(QString fileNamePrefix);
    void showPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
    void showPCL(QString fileName);
    void showPCL();

    void createMesh(QString pcdFileName, QString plyFileName);

    void detectMark(QString markFileName, double cornerWidth, double borderWidth = 100.f);

public:
    cv::Mat M1, D1, M2, D2, R, T;
    cv::Mat Q;
    cv::Mat rgbLeft, rgbRight;
    cv::Size imgSize;
    cv::Mat rectifyLeft, rectifyRight;
    cv::Mat disparity;

    cv::Mat M3, D3, R3, T3, dR, dT;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
};

#endif // PROCESSOR_H
