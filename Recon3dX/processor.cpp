#include "processor.h"

Processor::Processor(QObject *parent) :
    QObject(parent),
    point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
    R3(3,3, cv::DataType<double>::type),
    T3(3,1, cv::DataType<double>::type)
{   
    dR = cv::Mat::zeros(3,3, cv::DataType<double>::type);
    dR.at<double>(0,0) = 1;
    dR.at<double>(1,1) = 1;
    dR.at<double>(2,2) = 1;

    dT = cv::Mat::zeros(3,1, cv::DataType<double>::type);

    std::cout << dR << std::endl << dT;
}

void Processor::readParams(QString fileName)
{
    cv::FileStorage fs(fileName.toStdString(), CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        std::cout << "Failed to open file\n";
        return;
    }

    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;
    fs["M3"] >> M3;
    fs["D3"] >> D3;

    fs["R"] >> R;
    fs["T"] >> T;

    fs.release();

    std::cout << "Processor::readParams() OK!\n";
}

void Processor::readImagesFromFile(QString leftFileName, QString rightFileName)
{
    rgbLeft = cv::imread(leftFileName.toStdString());
    rgbRight = cv::imread(rightFileName.toStdString());

    imgSize = rgbLeft.size();
}

void Processor::setImages(cv::Mat& left, cv::Mat& right)
{
    left.copyTo(rgbLeft);
    right.copyTo(rgbRight);

    imgSize = rgbLeft.size();
}

void Processor::doRectify()
{
    cv::Rect roi1, roi2;
    cv::Mat R1, P1, R2, P2;
    cv::stereoRectify( M1, D1, M2, D2, imgSize, R, T, R1, R2, P1, P2, Q, 0, -1, imgSize, &roi1, &roi2 );

    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(M1, D1, R1, P1, imgSize, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, imgSize, CV_16SC2, map21, map22);

    cv::remap(rgbLeft, rectifyLeft, map11, map12, cv::INTER_LINEAR);
    cv::remap(rgbRight, rectifyRight, map21, map22, cv::INTER_LINEAR);

    //cvtColor(rectifyLeft, rectifyLeft, CV_BGR2GRAY);
    //cvtColor(rectifyRight, rectifyRight, CV_BGR2GRAY);

    std::cout << "Q:" << Q << std::endl;
}

/*欢哥使用的是Sgbm算法，用于彩色图像匹配，运行时间bm<sgbm<gc*/
void Processor::doMatch()
{
    //rectifyLeft = cv::imread("rectifyLeft.bmp");
    //rectifyRight = cv::imread("rectifyRight.bmp");
    //disparity = cv::imread("disparity2.pgm");
    //return;

    int SADWindowSize = 0;  //需要为奇数

    int numberOfDisparities = 0;    //需要为16的倍数
    numberOfDisparities = ((imgSize.width/8) + 15) & -16;

    cv::StereoSGBM  sgbm;

    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    int cn = rgbLeft.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 100;
    sgbm.speckleRange = 32;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = true;

    cv::Mat disp;
    sgbm(rectifyLeft, rectifyRight, disp);

    disp.convertTo(disparity, CV_8U, 255/(numberOfDisparities*16.));

    imwrite("rectifyLeft.bmp", rectifyLeft);
    imwrite("rectifyRight.bmp", rectifyRight);
    imwrite("disparity.pgm", disparity);
}

void Processor::doReconstruct()
{

    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0,3);
    Q13 = Q.at<double>(1,3);
    Q23 = Q.at<double>(2,3);
    Q32 = Q.at<double>(3,2);
    Q33 = Q.at<double>(3,3);

//    uchar d = 101;

//    double pw = -1.0 * static_cast<double>(d) * Q32 + Q33;
//    double pz = Q23;
//    pz = pz/pw;
//    std::cout << "pz:" << pz << std::endl;

//    return;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZRGB>);

    double px, py, pz;
    uchar pr, pg, pb;

    for (int i = 0; i < rectifyLeft.rows; i++)
    {
      uchar* rgb_ptr = rectifyLeft.ptr<uchar>(i);
      uchar* disp_ptr = disparity.ptr<uchar>(i);

      for (int j = 0; j < rectifyLeft.cols; j++)
      {
        //Get 3D coordinates
        uchar d = disp_ptr[j];
        if ( d == 0 ) continue; //Discard bad pixels

        double pw = -1.0 * static_cast<double>(d) * Q32 + Q33;
        px = static_cast<double>(j) + Q03;
        py = static_cast<double>(i) + Q13;
        pz = Q23;

        px = px/pw;
        py = py/pw;
        pz = pz/pw;

        /*
        //dR * P + dT
        Mat matP = Mat(3,1, cv::DataType<double>::type);
        matP.at<double>(0,0) = px;
        matP.at<double>(1,0) = py;
        matP.at<double>(2,0) = pz;

        Mat matP2 = dR * matP + dT;

        px = matP2.at<double>(0,0);
        py = matP2.at<double>(1,0);
        pz = matP2.at<double>(2,0);
        */

        //Get RGB info
        pb = rgb_ptr[3*j];
        pg = rgb_ptr[3*j+1];
        pr = rgb_ptr[3*j+2];

        int gray = (pr + pg + pb)/3.0;
        if(gray < 80) continue;

        //Insert info into point cloud structure
        pcl::PointXYZRGB point;
        point.x = px;
        point.y = py;
        point.z = pz;
        uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
        point.rgb = *reinterpret_cast<float*>(&rgb);

        pcl->points.push_back (point);
      }
    }

    pcl->width = (int) pcl->points.size();
    pcl->height = 1;

    pcl::io::savePLYFile("1.ply", *pcl);


    Eigen::Matrix4f trans;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            trans(i, j) = dR.at<double>(i, j);
        }

        trans(i, 3) = dT.at<double>(i, 0);
    }

    std::cout << trans << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectifyPcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*pcl, *rectifyPcl, trans);

    int cnt = rectifyPcl->points.size();
    for(int i = 0; i < cnt; i++)
    {
       point_cloud_ptr->points.push_back(rectifyPcl->points[i]);
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    std::cout << "point count:" << point_cloud_ptr->width << std::endl;

    pcl->clear();
    rectifyPcl->clear();
}

void Processor::savePCL(QString fileNamePrefix)
{
    QString pcdFileName = fileNamePrefix + ".ply";
    QString posFileName = fileNamePrefix + ".yml";

    //pcl::io::savePCDFileASCII(pcdFileName.toStdString(), *point_cloud_ptr);
    pcl::io::savePLYFile(pcdFileName.toStdString(), *point_cloud_ptr);

    cv::FileStorage fs(posFileName.toStdString(), CV_STORAGE_WRITE);
    if(!fs.isOpened())
    {
        std::cout << "Failed to open file\n";
        return;
    }

    fs << "R" << dR;
    fs << "T" << dT;

    fs.release();
}

//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer
boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem( 1.0 );
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 4000, 0, -1, 0);

  return (viewer);
}

void Processor::showPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl)
{
    //Create visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = createVisualizer( pcl );

    //Main loop
    while ( !viewer->wasStopped())
    {
      viewer->spinOnce(100);
      //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void Processor::showPCL(QString fileName)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile<pcl::PointXYZRGB>(fileName.toStdString(), *pcl);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile<pcl::PointXYZRGB>("E:\dev\recon3dx\bin\pcl\pointcloud11.ply", *pcl2);

    for(int i = 0; i < pcl2->points.size(); i++)
    {
        pcl->points.push_back(pcl2->points[i]);
    }

    pcl->width = (int) pcl->points.size();
    pcl->height = 1;

    pcl2->clear();

    showPCL(pcl);
}

void Processor::showPCL()
{
    showPCL(point_cloud_ptr);
}

void Processor::detectMark(QString markFileName, double cornerWidth, double borderWidth)
{
    cv::Mat gray_image = cv::imread(markFileName.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
    std::vector<cv::Point2f> corners;
    cv::Size chessboard_size(3, 3);

    if(gray_image.empty())
    {
        std::cout << "image read error" << std::endl;
        return;
    }

    if(!cv::findChessboardCorners(gray_image, chessboard_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE /*+ cv::CALIB_CB_FILTER_QUADS*/))
    {
        std::cout << "not find chessboard" << std::endl;
        return;
    }

    if(corners.size())
    {
        cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        //求R,T
        //generate world object coordinates
        std::vector<cv::Point3f> world_corners;
        for (int h=0; h<chessboard_size.height; h++)
        {
            for (int w=0; w<chessboard_size.width; w++)
            {
                //暂时这么定世界坐标
                world_corners.push_back(cv::Point3f(cornerWidth * h, cornerWidth * w, 0.f));
            }
        }

        cv::Mat vR(3,1, cv::DataType<double>::type);
        cv::Mat tmpR(3,3, cv::DataType<double>::type);
        cv::Mat tmpT(3,1, cv::DataType<double>::type);

        cv::solvePnP(world_corners, corners, M3, D3, vR, tmpT);
        cv::Rodrigues(vR, tmpR);   //rotation vector ->  rotation matrix

        static bool isInitDRT = false;
        if(isInitDRT)
        {
            dR = tmpR * R3.inv();
            dT = tmpT - dR * T3;
        }
        else
        {
            R3 = tmpR;
            T3 = tmpT;

            isInitDRT = true;
        }

    //    std::cout << "dR:\n" << dR << std::endl;
    //    std::cout << "dT:\n" << dT << std::endl;
    }
}

void Processor::createMesh(QString pcdFileName, QString plyFileName)
{
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

    if(io::loadPCDFile<PointXYZ> (pcdFileName.toStdString(), *cloud) == -1){
        cout << "ERROR: couldn't find file" << endl;
        return;
    } else {
        cout << "loaded" << endl;

        NormalEstimationOMP<PointXYZ, Normal> ne;
        search::KdTree<PointXYZ>::Ptr tree1 (new search::KdTree<PointXYZ>);
        tree1->setInputCloud (cloud);
        ne.setInputCloud (cloud);
        ne.setSearchMethod (tree1);
        ne.setKSearch (20);
        PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
        ne.compute (*normals);

        // Concatenate the XYZ and normal fields*
        PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
        concatenateFields(*cloud, *normals, *cloud_with_normals);

        // Create search tree*
        search::KdTree<PointNormal>::Ptr tree (new search::KdTree<PointNormal>);
        tree->setInputCloud (cloud_with_normals);

        cout << "begin marching cubes reconstruction" << endl;

        /*
        MarchingCubesRBF<PointNormal> mc;
        PolygonMesh::Ptr triangles(new PolygonMesh);
        mc.setInputCloud (cloud_with_normals);
        mc.setSearchMethod (tree);
        mc.reconstruct (*triangles);

        cout << triangles->polygons.size() << " triangles created" << endl;
        */

        MarchingCubesHoppe<PointNormal> mc;
        PolygonMesh triangles;
        mc.setInputCloud (cloud_with_normals);
        //mc.setIsoLevel (0.001f);
        //mc.setPercentageExtendGrid (0.5);
        mc.setGridResolution (100,100,100);
        mc.setSearchMethod (tree);
        mc.reconstruct (triangles);
        cout << triangles.polygons.size() << " triangles created" << endl;

        pcl::visualization::PCLVisualizer viewer ("3D Viewer");
        viewer.setBackgroundColor (0.2, 0.2, 0.2);
        viewer.addPolygonMesh(triangles);
        viewer.spin();

        io::savePLYFile(plyFileName.toStdString(), triangles);
    }
}
