#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    isSaveImg = false;
    //setWindowFlags(Qt::FramelessWindowHint);

    cam_mark.setUIControls(ui->cb_cam_mark, ui->lb_cam_mark, ui->lb_img_mark);
    cam_left.setUIControls(ui->cb_cam_left, ui->lb_cam_left, ui->lb_img_left);
    cam_right.setUIControls(ui->cb_cam_right, ui->lb_cam_right, ui->lb_img_right);

    cam_mark.update_camera_combo();
    cam_left.update_camera_combo();
    cam_right.update_camera_combo();

    imgPairIndex = 0;
    leftFileName = "img/left" + QString::number(imgPairIndex) + ".bmp";
    rightFileName = "img/right" + QString::number(imgPairIndex) + ".bmp";

    //leftFileName = "img/left.bmp";
    //rightFileName = "img/right.bmp";
}

MainWindow::~MainWindow()
{
    pw.stop();

    delete ui;
}

void MainWindow::on_btn_recon_clicked()
{
    if(isSaveImg)
        engine.readImagesFromFile(leftFileName, rightFileName);
    else{
        imgPairIndex++;
        leftFileName = "img/left" + QString::number(imgPairIndex) + ".bmp";
        rightFileName = "img/right" + QString::number(imgPairIndex) + ".bmp";
        engine.readImagesFromFile(leftFileName, rightFileName);
    }

    int64 t = cv::getTickCount();

    engine.doRectify();
    engine.doMatch();
    engine.doReconstruct();

    t = cv::getTickCount() - t;
    int costTime = t*1000/cv::getTickFrequency();

    QMessageBox::information(NULL, "重建完成", "重建完成 --------耗时  " + QString::number(costTime) + "  ms--------");
}

void MainWindow::on_btn_project_clicked()
{
    if(!pw.isHidden())
    {
        pw.stop();
    }
    else
    {
        pw.setStripWidth(ui->le_strip_w->text().toInt());
        pw.setBrightness(ui->le_strip_brightness->text().toInt());
        pw.start();
    }
}

void MainWindow::on_btn_choose_param_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(
                this,
                "选择相机参数文件",
                QDir::currentPath(),
                "参数文件 (*.yml *.xml);;All files(*.*)");

    if (!fileName.isNull())
    {
        ui->le_param_path->setText(fileName);
        engine.readParams(fileName);
    }
    else
    {
        ui->le_param_path->setText("");
    }
}

void MainWindow::on_btn_mark_clicked()
{
    cam_mark.saveImage("marker.bmp");

    engine.detectMark("marker.bmp", 24.5);
}


void MainWindow::on_btn_save_img_clicked()
{
    imgPairIndex++;

    leftFileName = "img/left" + QString::number(imgPairIndex) + ".bmp";
    rightFileName = "img/right" + QString::number(imgPairIndex) + ".bmp";

    //leftFileName = "img/left.bmp";
    //rightFileName = "img/right.bmp";

    cam_left.saveImage(leftFileName);
    cam_right.saveImage(rightFileName);

    isSaveImg = true;
}

void MainWindow::on_btn_show_left_clicked()
{
    cv::namedWindow( "left", CV_WINDOW_AUTOSIZE );
    cv::imshow("left", engine.rgbLeft);
}

void MainWindow::on_btn_show_right_clicked()
{
    cv::namedWindow( "right", CV_WINDOW_AUTOSIZE );
    cv::imshow("right", engine.rgbRight);
}

void MainWindow::on_btn_show_rectify_clicked()
{
    //engine.doRectify();//test git
    cv::namedWindow( "rectify_left", CV_WINDOW_AUTOSIZE );
    cv::imshow("rectify_left", engine.rectifyLeft);

    cv::namedWindow( "rectify_right", CV_WINDOW_AUTOSIZE );
    cv::imshow("rectify_right", engine.rectifyRight);
}

void MainWindow::on_btn_show_disparity_clicked()
{
    cv::namedWindow( "disparity", CV_WINDOW_AUTOSIZE );
    cv::imshow("disparity", engine.disparity);
}

void MainWindow::on_btn_save_res_clicked()
{
    QString pclFileName = "pcl/pointcloud" + QString::number(imgPairIndex);
    //QString pclFileName = "pcl/pointcloud";
    engine.savePCL(pclFileName);
}

void MainWindow::on_btn_show_pcl_clicked()
{
    if(!engine.point_cloud_ptr->empty())
    {
        engine.showPCL();
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(
                this,
                "选择点云文件",
                QDir::currentPath(),
                "点云文件 (*.ply);;All files(*.*)");

    if (!fileName.isNull())
    {
        engine.showPCL(fileName);
    }
}

void MainWindow::on_btn_grid_clicked()
{
    engine.createMesh("pcl/pointcloud0.pcd", "pcl/pointcloud0.ply");
}
