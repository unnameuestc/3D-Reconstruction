#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "processor.h"
#include "CameraObj.hpp"
#include "ProjectorWidget.hpp"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_btn_recon_clicked();
    void on_btn_project_clicked();
    void on_btn_choose_param_clicked();
    void on_btn_mark_clicked();
    void on_btn_save_img_clicked();
    void on_btn_show_left_clicked();
    void on_btn_show_right_clicked();
    void on_btn_show_rectify_clicked();
    void on_btn_show_disparity_clicked();
    void on_btn_save_res_clicked();
    void on_btn_show_pcl_clicked();

    void on_btn_grid_clicked();

private:
    Ui::MainWindow *ui;

    Processor engine;

    CameraObj cam_mark, cam_left, cam_right;
    ProjectorWidget pw;

    int imgPairIndex;
    QString leftFileName, rightFileName;

    bool isSaveImg;//判定是否进行过saveimage
};

#endif // MAINWINDOW_H
