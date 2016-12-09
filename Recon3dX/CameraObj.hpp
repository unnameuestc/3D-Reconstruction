#ifndef __CAMERAOBJ_H__
#define __CAMERAOBJ_H__

#include <QObject>
#include <QComboBox>
#include <QLabel>

#include "ImageLabel.hpp"
#include "VideoInput.hpp"
#include "io_util.hpp"

class CameraObj : public QObject
{
    Q_OBJECT

public:
    explicit CameraObj(QObject *parent = 0);
    ~CameraObj();

    int update_camera_combo(void);
    void init_camera_combo(void);//by zhy;list all options

    bool start_camera(void);
    void stop_camera(void);
    static void wait(int msecs);

    void setUIControls(QComboBox *combox, QLabel *label, ImageLabel *imagelabel);
    void saveImage(QString fileName);

signals:

public slots:
    void _on_new_camera_image(cv::Mat image);
    void on_camera_combo_currentIndexChanged(int index);

 private:
    QComboBox *camera_combo;
    QLabel *camera_resolution_label;
    ImageLabel *camera_image;

    VideoInput _video_input;
};

#endif // __CAMERAOBJ_H__
