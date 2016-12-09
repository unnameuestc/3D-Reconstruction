#include "CameraObj.hpp"

#include <QApplication>

CameraObj::CameraObj(QObject *parent) :
    QObject(parent)
{
}

CameraObj::~CameraObj()
{
    stop_camera();
}

void CameraObj::setUIControls(QComboBox *combox, QLabel *label, ImageLabel *imagelabel)
{
    this->camera_combo = combox;
    this->camera_resolution_label = label;
    this->camera_image = imagelabel;

    //connect(camera_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(on_camera_combo_currentIndexChanged(int)));
    //用activated信号，即使选择一样都会触发
    connect(camera_combo, SIGNAL(activated(int)), this, SLOT(on_camera_combo_currentIndexChanged(int)));
}



int CameraObj::update_camera_combo(void)
{
    //disable signals
    camera_combo->blockSignals(true);

    //save current value
    QString current = camera_combo->currentText();

    //update combo
    camera_combo->clear();
    camera_combo->addItems(_video_input.list_devices());

    //set current value
    int index = camera_combo->findText(current);
    camera_combo->setCurrentIndex((index < 0 ? 0 : index));

    //enable signals
    camera_combo->blockSignals(false);

    return camera_combo->count();
}

bool CameraObj::start_camera(void)
{
    int index = camera_combo->currentIndex();
    if (_video_input.get_camera_index()==index)
    {
        return true;
    }

    //busy cursor
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    QApplication::processEvents();

    stop_camera();

    _video_input.set_camera_index(index);
    _video_input.start();
    _video_input.waitForStart();

    if (!_video_input.isRunning())
    {   //error
        //TODO: display error --------
        camera_resolution_label->clear();
    }

    //connect display signal now that we know it worked
    connect(&_video_input, SIGNAL(new_image(cv::Mat)), this, SLOT(_on_new_camera_image(cv::Mat)), Qt::QueuedConnection);

    //restore regular cursor
    QApplication::restoreOverrideCursor();
    QApplication::processEvents();

    return true;
}

void CameraObj::stop_camera(void)
{
    //disconnect display signal first
    disconnect(&_video_input, SIGNAL(new_image(cv::Mat)), this, SLOT(_on_new_camera_image(cv::Mat)));

    //clean up
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor)); //busy cursor
    QApplication::processEvents(); //process pending signals
    camera_image->clear();
    camera_resolution_label->clear();

    //stop the thread
    if (_video_input.isRunning())
    {
        _video_input.stop();
        _video_input.wait();
    }

    //restore regular cursor
    QApplication::restoreOverrideCursor();
    QApplication::processEvents();
}

void CameraObj::_on_new_camera_image(cv::Mat image)
{
    camera_image->setImage(image);
    camera_resolution_label->setText(QString("[%1x%2]").arg(image.cols).arg(image.rows));
}

void CameraObj::on_camera_combo_currentIndexChanged(int index)
{
    camera_resolution_label->clear();
    start_camera();
}

void CameraObj::saveImage(QString fileName)
{
    //cv::imwrite(fileName.toStdString(), camera_image->getImage());

    camera_image->lockImg();
    (io_util::qImage(camera_image->getImage())).save(fileName);
    camera_image->unlockImg();
}

