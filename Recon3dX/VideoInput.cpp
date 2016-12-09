/*
Copyright (c) 2012, Daniel Moreno and Gabriel Taubin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Brown University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DANIEL MORENO AND GABRIEL TAUBIN BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "VideoInput.hpp"

#ifdef _MSC_VER
#   include <Dshow.h>
#endif

#ifdef Q_OS_MAC
#   include "VideoInput_QTkit.hpp"
#endif

#include <QApplication>
#include <QMetaType>
#include <QTime>

#include <iostream>

VideoInput::VideoInput(QObject  * parent): 
    QThread(parent),
    _camera_index(-1),
    _video_capture(NULL),
    _init(false),
    _stop(false)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
}

VideoInput::~VideoInput()
{
    stop_camera();
}

void VideoInput::run()
{
    _init = false;
    _stop = false;

    bool success = start_camera();

    _init = true;
    
    if (!success)
    {
        return;
    }

    int error = 0;
    int max_error = 10;
    int warmup = 10000;
    QTime timer;
    timer.start();
    while(_video_capture && !_stop && error<max_error)
    {
        IplImage * frame = cvQueryFrame(_video_capture);
        if (frame)
        {   //ok
            error = 0;
            emit new_image(cv::Mat(frame));
        }
        else
        {   //error
            if (timer.elapsed()>warmup) {error++;}
        }
    }

    //clean up
    stop_camera();
    QApplication::processEvents();
}

bool VideoInput::start_camera(void)
{
    if (_video_capture)
    {
        return false;
    }

    int index = _camera_index;
    if (index<0)
    {
        return false;
    }

#ifdef _MSC_VER
    int CLASS = CV_CAP_DSHOW;
#endif
#ifdef Q_OS_MAC
    int CLASS = CV_CAP_QT;
#endif

    _video_capture = cvCaptureFromCAM(CLASS + index);
    if(!_video_capture)
    {
        std::cerr << "camera open failed, index=" << index << std::endl;
        return false;
    }

#ifdef _MSC_VER
    // HACK: if removed, OpenCV falls back to 640x480 by default
    //ask for the maximum resolution 
    // (it works with the current implementation of OpencV)
    cvSetCaptureProperty(_video_capture, CV_CAP_PROP_FRAME_WIDTH, 800);
    cvSetCaptureProperty(_video_capture, CV_CAP_PROP_FRAME_HEIGHT, 600);
#endif

    return true;
}

void VideoInput::stop_camera(void)
{
    if (_video_capture)
    {
#ifndef Q_OS_MAC //HACK: do not close on mac because it hangs the application
        cvReleaseCapture(&_video_capture);
#endif
        _video_capture = NULL;
    }
}

void VideoInput::waitForStart(void)
{
    while (isRunning() && !_init)
    {
        QApplication::processEvents();
    }
}

QStringList VideoInput::list_devices(void)
{
    QStringList list;
    bool silent = true;
#ifdef _MSC_VER
    list = list_devices_dshow(silent);
#endif
#ifdef Q_OS_MAC
    list = list_devices_quicktime(silent);
#endif

    return list;
}

/*
   listDevices_dshow() is based on videoInput library by Theodore Watson:
   http://muonics.net/school/spring05/videoInput/

   Below is the original copyright
*/

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

//////////////////////////////////////////////////////////
//Written by Theodore Watson - theo.watson@gmail.com    //
//Do whatever you want with this code but if you find   //
//a bug or make an improvement I would love to know!    //
//                                                      //
//Warning This code is experimental                     //
//use at your own risk :)                               //
//////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/*                     Shoutouts

Thanks to:

           Dillip Kumar Kara for crossbar code.
           Zachary Lieberman for getting me into this stuff
           and for being so generous with time and code.
           The guys at Potion Design for helping me with VC++
           Josh Fisher for being a serious C++ nerd :)
           Golan Levin for helping me debug the strangest
           and slowest bug in the world!

           And all the people using this library who send in
           bugs, suggestions and improvements who keep me working on
           the next version - yeah thanks a lot ;)

*/
/////////////////////////////////////////////////////////

QStringList VideoInput::list_devices_dshow(bool silent)
{
    if(!silent)printf("\nVIDEOINPUT SPY MODE!\n\n");

    QStringList list;

#ifdef _MSC_VER
    ICreateDevEnum *pDevEnum = NULL;//vs提供的一个枚举器的类别过滤器,如视频捕捉设备或音频捕捉设备
    IEnumMoniker *pEnum = NULL;
    int deviceCounter = 0;
    
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL,
        CLSCTX_INPROC_SERVER, IID_ICreateDevEnum,
        reinterpret_cast<void**>(&pDevEnum));

    if (SUCCEEDED(hr))
    {
        // Create an enumerator for the video capture category.
        hr = pDevEnum->CreateClassEnumerator(
            CLSID_VideoInputDeviceCategory,
            &pEnum, 0);

       if(hr == S_OK){

             if(!silent)printf("SETUP: Looking For Capture Devices\n");
            IMoniker *pMoniker = NULL;

            while (pEnum->Next(1, &pMoniker, NULL) == S_OK){

                IPropertyBag *pPropBag;
                hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag,
                    (void**)(&pPropBag));

                if (FAILED(hr)){
                    pMoniker->Release();
                    continue;  // Skip this one, maybe the next one will work.
                }


                 // Find the description or friendly name.
                VARIANT varName;
                VariantInit(&varName);
                hr = pPropBag->Read(L"Description", &varName, 0);

                if (FAILED(hr)) hr = pPropBag->Read(L"FriendlyName", &varName, 0);

                if (SUCCEEDED(hr)){

                    hr = pPropBag->Read(L"FriendlyName", &varName, 0);

                    char deviceName[255] = {0};

                    int count = 0;
                    int maxLen = sizeof(deviceName)/sizeof(deviceName[0]) - 2;
                    while( varName.bstrVal[count] != 0x00 && count < maxLen) {
                        deviceName[count] = (char)varName.bstrVal[count];
                        count++;
                    }
                    deviceName[count] = 0;
                    list.append(deviceName);

                    if(!silent)printf("SETUP: %i) %s \n",deviceCounter, deviceName);
                }

                pPropBag->Release();
                pPropBag = NULL;

                pMoniker->Release();
                pMoniker = NULL;

                deviceCounter++;
            }

            pDevEnum->Release();
            pDevEnum = NULL;

            pEnum->Release();
            pEnum = NULL;
        }

         if(!silent)printf("SETUP: %i Device(s) found\n\n", deviceCounter);
    }
#endif //_MSC_VER

    return list;
}

QStringList VideoInput::list_devices_quicktime(bool silent)
{
    if(!silent) std::cerr << "\n[list_devices_quicktime]\n\n";

    QStringList list;

#ifdef Q_OS_MAC
    list = list_devices_qtkit(silent);
#endif //Q_OS_MAC

    return list;
}
