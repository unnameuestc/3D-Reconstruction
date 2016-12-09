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


#include "ProjectorWidget.hpp"

#include <QApplication>
#include <QDesktopWidget>
#include <QPainter>
#include <QScreen>

#include <iostream>
#include <assert.h>

ProjectorWidget::ProjectorWidget(QWidget * parent, Qt::WindowFlags flags) : 
    QWidget(parent, flags),
    _screen(1),
    _updated(false),
    stripWidth(5),
    brightness(255)
{
}

ProjectorWidget::~ProjectorWidget()
{
    stop();
}

void ProjectorWidget::reset(void)
{
    _updated = false;
    _pixmap = QPixmap();
}

void ProjectorWidget::start(void)
{
    stop();
    reset();

    //validate screen
    QDesktopWidget * desktop = QApplication::desktop();
    int screens =  desktop->screenCount();
    if (_screen<0 || _screen>=screens)
    {   //error, fix it
        _screen = screens - 1;
    }

    //display
    QRect screen_resolution = desktop->screenGeometry(_screen);
    move(QPoint(screen_resolution.x(), screen_resolution.y()));
    showFullScreen();
}

void ProjectorWidget::stop(void)
{
    hide();
    reset();
}

void ProjectorWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    bool updated = false;
    if (_pixmap.isNull())
    {   //update
        updated = true;
        make_pattern(width(), height());
    }

    //draw
    QRectF rect = QRectF(QPointF(0,0), QPointF(width(),height()));
    painter.drawPixmap(rect, _pixmap, rect);

    if (updated)
    {
        _updated = true;
    }
}

void ProjectorWidget::make_pattern(int cols, int rows)
{
    QImage image(cols, rows, QImage::Format_ARGB32);

    int cnt = stripWidth * 8 + 4;
    int *de = new int[cnt];

    memset(de, 0, sizeof(int) * stripWidth);
    de[stripWidth] = 1;
    memset(de + stripWidth + 1, 0, sizeof(int) * stripWidth);
    de[stripWidth * 2 + 1] = 1;
    memset(de + 2*stripWidth + 2, 0, sizeof(int) * stripWidth);
    memset(de + 3*stripWidth + 2, 1, sizeof(int) * stripWidth);
    memset(de + 4*stripWidth + 2, 0, sizeof(int) * stripWidth);
    memset(de + 5*stripWidth + 2, 1, sizeof(int) * stripWidth);
    de[stripWidth * 6 + 2] = 0;
    memset(de + 6*stripWidth + 3, 1, sizeof(int) * stripWidth);
    de[stripWidth * 7 + 3] = 0;
    memset(de + 7*stripWidth + 4, 1, sizeof(int) * stripWidth);

    /*
    int de[44] = {0,0,0,0,0,1,
                 0,0,0,0,0,1,
                 0,0,0,0,0,
                 1,1,1,1,1,
                 0,0,0,0,0,
                 1,1,1,1,1,0,
                 1,1,1,1,1,0,
                 1,1,1,1,1};
    */

    for (int h=0; h<rows; h++)
    {
        uchar * row = image.scanLine(h);
        for (int w=0; w<cols; w++)
        {
            uchar * px = row + (4*w);

            int value = 0;
            bool isBlack = false;

            isBlack = (de[w % cnt] == 0);
            value = isBlack ? 0 : brightness;

            px[0] = value; //B
            px[1] = value; //G
            px[2] = value; //R
            px[3] = 0xff;  //A
        }
    }

    _pixmap = QPixmap::fromImage(image);
}
