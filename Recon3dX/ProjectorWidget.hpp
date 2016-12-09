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

#ifndef __PROJECTORWIDGET_HPP__
#define __PROJECTORWIDGET_HPP__

#include <QWidget>

class ProjectorWidget : public QWidget
{
    Q_OBJECT

public:
    ProjectorWidget(QWidget * parent = 0, Qt::WindowFlags flags = 0);
    ~ProjectorWidget();

    inline void set_screen(int screen) {_screen = screen;}

    void reset(void);
    void start(void);
    void stop(void);

    void clear() { _pixmap = QPixmap(); update(); }
    const QPixmap * pixmap() const {return &_pixmap;}
    void setPixmap(const QPixmap & pixmap) { _pixmap = pixmap; update(); }

    inline bool is_updated(void) const {return _updated;}
    inline void clear_updated(void) {_updated = false;}

    inline void setStripWidth(int n){ stripWidth = n;}
    inline void setBrightness(int n){ brightness = n;}

protected:
    virtual void paintEvent(QPaintEvent *);
    void make_pattern(int cols, int rows);

private:
    int _screen;
    QPixmap _pixmap;
    volatile bool _updated;

    int stripWidth;
    int brightness;
};


#endif  /* __PROJECTORWIDGET_HPP__ */
