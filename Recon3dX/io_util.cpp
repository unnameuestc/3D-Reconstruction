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

#include "io_util.hpp"

#include <QPainter>

#include <iostream>
#include <fstream>
#include <float.h>

#if defined(_MSC_VER) && !defined(isnan)
# include <float.h>
# define isnan _isnan
#endif


QImage io_util::qImage(const cv::Mat & image)
{
    switch (image.type())
    {
    case CV_8UC3: return io_util::qImageFromRGB(image); break;
    case CV_8UC1: return io_util::qImageFromGray(image); break;
    }
    return QImage();
}

QImage io_util::qImageFromRGB(const cv::Mat & image)
{
    if (image.type()!=CV_8UC3)
    {   //unsupported type
        return QImage();
    }

    QImage qimg(image.cols, image.rows, QImage::Format_RGB32);
    for (int h=0; h<image.rows; h++)
    {
        const cv::Vec3b * row = image.ptr<cv::Vec3b>(h);
        unsigned * qrow = reinterpret_cast<unsigned *>(qimg.scanLine(h));
        for (register int w=0; w<image.cols; w++)
        {
            const cv::Vec3b & vec = row[w];
            qrow[w] = qRgb(vec[2], vec[1], vec[0]);
        }
    }
    return qimg;
}

QImage io_util::qImageFromGray(const cv::Mat & image)
{
    if (image.type()!=CV_8UC1)
    {   //unsupported type
        return QImage();
    }

    QImage qimg(image.cols, image.rows, QImage::Format_RGB32);
    for (int h=0; h<image.rows; h++)
    {
        const unsigned char * row = image.ptr<unsigned char>(h);
        unsigned * qrow = reinterpret_cast<unsigned *>(qimg.scanLine(h));
        for (register int w=0; w<image.cols; w++)
        {
            qrow[w] = qRgb(row[w], row[w], row[w]);
        }
    }
    return qimg;
}

bool io_util::write_pgm(const cv::Mat & image, const char * basename)
{
    if (!image.data || image.type()!=CV_32FC2 || !basename)
    {
        return false;
    }

    //open
    QString filename1 = QString("%1_col.pgm").arg(basename);
    QString filename2 = QString("%1_row.pgm").arg(basename);
    FILE * fp1 = fopen(qPrintable(filename1), "w");
    FILE * fp2 = fopen(qPrintable(filename2), "w");

    if (!fp1 || !fp2)
    {
        if (fp1) { fclose(fp1); }
        if (fp2) { fclose(fp2); }
        return false;
    }

    //write header
    /*
        P5      //binary grayscale   P2: ASCII grayscale
        24 7    //width height
        15      //maximum value
    */
    fprintf(fp1, "P2\n%d %d\n1024\n", image.cols, image.rows);
    fprintf(fp2, "P2\n%d %d\n1024\n", image.cols, image.rows);

    //write contents
    for (int h=0; h<image.rows; h++)
    {
        const cv::Vec2f * row = image.ptr<cv::Vec2f>(h);
        for (int w=0; w<image.cols; w++)
        {
            cv::Vec2f const& pattern = row[w];
            char c = (w+1<image.cols ? ' ' : '\n');
            fprintf(fp1, "%u%c", static_cast<unsigned>(pattern[0]), c);
            fprintf(fp2, "%u%c", static_cast<unsigned>(pattern[1]), c);
        }
    }

    //close
    if (fp1)
    {
        fclose(fp1);
    }
    if (fp2)
    {
        fclose(fp2);
    }

    return true;
}
