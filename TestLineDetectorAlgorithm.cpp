/*IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.

By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install,
copy or use the software.


License Agreement
For Open Source Computer Vision Library

Copyright (C) 2011-2012, Lilian Zhang, all rights reserved.
Copyright (C) 2013, Manuele Tamburrano, Stefano Fabri, all rights reserved.
Copyright (C) 2016-2017, Chenxi Liu, all rights reserved.
Third party copyrights are property of their respective owners.

To extract edge and lines, this library implements the EDLines Algorithm and the Edge Drawing detector:
http://www.sciencedirect.com/science/article/pii/S0167865511001772
http://www.sciencedirect.com/science/article/pii/S1047320312000831

All original dependencies except Opencv have been removed and the code has been optimized for Opencv 2.4.x
PairWiseLineMatching has not been touched and it still needs original dependencies, because the aim of this
porting was to match descriptors with Opencv matchers.
http://www.sciencedirect.com/science/article/pii/S1047320313000874
http://www.mip.informatik.uni-kiel.de/tiki-index.php?page=Lilian+Zhang

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* The name of the copyright holders may not be used to endorse or promote products
derived from this software without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall the Intel Corporation or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


#include <math.h>
#include <time.h>
#include <fstream>

#include "Defines.h"
#include "Structs.h"
#include "TestLineDetectorAlgorithm.h"
#include "lsd.h"

using namespace cv;
using namespace std; 


int main()
{

  //load first image from file
	std::string imageName1("1.bmp");
	cv::Mat leftImage;
    leftImage = cv::imread(imageName1, cv::IMREAD_GRAYSCALE);   // Read the file
    if(! leftImage.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

  //load second image from file
	std::string imageName2("2.bmp");
	
    cv::Mat rightImage;
    rightImage = cv::imread(imageName2, cv::IMREAD_GRAYSCALE);   // Read the file
    if(! rightImage.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

	srand((unsigned)time(0));
	int lowest=100, highest=255;
	int range=(highest-lowest)+1;
	unsigned int r, g, b; //the color of lines

	//initial variables
	cv::Mat leftColorImage(leftImage.size(), CV_8UC3);
	cv::Mat rightColorImage(rightImage.size(), CV_8UC3);
	
    cv::cvtColor(leftImage, leftColorImage, cv::COLOR_GRAY2RGB);
    cv::cvtColor(rightImage, rightColorImage, cv::COLOR_GRAY2RGB);

	//extract lines, compute their descriptors and match lines
	LineDescriptor lineDesc;

	ScaleLineSet   linesInLeft;
	ScaleLineSet   linesInRight;
	std::vector<unsigned int> matchResult;

    image_int8u_p _leftImage = bytemat_to_int8u_image(leftImage);
    image_int8u_p _rightImage = bytemat_to_int8u_image(rightImage);


    double t = (double)cv::getTickCount();

// line detector
//****************************************************************************************************//

    //lsd detector
    int n_out = 0;
    float *lsd_ret = lsd_scale(&n_out, _leftImage->data, _leftImage->xsize, _leftImage->ysize, 0.2f, 0.2f);
    free(lsd_ret);


    //EDlines detector
    lineDesc.Run(0.2f, 0.5f, _leftImage, linesInLeft);
    lineDesc.Run(1.f, 1.f, _rightImage, linesInRight);


//****************************************************************************************************//
    t = ((double)cv::getTickCount() - t) / (double)cv::getTickFrequency();
    std::cout << "cost time: " << 1000 * t << "ms" << std::endl;

    if (_leftImage)free_image_int8u(_leftImage);
    if (_rightImage)free_image_int8u(_rightImage);
	

	//draw  extracted lines into images
	cv::Point startPoint;
	cv::Point endPoint;
	cv::Point point;

	for(unsigned int i=0; i<linesInLeft.size(); i++){
		r = lowest+int(rand()%range);
		g = lowest+int(rand()%range);
		b = lowest+int(rand()%range);
		startPoint = cv::Point(int(linesInLeft[i][0].startPointX),int(linesInLeft[i][0].startPointY));
		endPoint   = cv::Point(int(linesInLeft[i][0].endPointX),  int(linesInLeft[i][0].endPointY));
		cv::line( leftColorImage,startPoint,endPoint,cv::Scalar(r,g,b));

	}
	for(unsigned int i=0; i<linesInRight.size(); i++){
		r = lowest+int(rand()%range);
		g = lowest+int(rand()%range);
		b = lowest+int(rand()%range);
		startPoint = cv::Point(int(linesInRight[i][0].startPointX),int(linesInRight[i][0].startPointY));
		endPoint   = cv::Point(int(linesInRight[i][0].endPointX),  int(linesInRight[i][0].endPointY));
		cv::line( rightColorImage,startPoint,endPoint,cv::Scalar(r,g,b));

	}
    cv::imwrite("LinesInImage1.bmp", leftColorImage);
    cv::imwrite("LinesInImage2.bmp", rightColorImage);

    //end
    getchar();
    return 0;
}



void mcv_EDLines_detector(cv::Mat src)
{
    int lowest = 100, highest = 255;
    int range = (highest - lowest) + 1;
    unsigned int r, g, b; //the color of lines

    //initial variables
    cv::Mat leftColorImage(src.size(), CV_8UC3);

    cv::cvtColor(src, leftColorImage, cv::COLOR_GRAY2RGB);

    LineDescriptor lineDesc;

    ScaleLineSet   linesInLeft;
    std::vector<unsigned int> matchResult;

    image_int8u_p _src = bytemat_to_int8u_image(src);

    double t = (double)cv::getTickCount();
    
    lineDesc.Run(0.8f, 0.9f, _src, linesInLeft);

    t = ((double)cv::getTickCount() - t) / (double)cv::getTickFrequency();
    std::cout << "cost time: " << 1000 * t << "ms" << std::endl;

    src = int8u_image_to_int8umat(_src);
    free_image_int8u(_src);


    cv::Point startPoint;
    cv::Point endPoint;
    cv::Point point;

    for (unsigned int i = 0; i < linesInLeft.size(); i++)
    {
        r = lowest + int(rand() % range);
        g = lowest + int(rand() % range);
        b = lowest + int(rand() % range);
        startPoint = cv::Point(int(linesInLeft[i][0].startPointX), int(linesInLeft[i][0].startPointY));
        endPoint = cv::Point(int(linesInLeft[i][0].endPointX), int(linesInLeft[i][0].endPointY));
        cv::line(leftColorImage, startPoint, endPoint, cv::Scalar(r, g, b));
    }

    cv::imwrite("LinesInImage1.bmp", src);

}