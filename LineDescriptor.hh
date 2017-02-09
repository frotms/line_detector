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

#ifndef LINEDESCRIPTOR_HH_
#define LINEDESCRIPTOR_HH_


#include "EDLineDetector.hh"
#include "LineStructure.hh"
#include <math.h>

#include <map>

#ifndef X_PI
#define X_PI 3.141592654F//3.14159265358979323846
#endif


struct ScaledLine{
  unsigned int scaledCount;//the scaled which this line is detected
  unsigned int lineIDInScaled;//the line ID in that scaled image
  unsigned int lineIDInScaleLineVec;//the line ID in Scale line vector
  float lineLength; //the length of line in original image scale
};


/* This class is used to generate the line descriptors from multi-scale images  */
class LineDescriptor
{
public:
	LineDescriptor();
	LineDescriptor(unsigned int numOfBand, unsigned int widthOfBand);
	~LineDescriptor();

public:
   
    /** EDLines Interface
	@param scaleX      When different from 1.0, EDLines will vertically scale the
                   input image by 'scaleX' factor by Gaussian filtering,
                   before detecting line segments.
                   Example: if scaleX=0.8, the input image will be subsampled
                   to 80% of its width, before the EDlines detector
                   is applied.

	@param scaleY      When different from 1.0, EDLines will vertically scale the
                   input image by 'scaleY' factor by Gaussian filtering,
                   before detecting line segments.
                   Example: if scaleY=0.8, the input image will be subsampled
                   to 80% of its width, before the EDlines detector
                   is applied.

	@param image       Pointer to image_int8u_s.

	@param keyLines    Pointer to an ScaleLineSet where EDLines will store the
                   lines information.

	@return
	*/
    int Run(float scaleX, float scaleY, 
        image_int8u_p image, ScaleLineSet & keyLines);
    
private:

    /*This function is used to detect lines from multi-scale images.*/
    int ScaledKeyLines(image_int8u_p image, ScaleLineSet &keyLines);

    /*This function is used to get numbers of pixels in a line from image.*/
    int GetLinePixelsNums(float startX, float startY, float endX, float endY);

    /*This function is used to get information of lines from downsampled image.*/
    void InverseGaussianSamplerLines(float_pixel_t gs_scale, ScaleLineSet &keyLines);


private:

	/*For each scaled of image, we define an EDLineDetector, because we can get gradient images (dxImg, dyImg, gImg)
	 *from the EDLineDetector class without extra computation cost. Another reason is that, if we use
	 *a single EDLineDetector to detect lines in different scaled of images, then we need to allocate and release
	 *memory for gradient images (dxImg, dyImg, gImg) repeatedly for their varying size*/
	std::vector<EDLineDetector*> edLineVec_;

	//int ksize_; //the size of Gaussian kernel: ksize X ksize, default value is 5.

	unsigned int  numOfBand_;//the number of band used to compute line descriptor
	unsigned int  widthOfBand_;//the width of band;
	std::vector<float> gaussCoefL_;//the local gaussian coefficient apply to the orthogonal line direction within each band;
	std::vector<float> gaussCoefG_;//the global gaussian coefficient apply to each Row within line support region



};

#endif /* LINEDESCRIPTOR_HH_ */
