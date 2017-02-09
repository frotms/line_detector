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
#ifndef LINESTRUCTURE_HH_
#define LINESTRUCTURE_HH_

#include <vector>

struct SingleLineInfo
{
	/*endPoints, the coordinate origin is the top-left corner of the original image.
	 *startPointX = sPointInScaledX * (factor)^scaledCount;	*/
	float startPointX;
	float startPointY;
	float endPointX;
	float endPointY;
	//endPoints, the coordinate origin is the top-left corner of the scaled image.
	float sPointInScaledX;
	float sPointInScaledY;
	float ePointInScaledX;
	float ePointInScaledY;
	//direction of a line, the angle between positive line direction (dark side is in the left) and positive X axis.
	float direction;
	//the summation of gradient magnitudes of pixels on scaled lines
	float salience;
	//the length of line
	float lineLength;
	//number of pixels
	unsigned int numOfPixels;
	//the scaled which this line is detected
	unsigned int scaledCount;
	//the decriptor of line
	std::vector<float> descriptor;
};

// Specifies a vector of lines.
typedef std::vector<SingleLineInfo> LineSet;

typedef std::vector<LineSet> ScaleLineSet;//each element in ScaleLineSet is a vector of lines which corresponds the same line detected in different scaled images.



#endif /* LINESTRUCTURE_HH_ */
