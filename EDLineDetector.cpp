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


#include "EDLineDetector.hh"

#define Horizontal  255//if |dx|<|dy|;
#define Vertical    0//if |dy|<=|dx|;
#define UpDir       1
#define RightDir    2
#define DownDir     3
#define LeftDir     4
#define TryTime     6
#define SkipEdgePoint 2

using namespace std;
EDLineDetector::EDLineDetector()
{
	//set parameters for line segment detection

	gradienThreshold_ = 80; // ***** ORIGINAL WAS 25
	anchorThreshold_  = 2;//8
	scanIntervals_    = 2;//2
	minLineLen_       = 15;
	lineFitErrThreshold_ = 1.4f;


	InitEDLine_();
}
EDLineDetector::EDLineDetector(EDLineParam param)
{
	//set parameters for line segment detection
	gradienThreshold_ = (short)param.gradientThreshold;
	anchorThreshold_  = (unsigned char)param.anchorThreshold;
	scanIntervals_    = param.scanIntervals;
	minLineLen_       = param.minLineLen;
	lineFitErrThreshold_ = param.lineFitErrThreshold;
	InitEDLine_();
}
void EDLineDetector::InitEDLine_()
{
	bValidate_ = true;

    ATA = new_image_float(2,2);
    ATV = new_image_float(1,2);
    tempMatLineFit = new_image_float(2, 2);
    tempVecLineFit = new_image_float(1, 2);
    fitMatT = new_image_float(minLineLen_, 2);
    fitVec = new_image_float(minLineLen_, 1);
    

	for(int i=0; i<minLineLen_; i++){
        fitMatT->data[1*fitMatT->xsize + i] = 1;
	}

	pFirstPartEdgeX_ = NULL;
	pFirstPartEdgeY_ = NULL;
	pFirstPartEdgeS_ = NULL;
	pSecondPartEdgeX_ = NULL;
	pSecondPartEdgeY_ = NULL;
	pSecondPartEdgeS_ = NULL;
	pAnchorX_ = NULL;
	pAnchorY_ = NULL;


	dirImg_ = NULL;
	gImgWO_ = NULL;
    gImg_ = NULL;
    dxImg_ = NULL;
    dyImg_ = NULL;
    edgeImage_ = NULL;
}
EDLineDetector::~EDLineDetector(){
	if(pFirstPartEdgeX_!=NULL){
		delete [] pFirstPartEdgeX_;
		delete [] pFirstPartEdgeY_;
		delete [] pSecondPartEdgeX_;
		delete [] pSecondPartEdgeY_;
		delete [] pAnchorX_;
		delete [] pAnchorY_;
	}
	if(pFirstPartEdgeS_ != NULL){
		delete [] pFirstPartEdgeS_;
		delete [] pSecondPartEdgeS_;
	}

    free_image_int8u(edgeImage_);
	free_image_int8u(dirImg_);
	free_image_int16s(gImgWO_);
    free_image_int16s(gImg_);
    free_image_int16s(dxImg_);
    free_image_int16s(dyImg_);

    free_image_float(ATA);
    free_image_float(ATV);
    free_image_float(fitMatT);
    free_image_float(fitVec);
    free_image_float(tempMatLineFit);
    free_image_float(tempVecLineFit);

}


typedef enum _ORIENT_CODE
{
    ORIENT_HORIZONAL = 1,       // horizotal
    ORIENT_VERTICAL = 2,        // vertical


}ORIENT_CODE;

void sobel_edge(ORIENT_CODE oriention, image_int8u_p src, image_int16s_p dst)
{
    unsigned char *psrc = NULL;
    unsigned int nsize;
    unsigned int i, j, _center, offset_up, offset_down;
    unsigned int _tp, _td, _t;

    nsize = src->xsize * src->ysize;

    //no edge processing
    //memset(dst->data,ZERO,sizeof(short));

    psrc = src->data;
    switch (oriention)
    {
    case ORIENT_HORIZONAL:
        for (i = 1; i < src->ysize - 1; i++)
        {
            _center = i*src->xsize;
            offset_up = _center - src->xsize;
            offset_down = _center + src->xsize;
            for (j = 1; j < src->xsize - 1; j++)
            {
                _tp = offset_up + j;
                _td = offset_down + j;
                _t = _center + j;
                dst->data[_t] = ((short)psrc[_tp + 1] - (short)psrc[_tp - 1])
                    + ((short)psrc[_td + 1] - (short)psrc[_td - 1])
                    + (((short)psrc[_t + 1] - (short)psrc[_t - 1]) << 1);
            }
        }
        break;

    case  ORIENT_VERTICAL:
        for (i = 1; i < src->ysize - 1; i++)
        {
            _center = i*src->xsize;
            offset_up = _center - src->xsize;
            offset_down = _center + src->xsize;
            for (j = 1; j < src->xsize - 1; j++)
            {
                _tp = offset_up + j;
                _td = offset_down + j;
                _t = _center + j;

                dst->data[_t] = ((short)psrc[_tp - 1] + (((short)psrc[_tp]) << 1) + (short)psrc[_tp + 1])
                    - ((short)psrc[_td - 1] + (((short)psrc[_td]) << 1) + (short)psrc[_td + 1]);
            }
        }
        break;

    default:
        printf("sobel oriention is wrong!");
        break;
    }

    psrc = NULL;

}


void mcv_sobel(ORIENT_CODE oriention, image_int8u_p src, image_int16s_p dst)
{
    sobel_edge(oriention, src, dst);
}

void array_abs(image_int16s_p src, image_int16s_p dst)
{
    int nsize;
    int k;
    short *psrc = NULL, *pdst = NULL;

    nsize = src->xsize*src->ysize;

    psrc = src->data;
    pdst = dst->data;
    for (k = 0; k < nsize; k++)
    {
        *pdst = (*psrc >= ZERO) ? (*psrc) : (-*psrc);
        psrc++;
        pdst++;
    }

    psrc = NULL;
    pdst = NULL;

}


void mcv_abs(image_int16s_p src, image_int16s_p dst)
{
    array_abs(src, dst);
}


int array_add(image_float_p src1, image_float_p src2, image_float_p dst)
{
    int _code = 0;
    int k, nsize;
    float *psrc1 = NULL, *psrc2 = NULL, *pdst = NULL;

    if ((src1 == NULL) || (src2 == NULL) || (dst == NULL)
        || (src1->xsize != src2->xsize) || (src1->ysize != src2->ysize)
        || (src1->xsize != dst->xsize) || (src1->ysize != dst->ysize))
        return 1;

    nsize = src1->xsize*src1->ysize;

    psrc1 = src1->data;
    psrc2 = src2->data;
    pdst = dst->data;
    for (k = 0; k < nsize; k++)
    {
        *pdst = *psrc1 + *psrc2;
        pdst++;
        psrc1++;
        psrc2++;
    }

    psrc1 = NULL;
    psrc2 = NULL;
    pdst = NULL;

    return 0;
}

void array_add(image_int16s_p src1, image_int16s_p src2, image_int16s_p dst)
{
    short *psrc1 = NULL, *psrc2 = NULL, *pdst = NULL;
    int nsize;
    int k;

    nsize = src1->xsize*src1->ysize;

    psrc1 = src1->data;
    psrc2 = src2->data;
    pdst = dst->data;

    for (k = 0; k < nsize; k++)
    {
        *pdst = *psrc1 + *psrc2;
        psrc1++;
        psrc2++;
        pdst++;
    }

}

void mcv_add(image_int16s_p src1, image_int16s_p src2, image_int16s_p dst)
{
    array_add(src1, src2, dst);
}

void mcv_add(image_float_p src1, image_float_p src2, image_float_p dst)
{
    array_add(src1, src2, dst);
}


image_int16s_p array_threshold(short thresh, image_int16s_p src)
{
    image_int16s_p dst = NULL;
    int nsize;
    int k;
    short *psrc = NULL, *pdst = NULL;

    dst = new image_int16s_s[1];

    dst->xsize = src->xsize;
    dst->ysize = src->ysize;
    nsize = src->xsize*src->ysize;

    dst->data = new short[nsize];

    psrc = src->data;
    pdst = dst->data;
    for (k = 0; k<nsize; k++)
    {
        *pdst = (*psrc > thresh) ? (*psrc) : (ZERO);
        psrc++;
        pdst++;
    }

    psrc = NULL;
    pdst = NULL;

    return dst;

}


void array_threshold(short thresh, image_int16s_p src, image_int16s_p dst)
{
    int nsize;
    int k;
    short *psrc = NULL, *pdst = NULL;

    nsize = src->xsize*src->ysize;

    psrc = src->data;
    pdst = dst->data;
    for (k = 0; k<nsize; k++)
    {
        *pdst = (*psrc > thresh) ? (*psrc) : (ZERO);
        psrc++;
        pdst++;
    }

    psrc = NULL;
    pdst = NULL;


}


void mcv_threshold(short thresh, image_int16s_p src, image_int16s_p dst)
{
    array_threshold(thresh, src, dst);

}


void array_compare_lt(image_int16s_p src1, image_int16s_p src2, image_int8u_p dst)
{
    short *psrc1 = NULL, *psrc2 = NULL;
    unsigned char *pdst = NULL;
    int nsize, k;

    psrc1 = src1->data;
    psrc2 = src2->data;
    pdst = dst->data;

    nsize = src1->xsize*src1->ysize;

    for (k = 0; k < nsize; k++)
    {
        *pdst = (*psrc1 < *psrc2) ? (255) : (ZERO);
        pdst++;
        psrc1++;
        psrc2++;
    }

}

void mcv_compare_CMP_LT(image_int16s_p src1, image_int16s_p src2, image_int8u_p dst)
{

    array_compare_lt(src1, src2, dst);

}


image_int16s_p array_devide(int n, image_int16s_p src)
{
    image_int16s_p dst = NULL;
    int nsize;
    int k;
    float n_inv;
    short *psrc = NULL, *pdst = NULL;

    dst = new image_int16s_s[1];

    dst->xsize = src->xsize;
    dst->ysize = src->ysize;
    nsize = src->xsize*src->ysize;

    dst->data = new short[nsize];

    psrc = src->data;
    pdst = dst->data;

    n_inv = 1.f / n;
    for (k = 0; k < nsize; k++)
    {
        *pdst = (short)(n_inv * (*psrc));
        psrc++;
        pdst++;
    }

    psrc = NULL;
    pdst = NULL;

    return dst;
}

void array_devide(int n, image_int16s_p src, image_int16s_p dst)
{
    int nsize;
    int k;
    short *psrc = NULL, *pdst = NULL;

    nsize = src->xsize*src->ysize;

    psrc = src->data;
    pdst = dst->data;
    for (k = 0; k < nsize; k++)
    {
        *pdst = *psrc / n;
        psrc++;
        pdst++;
    }

    psrc = NULL;
    pdst = NULL;

}


void mcv_mat_divide(int n, image_int16s_p src, image_int16s_p dst)
{
    array_devide(n, src, dst);
}



//A*AT
int array_multiply_transpose_float(image_float_p src, image_float_p dst)
{
    int i, j;
    int m, n, l;
    float _sum;
    float *psrc = NULL, *pdst = NULL;

    if (src == NULL || dst == NULL || src->data == NULL || dst->data == NULL)
        return 1;

    if (dst->xsize != dst->ysize)
        return 1;

    //A*AT
    if (src->ysize != dst->xsize)
        return 1;

    m = src->ysize;
    n = src->xsize;
    psrc = src->data;
    pdst = dst->data;

    //b = (a * a^T)
    for (i = 0; i <= m - 1; i++)
    for (j = 0; j <= m - 1; j++)
    {
        _sum = 0.f;
        for (l = 0; l <= n - 1; l++)
            _sum += psrc[i*n + l] * psrc[j*n + l];
        pdst[i*m + j] = _sum;
    }

    psrc = NULL;
    pdst = NULL;

    return 0;
}


//A*BT
int array_multiply2_transpose_float(image_float_p src1, image_float_p src2, image_float_p dst)
{
    int i, j, k;
    float _sum;

    if (src1 == NULL || src2 == NULL || dst == NULL
        || src1->data == NULL || src2->data == NULL || dst->data == NULL)
        return 1;

    //A*BT
    if (src1->ysize != dst->ysize || src2->ysize != dst->xsize)
        return 1;

    if (src1->xsize != src2->xsize)
        return 1;

    for (i = 0; i < dst->ysize; i++)
    {
        for (j = 0; j < dst->xsize; j++)
        {
            _sum = 0.f;
            for (k = 0; k < src1->xsize; k++)
            {
                _sum += (src1->data[i*src1->xsize + k] * src2->data[j*src2->xsize + k]);
            }
            dst->data[i*dst->xsize + j] = _sum;
        }
    }

    return 0;
}


int array_multiply(image_float_p src1, image_float_p src2, image_float_p dst)
{
    int i, j, l, u;
    int m, n, k;
    float *a = NULL, *b = NULL, *c = NULL;

    if (src1 == NULL || src2 == NULL || dst == NULL
        || src1->data == NULL || src2->data == NULL || dst->data == NULL)
        return 1;

    if (src1->xsize != src2->ysize)
        return 1;

    if (dst->ysize != src1->ysize || dst->xsize != src2->xsize)
        return 1;

    m = src1->ysize;
    n = src1->xsize;
    k = src2->xsize;

    a = src1->data;
    b = src2->data;
    c = dst->data;

    for (i = 0; i <= m - 1; i++)
    for (j = 0; j <= k - 1; j++)
    {
        u = i*k + j; c[u] = 0.f;
        for (l = 0; l <= n - 1; l++)
            c[u] = c[u] + a[i*n + l] * b[l*k + j];
    }

    a = NULL;
    b = NULL;
    c = NULL;

    return 0;
}


void mcv_multiply_float(image_float_p src1, image_float_p src2, image_float_p dst)
{
    array_multiply(src1, src2, dst);
}


void mcv_multiply_transpose_float(image_float_p src, image_float_p dst)
{
    array_multiply_transpose_float(src, dst);
}



void mcv_multiply2_transpose_float(image_float_p src1, image_float_p src2, image_float_p dst)
{
    array_multiply2_transpose_float(src1, src2, dst);

}


int EDLineDetector::EdgeDrawing(image_int8u_p image, EdgeChains &edgeChains, bool smoothed)
{
    imageWidth = image->xsize;
    imageHeight = image->ysize;

	unsigned int pixelNum = imageWidth*imageHeight;
	unsigned int edgePixelArraySize = pixelNum/5;
	unsigned int maxNumOfEdge = edgePixelArraySize/20;
	//compute dx, dy images
    if ((gImg_ == NULL) || ((gImg_->xsize != (int)imageWidth) || (gImg_->ysize != (int)imageHeight))){
		if(pFirstPartEdgeX_!= NULL){
			delete [] pFirstPartEdgeX_;
			delete [] pFirstPartEdgeY_;
			delete [] pSecondPartEdgeX_;
			delete [] pSecondPartEdgeY_;
			delete [] pFirstPartEdgeS_;
			delete [] pSecondPartEdgeS_;
			delete [] pAnchorX_;
			delete [] pAnchorY_;
		}

        dxImg_ = new_image_int16s(imageWidth, imageHeight);
        dyImg_ = new_image_int16s(imageWidth, imageHeight);
		gImgWO_ = new_image_int16s(imageWidth, imageHeight);
        gImg_ = new_image_int16s(imageWidth, imageHeight);
		dirImg_ = new_image_int8u(imageWidth, imageHeight);
        edgeImage_ = new_image_int8u(imageWidth, imageHeight);
		pFirstPartEdgeX_ = new unsigned int[edgePixelArraySize];
		pFirstPartEdgeY_ = new unsigned int[edgePixelArraySize];
		pSecondPartEdgeX_ = new unsigned int[edgePixelArraySize];
		pSecondPartEdgeY_ = new unsigned int[edgePixelArraySize];
		pFirstPartEdgeS_ = new unsigned int[maxNumOfEdge];
		pSecondPartEdgeS_ = new unsigned int[maxNumOfEdge];
		pAnchorX_ = new unsigned int[edgePixelArraySize];
		pAnchorY_ = new unsigned int[edgePixelArraySize];
	}

	mcv_sobel(ORIENT_HORIZONAL, image, dxImg_);
	mcv_sobel(ORIENT_VERTICAL, image, dyImg_);
	
	//compute gradient and direction images
    image_int16s_p dxABS_m = NULL, dyABS_m = NULL;
    image_int16s_p sumDxDy = NULL;
        
    dxABS_m = new_image_int16s(dxImg_->xsize, dxImg_->ysize);
    dyABS_m = new_image_int16s(dyImg_->xsize, dyImg_->ysize);
    sumDxDy = new_image_int16s(dyImg_->xsize, dyImg_->ysize);

    mcv_abs(dxImg_, dxABS_m);
    mcv_abs(dyImg_, dyABS_m);

    mcv_add(dyABS_m, dxABS_m, sumDxDy);
	
    mcv_threshold(gradienThreshold_ + 1, sumDxDy, gImg_);

	mcv_mat_divide(4, gImg_, gImg_);        
    mcv_mat_divide(4, sumDxDy, gImgWO_);    

	mcv_compare_CMP_LT(dxABS_m, dyABS_m, dirImg_);  

    free_image_int16s(sumDxDy);
    free_image_int16s(dxABS_m);
    free_image_int16s(dyABS_m);
	
    short *pgImg = gImg_->data;

	unsigned char *pdirImg = dirImg_->data;
	
	//extract the anchors in the gradient image, store into a vector
	memset(pAnchorX_,  0, edgePixelArraySize*sizeof(unsigned int));//initialization
	memset(pAnchorY_,  0, edgePixelArraySize*sizeof(unsigned int));
	unsigned int anchorsSize = 0;
	int indexInArray;
	unsigned char gValue1, gValue2, gValue3;
	for(unsigned int w=1; w<imageWidth-1; w=w+scanIntervals_){
		for(unsigned int h=1; h<imageHeight-1; h=h+scanIntervals_){
			indexInArray = h*imageWidth+w;
			if(pdirImg[indexInArray]==Horizontal){//if the direction of pixel is horizontal, then compare with up and down
				if(pgImg[indexInArray]>=pgImg[indexInArray-imageWidth]+anchorThreshold_
						&&pgImg[indexInArray]>=pgImg[indexInArray+imageWidth]+anchorThreshold_){// (w,h) is accepted as an anchor
							pAnchorX_[anchorsSize]   = w;
							pAnchorY_[anchorsSize++] = h;
				}
			}else{
				if(pgImg[indexInArray]>=pgImg[indexInArray-1]+anchorThreshold_
						&&pgImg[indexInArray]>=pgImg[indexInArray+1]+anchorThreshold_){// (w,h) is accepted as an anchor
							pAnchorX_[anchorsSize]   = w;
							pAnchorY_[anchorsSize++] = h;
				}
			}
		}
	}
	if(anchorsSize>edgePixelArraySize){
		cout<<"anchor size is larger than its maximal size. anchorsSize="<<anchorsSize
		<<", maximal size = "<<edgePixelArraySize <<endl;
		return -1;
	}

	//link the anchors by smart routing
    memset(edgeImage_->data, ZERO, edgeImage_->xsize*edgeImage_->ysize*sizeof(unsigned char));
	unsigned char *pEdgeImg = edgeImage_->data;
	memset(pFirstPartEdgeX_,  0, edgePixelArraySize*sizeof(unsigned int));//initialization
	memset(pFirstPartEdgeY_,  0, edgePixelArraySize*sizeof(unsigned int));
	memset(pSecondPartEdgeX_, 0, edgePixelArraySize*sizeof(unsigned int));
	memset(pSecondPartEdgeY_, 0, edgePixelArraySize*sizeof(unsigned int));
	memset(pFirstPartEdgeS_,  0, maxNumOfEdge*sizeof(unsigned int));
	memset(pSecondPartEdgeS_, 0, maxNumOfEdge*sizeof(unsigned int));
	unsigned int offsetPFirst=0, offsetPSecond=0;
	unsigned int offsetPS=0;

	unsigned int x, y;
	unsigned int lastX, lastY;
	unsigned char lastDirection;//up = 1, right = 2, down = 3, left = 4;
	unsigned char shouldGoDirection;//up = 1, right = 2, down = 3, left = 4;
	int edgeLenFirst, edgeLenSecond;

    lastX = lastY = 0;

	for(unsigned int i=0; i<anchorsSize; i++){
		x = pAnchorX_[i];
		y = pAnchorY_[i];
		indexInArray = y*imageWidth+x;
		if(pEdgeImg[indexInArray]){//if anchor i is already been an edge pixel.
			continue;
		}
		/*The walk stops under 3 conditions:
		 * 1. We move out of the edge areas, i.e., the thresholded gradient value
		 *    of the current pixel is 0.
		 * 2. The current direction of the edge changes, i.e., from horizontal
		 *    to vertical or vice versa.?? (This is turned out not correct. From the online edge draw demo
		 *    we can figure out that authors don't implement this rule either because their extracted edge
		 *    chain could be a circle which means pixel directions would definitely be different
		 *    in somewhere on the chain.)
		 * 3. We encounter a previously detected edge pixel. */
		pFirstPartEdgeS_[offsetPS] = offsetPFirst;
		if(pdirImg[indexInArray]==Horizontal){//if the direction of this pixel is horizontal, then go left and right.
			//fist go right, pixel direction may be different during linking.
			lastDirection = RightDir;
			while(pgImg[indexInArray]>0&&!pEdgeImg[indexInArray]){
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pFirstPartEdgeX_[offsetPFirst] = x;
				pFirstPartEdgeY_[offsetPFirst++] = y;
				shouldGoDirection = 0;//unknown
				if(pdirImg[indexInArray]==Horizontal){//should go left or right 
					if(lastDirection == UpDir ||lastDirection == DownDir){//change the pixel direction now
						if(x>lastX){//should go right
							shouldGoDirection = RightDir;
						}else{//should go left
							shouldGoDirection = LeftDir;
						}
					} 
					lastX = x;
					lastY = y;
					if(lastDirection == RightDir||shouldGoDirection == RightDir){//go right
						if(x==imageWidth-1||y==0||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray-imageWidth+1];
                        gValue2 = (unsigned char)pgImg[indexInArray + 1];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-right
							x = x+1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-right
							x = x+1;
							y = y+1;
						}else{//straight-right
							x = x+1;
						}
						lastDirection = RightDir;
					} else if(lastDirection == LeftDir || shouldGoDirection==LeftDir){//go left
						if(x==0||y==0||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
                        gValue2 = (unsigned char)pgImg[indexInArray - 1];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-left
							x = x-1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-left
							x = x-1;
							y = y+1;
						}else{//straight-left
							x = x-1;
						}
						lastDirection = LeftDir;
					}
				}else{//should go up or down.
					if(lastDirection == RightDir || lastDirection == LeftDir){//change the pixel direction now
						if(y>lastY){//should go down
							shouldGoDirection = DownDir;
						}else{//should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if(lastDirection==DownDir || shouldGoDirection == DownDir){//go down
						if(x==0||x==imageWidth-1||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//down-right
							x = x+1;
							y = y+1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-left
							x = x-1;
							y = y+1;
						}else{//straight-down
							y = y+1;
						}
						lastDirection = DownDir;
					}else if(lastDirection==UpDir || shouldGoDirection == UpDir){//go up
						if(x==0||x==imageWidth-1||y==0){//reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
                        gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-right
							x = x+1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//up-left
							x = x-1;
							y = y-1;
						}else{//straight-up
							y = y-1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y*imageWidth+x;
			}//end while go right
			//then go left, pixel direction may be different during linking.
			x = pAnchorX_[i];
			y = pAnchorY_[i];
			indexInArray = y*imageWidth+x;
			pEdgeImg[indexInArray] = 0;//mark the anchor point be a non-edge pixel and
			lastDirection = LeftDir;
			pSecondPartEdgeS_[offsetPS] = offsetPSecond;
			while(pgImg[indexInArray]>0&&!pEdgeImg[indexInArray]){
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pSecondPartEdgeX_[offsetPSecond] = x;
				pSecondPartEdgeY_[offsetPSecond++] = y;
				shouldGoDirection = 0;//unknown
				if(pdirImg[indexInArray]==Horizontal){//should go left or right
					if(lastDirection == UpDir ||lastDirection == DownDir){//change the pixel direction now
						if(x>lastX){//should go right
							shouldGoDirection = RightDir;
						}else{//should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if(lastDirection == RightDir||shouldGoDirection == RightDir){//go right
						if(x==imageWidth-1||y==0||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray + 1];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-right
							x = x+1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-right
							x = x+1;
							y = y+1;
						}else{//straight-right
							x = x+1;
						}
						lastDirection = RightDir;
					}else	if(lastDirection == LeftDir || shouldGoDirection==LeftDir){//go left
						if(x==0||y==0||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
                        gValue2 = (unsigned char)pgImg[indexInArray - 1];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-left
							x = x-1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-left
							x = x-1;
							y = y+1;
						}else{//straight-left
							x = x-1;
						}
						lastDirection = LeftDir;
					}
				}else{//should go up or down.
					if(lastDirection == RightDir || lastDirection == LeftDir){//change the pixel direction now
						if(y>lastY){//should go down
							shouldGoDirection = DownDir;
						}else{//should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if(lastDirection==DownDir || shouldGoDirection == DownDir){//go down
						if(x==0||x==imageWidth-1||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//down-right
							x = x+1;
							y = y+1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-left
							x = x-1;
							y = y+1;
						}else{//straight-down
							y = y+1;
						}
						lastDirection = DownDir;
					}else	if(lastDirection==UpDir || shouldGoDirection == UpDir){//go up
						if(x==0||x==imageWidth-1||y==0){//reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
                        gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-right
							x = x+1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//up-left
							x = x-1;
							y = y-1;
						}else{//straight-up
							y = y-1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y*imageWidth+x;
			}//end while go left
			//end anchor is Horizontal
		}else{//the direction of this pixel is vertical, go up and down
			//fist go down, pixel direction may be different during linking.
			lastDirection = DownDir;
			while(pgImg[indexInArray]>0&&!pEdgeImg[indexInArray]){
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pFirstPartEdgeX_[offsetPFirst] = x;
				pFirstPartEdgeY_[offsetPFirst++] = y;
				shouldGoDirection = 0;//unknown
				if(pdirImg[indexInArray]==Horizontal){//should go left or right
					if(lastDirection == UpDir ||lastDirection == DownDir){//change the pixel direction now
						if(x>lastX){//should go right
							shouldGoDirection = RightDir;
						}else{//should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if(lastDirection == RightDir||shouldGoDirection == RightDir){//go right
						if(x==imageWidth-1||y==0||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray + 1];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-right
							x = x+1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-right
							x = x+1;
							y = y+1;
						}else{//straight-right
							x = x+1;
						}
						lastDirection = RightDir;
					}else	if(lastDirection == LeftDir || shouldGoDirection==LeftDir){//go left
						if(x==0||y==0||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
                        gValue2 = (unsigned char)pgImg[indexInArray - 1];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-left
							x = x-1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-left
							x = x-1;
							y = y+1;
						}else{//straight-left
							x = x-1;
						}
						lastDirection = LeftDir;
					}
				}else{//should go up or down.
					if(lastDirection == RightDir || lastDirection == LeftDir){//change the pixel direction now
						if(y>lastY){//should go down
							shouldGoDirection = DownDir;
						}else{//should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if(lastDirection==DownDir || shouldGoDirection == DownDir){//go down
						if(x==0||x==imageWidth-1||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//down-right
							x = x+1;
							y = y+1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-left
							x = x-1;
							y = y+1;
						}else{//straight-down
							y = y+1;
						}
						lastDirection = DownDir;
					}else	if(lastDirection==UpDir || shouldGoDirection == UpDir){//go up
						if(x==0||x==imageWidth-1||y==0){//reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
                        gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-right
							x = x+1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//up-left
							x = x-1;
							y = y-1;
						}else{//straight-up
							y = y-1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y*imageWidth+x;
			}//end while go down
			//then go up, pixel direction may be different during linking.
			lastDirection = UpDir;
			x = pAnchorX_[i];
			y = pAnchorY_[i];
			indexInArray = y*imageWidth+x;
			pEdgeImg[indexInArray] = 0;//mark the anchor point be a non-edge pixel and
			pSecondPartEdgeS_[offsetPS] = offsetPSecond;
			while(pgImg[indexInArray]>0&&!pEdgeImg[indexInArray]){
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pSecondPartEdgeX_[offsetPSecond] = x;
				pSecondPartEdgeY_[offsetPSecond++] = y;
				shouldGoDirection = 0;//unknown
				if(pdirImg[indexInArray]==Horizontal){//should go left or right
					if(lastDirection == UpDir ||lastDirection == DownDir){//change the pixel direction now
						if(x>lastX){//should go right
							shouldGoDirection = RightDir;
						}else{//should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if(lastDirection == RightDir||shouldGoDirection == RightDir){//go right
						if(x==imageWidth-1||y==0||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray + 1];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-right
							x = x+1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-right
							x = x+1;
							y = y+1;
						}else{//straight-right
							x = x+1;
						}
						lastDirection = RightDir;
					}else	if(lastDirection == LeftDir || shouldGoDirection==LeftDir){//go left
						if(x==0||y==0||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
                        gValue2 = (unsigned char)pgImg[indexInArray - 1];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-left
							x = x-1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-left
							x = x-1;
							y = y+1;
						}else{//straight-left
							x = x-1;
						}
						lastDirection = LeftDir;
					}
				}else{//should go up or down.
					if(lastDirection == RightDir || lastDirection == LeftDir){//change the pixel direction now
						if(y>lastY){//should go down
							shouldGoDirection = DownDir;
						}else{//should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if(lastDirection==DownDir || shouldGoDirection == DownDir){//go down
						if(x==0||x==imageWidth-1||y==imageHeight-1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
                        gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//down-right
							x = x+1;
							y = y+1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//down-left
							x = x-1;
							y = y+1;
						}else{//straight-down
							y = y+1;
						}
						lastDirection = DownDir;
					}else	if(lastDirection==UpDir || shouldGoDirection == UpDir){//go up
						if(x==0||x==imageWidth-1||y==0){//reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
                        gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
                        gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
                        gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if(gValue1>=gValue2&&gValue1>=gValue3){//up-right
							x = x+1;
							y = y-1;
						}else if(gValue3>=gValue2&&gValue3>=gValue1){//up-left
							x = x-1;
							y = y-1;
						}else{//straight-up
							y = y-1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y*imageWidth+x;
			}//end while go up
		}//end anchor is Vertical
		//only keep the edge chains whose length is larger than the minLineLen_;
		edgeLenFirst = offsetPFirst - pFirstPartEdgeS_[offsetPS];
		edgeLenSecond = offsetPSecond - pSecondPartEdgeS_[offsetPS];
		if(edgeLenFirst+edgeLenSecond<minLineLen_+1){//short edge, drop it
			offsetPFirst = pFirstPartEdgeS_[offsetPS];
			offsetPSecond = pSecondPartEdgeS_[offsetPS];
		}else{
			offsetPS++;
		}
	}
	//store the last index
	pFirstPartEdgeS_[offsetPS]  = offsetPFirst;
	pSecondPartEdgeS_[offsetPS] = offsetPSecond;
	if(offsetPS>maxNumOfEdge){
		cout<<"Edge drawing Error: The total number of edges is larger than MaxNumOfEdge, "
		"numofedge = "<<offsetPS<<", MaxNumOfEdge="<<maxNumOfEdge<<endl;
		return -1;
	}
	if(offsetPFirst>edgePixelArraySize||offsetPSecond>edgePixelArraySize){
		cout<<"Edge drawing Error: The total number of edge pixels is larger than MaxNumOfEdgePixels, "
		"numofedgePixel1 = "<<offsetPFirst<<",  numofedgePixel2 = "<<offsetPSecond <<
		", MaxNumOfEdgePixel="<<edgePixelArraySize<<endl;
		return -1;
	}

	/*now all the edge information are stored in pFirstPartEdgeX_, pFirstPartEdgeY_,
	 *pFirstPartEdgeS_,  pSecondPartEdgeX_, pSecondPartEdgeY_, pSecondPartEdgeS_;
	 *we should reorganize them into edgeChains for easily using.	*/
	int tempID;
	edgeChains.xCors.resize(offsetPFirst+offsetPSecond);
	edgeChains.yCors.resize(offsetPFirst+offsetPSecond);
	edgeChains.sId.resize(offsetPS+1);
	unsigned int *pxCors = edgeChains.xCors.data();
	unsigned int *pyCors = edgeChains.yCors.data();
	unsigned int *psId   = edgeChains.sId.data();
	offsetPFirst = 0;
	offsetPSecond= 0;
	unsigned int indexInCors = 0;
	unsigned int numOfEdges = 0;
	for(unsigned int edgeId = 0; edgeId<offsetPS; edgeId++){
		//step1, put the first and second parts edge coordinates together from edge start to edge end
		psId[numOfEdges++] = indexInCors;
		indexInArray = pFirstPartEdgeS_[edgeId];
		offsetPFirst = pFirstPartEdgeS_[edgeId+1];
		for(tempID = offsetPFirst-1; tempID>= indexInArray; tempID--){//add first part edge
			pxCors[indexInCors]   = pFirstPartEdgeX_[tempID];
			pyCors[indexInCors++] = pFirstPartEdgeY_[tempID];
		}
		indexInArray = pSecondPartEdgeS_[edgeId];
		offsetPSecond= pSecondPartEdgeS_[edgeId+1];
        for (tempID = indexInArray + 1; tempID<(int)offsetPSecond; tempID++){//add second part edge
			pxCors[indexInCors]  = pSecondPartEdgeX_[tempID];
			pyCors[indexInCors++]= pSecondPartEdgeY_[tempID];
		}
	}
	psId[numOfEdges] = indexInCors;//the end index of the last edge
	edgeChains.numOfEdges = numOfEdges;


	return 1;
}


//int EDLineDetector::EDline(cv::Mat &image, LineChains &lines, bool smoothed)
int EDLineDetector::EDline(image_int8u_p image, LineChains &lines, bool smoothed)
{

	//first, call EdgeDrawing function to extract edges
	EdgeChains edges;
    
	if(!EdgeDrawing(image, edges,smoothed)){
		cout<<"Line Detection not finished"<<endl;
		return -1;
	}
	if (edges.numOfEdges == ZERO)
		return 1;
	
	//detect lines
	unsigned int linePixelID = edges.sId[edges.numOfEdges];
	lines.xCors.resize(linePixelID);
	lines.yCors.resize(linePixelID);
	lines.sId.resize(5*edges.numOfEdges);
	unsigned int *pEdgeXCors = edges.xCors.data();
	unsigned int *pEdgeYCors = edges.yCors.data();
	unsigned int *pEdgeSID   = edges.sId.data();
	unsigned int *pLineXCors = lines.xCors.data();
	unsigned int *pLineYCors = lines.yCors.data();
	unsigned int *pLineSID   = lines.sId.data();
	logNT_ = 2.f * ( log10( (float) imageWidth ) + log10( (float) imageHeight ) );
	float lineFitErr;//the line fit error;
	std::array<float, 2> lineEquation;
	lineEquations_.clear();
	lineEndpoints_.clear();
	lineDirection_.clear();
	unsigned char *pdirImg = dirImg_->data;
	unsigned int numOfLines = 0;
	unsigned int offsetInEdgeArrayS, offsetInEdgeArrayE, newOffsetS;//start index and end index
	unsigned int offsetInLineArray=0;
	float direction;//line direction

    newOffsetS = 0;
    lineFitErr = 0.f;

	for(unsigned int edgeID=0; edgeID<edges.numOfEdges; edgeID++){
		offsetInEdgeArrayS = pEdgeSID[edgeID];
		offsetInEdgeArrayE = pEdgeSID[edgeID+1];
		while(offsetInEdgeArrayE > offsetInEdgeArrayS+minLineLen_){//extract line segments from an edge, may find more than one segments
			//find an initial line segment
			while(offsetInEdgeArrayE > offsetInEdgeArrayS+minLineLen_){
				lineFitErr = LeastSquaresLineFit_(pEdgeXCors,pEdgeYCors,
						offsetInEdgeArrayS,lineEquation);
				if(lineFitErr<=lineFitErrThreshold_) break;//ok, an initial line segment detected
				offsetInEdgeArrayS +=SkipEdgePoint; //skip the first two pixel in the chain and try with the remaining pixels
			}
			if(lineFitErr>lineFitErrThreshold_) break; //no line is detected
			//An initial line segment is detected. Try to extend this line segment
			pLineSID[numOfLines] = offsetInLineArray;
			float coef1 = 0.f;//for a line ax+by+c=0, coef1 = 1/sqrt(a^2+b^2);
			float pointToLineDis;//for a line ax+by+c=0 and a point(xi, yi), pointToLineDis = coef1*|a*xi+b*yi+c|
			bool bExtended=true;
			bool bFirstTry = true;
			int numOfOutlier;//to against noise, we accept a few outlier of a line.
			int tryTimes = 0;
			if(pdirImg[pEdgeYCors[offsetInEdgeArrayS]*imageWidth + pEdgeXCors[offsetInEdgeArrayS]]==Horizontal){//y=ax+b, i.e. ax-y+b=0
				while(bExtended){
					tryTimes++;
					if(bFirstTry){
						bFirstTry = false;
						for(int i=0; i<minLineLen_; i++){//First add the initial line segment to the line array
							pLineXCors[offsetInLineArray]   = pEdgeXCors[offsetInEdgeArrayS];
							pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						}
					}else{//after each try, line is extended, line equation should be re-estimated
						//adjust the line equation
						lineFitErr = LeastSquaresLineFit_(pLineXCors,pLineYCors,pLineSID[numOfLines],
								newOffsetS,offsetInLineArray,lineEquation);
					}
					coef1 = 1/sqrt(lineEquation[0]*lineEquation[0]+1);
					numOfOutlier=0;
					newOffsetS = offsetInLineArray;
					while(offsetInEdgeArrayE > offsetInEdgeArrayS){
						pointToLineDis = fabs(lineEquation[0]*pEdgeXCors[offsetInEdgeArrayS] -
								pEdgeYCors[offsetInEdgeArrayS] + lineEquation[1])*coef1;
						pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
						pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						if(pointToLineDis>lineFitErrThreshold_){
							numOfOutlier++;
							if(numOfOutlier>3) break;
						}else{//we count number of connective outliers.
							numOfOutlier=0;
						}
					}
					//pop back the last few outliers from lines and return them to edge chain
					offsetInLineArray  -=numOfOutlier;
					offsetInEdgeArrayS -=numOfOutlier;
					if(offsetInLineArray - newOffsetS>0&&tryTimes<TryTime){//some new pixels are added to the line
					}else{
						bExtended = false;//no new pixels are added.
					}
				}
				//the line equation coefficients,for line w1x+w2y+w3 =0, we normalize it to make w1^2+w2^2 = 1.
				std::array<float, 3> lineEqu = {lineEquation[0]*coef1, -1*coef1, lineEquation[1]*coef1};
				if(LineValidation_(pLineXCors,pLineYCors,pLineSID[numOfLines],offsetInLineArray,lineEqu,direction)){//check the line
					//store the line equation coefficients
					lineEquations_.push_back(lineEqu);
					/*At last, compute the line endpoints and store them.
					 *we project the first and last pixels in the pixelChain onto the best fit line
					 *to get the line endpoints.
					 *xp= (w2^2*x0-w1*w2*y0-w3*w1)/(w1^2+w2^2)
					 *yp= (w1^2*y0-w1*w2*x0-w3*w2)/(w1^2+w2^2)  */
					std::array<float, 4> lineEndP;//line endpoints
					float a1 = lineEqu[1]*lineEqu[1];
					float a2 = lineEqu[0]*lineEqu[0];
					float a3 = lineEqu[0]*lineEqu[1];
					float a4 = lineEqu[2]*lineEqu[0];
					float a5 = lineEqu[2]*lineEqu[1];
					unsigned int Px = pLineXCors[pLineSID[numOfLines] ];//first pixel
					unsigned int Py = pLineYCors[pLineSID[numOfLines] ];
					lineEndP[0] = a1*Px-a3*Py-a4;//x
					lineEndP[1] = a2*Py-a3*Px-a5;//y
					Px = pLineXCors[offsetInLineArray -1 ];//last pixel
					Py = pLineYCors[offsetInLineArray -1 ];
					lineEndP[2] = a1*Px-a3*Py-a4;//x
					lineEndP[3] = a2*Py-a3*Px-a5;//y
					lineEndpoints_.push_back(lineEndP);
					lineDirection_.push_back(direction);
					numOfLines++;
				}else{
					offsetInLineArray = pLineSID[numOfLines];// line was not accepted, the offset is set back
				}
			}else{//x=ay+b, i.e. x-ay-b=0
				while(bExtended){
					tryTimes++;
					if(bFirstTry){
						bFirstTry = false;
						for(int i=0; i<minLineLen_; i++){//First add the initial line segment to the line array
							pLineXCors[offsetInLineArray]   = pEdgeXCors[offsetInEdgeArrayS];
							pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						}
					}else{//after each try, line is extended, line equation should be re-estimated
						//adjust the line equation
						lineFitErr = LeastSquaresLineFit_(pLineXCors,pLineYCors,pLineSID[numOfLines],
								newOffsetS,offsetInLineArray,lineEquation);
					}
					coef1 = 1/sqrt(1+lineEquation[0]*lineEquation[0]);
					numOfOutlier=0;
					newOffsetS = offsetInLineArray;
					while(offsetInEdgeArrayE > offsetInEdgeArrayS){
						pointToLineDis = fabs(pEdgeXCors[offsetInEdgeArrayS] -
								lineEquation[0]*pEdgeYCors[offsetInEdgeArrayS] - lineEquation[1])*coef1;
						pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
						pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						if(pointToLineDis>lineFitErrThreshold_){
							numOfOutlier++;
							if(numOfOutlier>3) break;
						}else{//we count number of connective outliers.
							numOfOutlier=0;
						}
					}
					//pop back the last few outliers from lines and return them to edge chain
					offsetInLineArray  -= numOfOutlier;
					offsetInEdgeArrayS -= numOfOutlier;
					if(offsetInLineArray - newOffsetS>0&&tryTimes<TryTime){//some new pixels are added to the line
					}else{
						bExtended = false;//no new pixels are added.
					}
				}
				//the line equation coefficients,for line w1x+w2y+w3 =0, we normalize it to make w1^2+w2^2 = 1.
				std::array<float, 3> lineEqu= {1*coef1,-lineEquation[0]*coef1, -lineEquation[1]*coef1};
				if(LineValidation_(pLineXCors,pLineYCors,pLineSID[numOfLines],offsetInLineArray,lineEqu,direction)){//check the line
					//store the line equation coefficients
					lineEquations_.push_back(lineEqu);
					/*At last, compute the line endpoints and store them.
					 *we project the first and last pixels in the pixelChain onto the best fit line
					 *to get the line endpoints.
					 *xp= (w2^2*x0-w1*w2*y0-w3*w1)/(w1^2+w2^2)
					 *yp= (w1^2*y0-w1*w2*x0-w3*w2)/(w1^2+w2^2)  */
					std::array<float, 4> lineEndP;//line endpoints
					float a1 = lineEqu[1]*lineEqu[1];
					float a2 = lineEqu[0]*lineEqu[0];
					float a3 = lineEqu[0]*lineEqu[1];
					float a4 = lineEqu[2]*lineEqu[0];
					float a5 = lineEqu[2]*lineEqu[1];
					unsigned int Px = pLineXCors[pLineSID[numOfLines] ];//first pixel
					unsigned int Py = pLineYCors[pLineSID[numOfLines] ];
					lineEndP[0] = a1*Px-a3*Py-a4;//x
					lineEndP[1] = a2*Py-a3*Px-a5;//y
					Px = pLineXCors[offsetInLineArray -1 ];//last pixel
					Py = pLineYCors[offsetInLineArray -1 ];
					lineEndP[2] = a1*Px-a3*Py-a4;//x
					lineEndP[3] = a2*Py-a3*Px-a5;//y
					lineEndpoints_.push_back(lineEndP);
					lineDirection_.push_back(direction);
					numOfLines++;
				}else{
					offsetInLineArray = pLineSID[numOfLines];// line was not accepted, the offset is set back
				}
			}
			//Extract line segments from the remaining pixel; Current chain has been shortened already.
		}
	}//end for(unsigned int edgeID=0; edgeID<edges.numOfEdges; edgeID++)
	pLineSID[numOfLines] = offsetInLineArray;
	lines.numOfLines = numOfLines;

	return 1;
}


float EDLineDetector::LeastSquaresLineFit_(  unsigned int *xCors,   unsigned int *yCors,
		unsigned int offsetS,		std::array<float, 2> &lineEquation)
{
	if(lineEquation.size()!=2){
	    std::cout<<"SHOULD NOT BE != 2"<<std::endl;
        exit(1);
	}
	float * pMatT = NULL;
    float * pATA = NULL;
	float fitError = 0;
	float coef;
	unsigned char *pdirImg = dirImg_->data;
	unsigned int offset = offsetS;
	/*If the first pixel in this chain is horizontal,
	 *then we try to find a horizontal line, y=ax+b;*/
	if(pdirImg[yCors[offsetS]*imageWidth + xCors[offsetS]]==Horizontal){
		/*Build the system,and solve it using least square regression: mat * [a,b]^T = vec
		 * [x0,1]         [y0]
		 * [x1,1] [a]     [y1]
		 *    .   [b]  =   .
		 * [xn,1]         [yn]*/
		//pMatT= fitMatT.ptr<float>();//fitMatT = [x0, x1, ... xn; 1,1,...,1];
        pMatT = fitMatT->data;//fitMatT = [x0, x1, ... xn; 1,1,...,1];
        
		for(int i=0; i<minLineLen_; i++){
			//*(pMatT+minLineLen_) = 1; //the value are not changed;
			*(pMatT++)           = (float)xCors[offsetS];
            fitVec->data[0 * fitVec->xsize + i] = (float)yCors[offsetS++];
            
		}

        mcv_multiply_transpose_float(fitMatT,ATA);
        mcv_multiply2_transpose_float(fitMatT,fitVec,ATV);

		/* [a,b]^T = Inv(mat^T * mat) * mat^T * vec */
        pATA = ATA->data;
		coef = 1.f/(float(pATA[0])*float(pATA[3]) - float(pATA[1])*float(pATA[2]));
		//		lineEquation = svd.Invert(ATA) * matT * vec;

        lineEquation[0] = coef *(float(pATA[3])*float(ATV->data[0*ATV->xsize+0])
            - float(pATA[1])*float(ATV->data[0 * ATV->xsize + 1]));
        lineEquation[1] = coef *(float(pATA[0])*float(ATV->data[0 * ATV->xsize + 1])
            - float(pATA[2])*float(ATV->data[0 * ATV->xsize + 0]));

		/*compute line fit error */
		for(int i=0; i<minLineLen_; i++){
			coef = float(yCors[offset]) - float(xCors[offset++]) * lineEquation[0] - lineEquation[1];
			fitError += coef*coef;
		}
		return sqrt(fitError);
	}
	/*If the first pixel in this chain is vertical,
	 *then we try to find a vertical line, x=ay+b;*/
	if(pdirImg[yCors[offsetS]*imageWidth + xCors[offsetS]]==Vertical){
		/*Build the system,and solve it using least square regression: mat * [a,b]^T = vec
		 * [y0,1]         [x0]
		 * [y1,1] [a]     [x1]
		 *    .   [b]  =   .
		 * [yn,1]         [xn]*/

        pMatT = fitMatT->data;//fitMatT = [y0, y1, ... yn; 1,1,...,1];

		for(int i=0; i<minLineLen_; i++){
			//*(pMatT+minLineLen_) = 1;//the value are not changed;
            *(pMatT++) = (float)yCors[offsetS];
            fitVec->data[0*fitVec->xsize + i] = (float)xCors[offsetS++];
            
		}

        mcv_multiply_transpose_float(fitMatT, ATA);
        mcv_multiply2_transpose_float(fitMatT, fitVec, ATV);

		/* [a,b]^T = Inv(mat^T * mat) * mat^T * vec */
        pATA = ATA->data;
		coef = 1.f/(float(pATA[0])*float(pATA[3]) - float(pATA[1])*float(pATA[2]));

        lineEquation[0] = coef *(float(pATA[3])*float(ATV->data[0 * ATV->xsize + 0])
            - float(pATA[1])*float(ATV->data[0 * ATV->xsize + 1]));
        lineEquation[1] = coef *(float(pATA[0])*float(ATV->data[0 * ATV->xsize + 1])
            - float(pATA[2])*float(ATV->data[0 * ATV->xsize + 0]));

        
		/*compute line fit error */
		for(int i=0; i<minLineLen_; i++){
			coef = float(xCors[offset]) - float(yCors[offset++]) * lineEquation[0] - lineEquation[1];
			fitError += coef*coef;
		}
		return sqrt(fitError);
	}
	return 0;
}
float EDLineDetector::LeastSquaresLineFit_(  unsigned int *xCors,   unsigned int *yCors,
		unsigned int offsetS, unsigned int newOffsetS,
		unsigned int offsetE,	std::array<float, 2> &lineEquation)
{
	int length = offsetE - offsetS;
	int newLength = offsetE - newOffsetS;
	if(length<=0||newLength<=0){
		cout<<"EDLineDetector::LeastSquaresLineFit_ Error:"
		" the expected line index is wrong...offsetE = "
		<<offsetE<<", offsetS="<<offsetS<<", newOffsetS="<<newOffsetS<<endl;
		return -1;
	}
	if(lineEquation.size()!=2){
	    std::cout<<"SHOULD NOT BE != 2"<<std::endl;
        return -1;
	}

    image_float_p matT = new_image_float(newLength,2);
    image_float_p vec = new_image_float(1,newLength);
	float * pMatT = NULL;
    float * pATA = NULL;
	float coef;
	unsigned char *pdirImg = dirImg_->data;
	/*If the first pixel in this chain is horizontal,
	 *then we try to find a horizontal line, y=ax+b;*/
	if(pdirImg[yCors[offsetS]*imageWidth + xCors[offsetS] ]==Horizontal){
		/*Build the new system,and solve it using least square regression: mat * [a,b]^T = vec
		 * [x0',1]         [y0']
		 * [x1',1] [a]     [y1']
		 *    .    [b]  =   .
		 * [xn',1]         [yn']*/

        pMatT = matT->data;//matT = [x0', x1', ... xn'; 1,1,...,1]
		for(int i=0; i<newLength; i++){
			*(pMatT+newLength) = 1;
            *(pMatT++) = (float)xCors[newOffsetS];
            vec->data[0 * vec->xsize + i] = (float)yCors[newOffsetS++];
		}
		/* [a,b]^T = Inv(ATA + mat^T * mat) * (ATV + mat^T * vec) */
        mcv_multiply_transpose_float(matT, tempMatLineFit);
        mcv_multiply_float(matT, vec, tempVecLineFit);
        mcv_add(ATA,tempMatLineFit,ATA);
        mcv_add(ATV, tempVecLineFit, ATV);


        pATA = ATA->data;
		coef = 1.f/(float(pATA[0])*float(pATA[3]) - float(pATA[1])*float(pATA[2]));

        lineEquation[0] = coef *(float(pATA[3])*float(ATV->data[0 * ATV->xsize + 0])
            - float(pATA[1])*float(ATV->data[0 * ATV->xsize + 1]));
        lineEquation[1] = coef *(float(pATA[0])*float(ATV->data[0 * ATV->xsize + 1])
            - float(pATA[2])*float(ATV->data[0*ATV->xsize+0]));

		free_image_float(matT);
        free_image_float(vec);
		
		return 0;
	}
	/*If the first pixel in this chain is vertical,
	 *then we try to find a vertical line, x=ay+b;*/
	if(pdirImg[yCors[offsetS]*imageWidth + xCors[offsetS] ]==Vertical){
		/*Build the system,and solve it using least square regression: mat * [a,b]^T = vec
		 * [y0',1]         [x0']
		 * [y1',1] [a]     [x1']
		 *    .    [b]  =   .
		 * [yn',1]         [xn']*/

        pMatT = matT->data;//matT = [y0', y1', ... yn'; 1,1,...,1]
		for(int i=0; i<newLength; i++){
			*(pMatT+newLength) = 1;
            *(pMatT++) = (float)yCors[newOffsetS];
            vec->data[0 * vec->xsize + i] = (float)xCors[newOffsetS++];
            
		}
		/* [a,b]^T = Inv(ATA + mat^T * mat) * (ATV + mat^T * vec) */
        mcv_multiply_transpose_float(matT,tempMatLineFit);
        mcv_multiply_float(matT,vec,tempVecLineFit);
        mcv_add(ATA, tempMatLineFit, ATA);
        mcv_add(ATV, tempVecLineFit, ATV);

        pATA = ATA->data;
		coef = 1.f/(float(pATA[0])*float(pATA[3]) - float(pATA[1])*float(pATA[2]));

        lineEquation[0] = coef *(float(pATA[3])*float(ATV->data[0 * ATV->xsize + 0])
            - float(pATA[1])*float(ATV->data[0 * ATV->xsize + 1]));
        lineEquation[1] = coef *(float(pATA[0])*float(ATV->data[0 * ATV->xsize + 1])
             - float(pATA[2])*float(ATV->data[0 * ATV->xsize + 0]));

	}

    free_image_float(matT);
    free_image_float(vec);
	return 0;
}

bool EDLineDetector::LineValidation_(  unsigned int *xCors,   unsigned int *yCors,
		unsigned int offsetS, unsigned int offsetE,
		std::array<float, 3> &lineEquation, float &direction)
{
	if(bValidate_){
		int n = offsetE - offsetS;
		/*first compute the direction of line, make sure that the dark side always be the
		 *left side of a line.*/
		int meanGradientX=0, meanGradientY=0;
        short *pdxImg = dxImg_->data;
        short *pdyImg = dyImg_->data;
		float dx, dy;
		std::vector<float> pointDirection;
		int index;
		for(int i=0; i<n; i++){
			index = yCors[offsetS]*imageWidth + xCors[offsetS++];
			meanGradientX += pdxImg[index];
			meanGradientY += pdyImg[index];
			dx = (float) pdxImg[index];
			dy = (float) pdyImg[index];
			pointDirection.push_back(atan2(-dx,dy));
		}
		dx = fabs(lineEquation[1]);
		dy = fabs(lineEquation[0]);
		if(meanGradientX==0&&meanGradientY==0){//not possible, if happens, it must be a wrong line,
			return false;
		}
		if(meanGradientX>0&&meanGradientY>=0){//first quadrant, and positive direction of X axis.
			direction = atan2(-dy,dx);//line direction is in fourth quadrant
		}
		if(meanGradientX<=0&&meanGradientY>0){//second quadrant, and positive direction of Y axis.
			direction  = atan2(dy,dx);//line direction is in first quadrant
		}
		if(meanGradientX<0&&meanGradientY<=0){//third quadrant, and negative direction of X axis.
			direction = atan2(dy,-dx);//line direction is in second quadrant
		}
		if(meanGradientX>=0&&meanGradientY<0){//fourth quadrant, and negative direction of Y axis.
			direction = atan2(-dy,-dx);//line direction is in third quadrant
		}
		/*then check whether the line is on the border of the image. We don't keep the border line.*/
		if(fabs(direction)<0.15||X_PI-fabs(direction)<0.15f){//Horizontal line
			if(fabs(lineEquation[2])<10||fabs(imageHeight - fabs(lineEquation[2]))<10){//upper border or lower border
				return false;
			}
		}
		if(fabs(fabs(direction)-X_PI*0.5f)<0.15f){//Vertical line
			if(fabs(lineEquation[2])<10||fabs(imageWidth - fabs(lineEquation[2]))<10){//left border or right border
				return false;
			}
		}
		//count the aligned points on the line which have the same direction as the line.
		float disDirection;
		int k = 0;
		for(int i=0; i<n; i++){
			disDirection = fabs(direction - pointDirection[i]);
			if(fabs(2*X_PI-disDirection)<0.392699f||disDirection<0.392699f){//same direction, pi/8 = 0.392699081698724
				k++;
			}
		}
		//now compute NFA(Number of False Alarms)
		float ret = nfa(n,k,0.125f,logNT_);

		return (ret>0); //0 corresponds to 1 mean false alarm
	}else{
		return true;
	}
}

//int EDLineDetector::EDline(cv::Mat &image, bool smoothed)
int EDLineDetector::EDline(image_int8u_p image, bool smoothed)
{
	if(!EDline(image, lines_, smoothed)){
		return -1;
	}
	lineSalience_.clear();
	lineSalience_.resize(lines_.numOfLines);

	short *pgImg  = gImgWO_->data;

	unsigned int indexInLineArray;
	unsigned int *pXCor = lines_.xCors.data();
	unsigned int *pYCor = lines_.yCors.data();
	unsigned int *pSID  = lines_.sId.data();
	for(unsigned int i=0; i<lineSalience_.size(); i++){
		int salience=0;
		for(indexInLineArray = pSID[i];indexInLineArray < pSID[i+1]; indexInLineArray++){
			salience += (unsigned char)pgImg[pYCor[indexInLineArray]*imageWidth+pXCor[indexInLineArray] ];
		}
		lineSalience_[i] = (float) salience;
	}

	return 1;
}


