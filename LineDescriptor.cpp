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

#include "LineDescriptor.hh"
#include "mcv_algorithms.h"
#define SalienceScale 0.9F//0.9

using namespace std;

LineDescriptor::LineDescriptor()
{
	//ksize_       = 3;
	edLineVec_.resize(ONE);
    edLineVec_[ZERO] = new EDLineDetector;

	numOfBand_   = 9;//9 is a good value.
	widthOfBand_ = 7;//widthOfBand_%3 must equal to 0; 7 is a good value.
	gaussCoefL_.resize(widthOfBand_*3);
	float u = (float)((widthOfBand_*3-1)/2);
    float sigma = (float)((widthOfBand_ * 2 + 1) / 2);// (widthOfBand_*2+1)/2;
	float invsigma2 = -1/(2*sigma*sigma);
	float dis;
	for(int i=0; i<(int)widthOfBand_*3; i++){
		dis = i-u;
		gaussCoefL_[i] = exp(dis*dis*invsigma2);
	}
	gaussCoefG_.resize(numOfBand_*widthOfBand_);
	u = (float)((numOfBand_*widthOfBand_-1)/2);
	sigma = u;
	invsigma2 = -1/(2*sigma*sigma);
	for(int i=0; i<(int)(numOfBand_*widthOfBand_); i++){
		dis = i-u;
		gaussCoefG_[i] = exp(dis*dis*invsigma2);

	}

}

LineDescriptor::LineDescriptor(unsigned int numOfBand, unsigned int widthOfBand){

	//ksize_       = 5;
    edLineVec_.resize(ONE);
    edLineVec_[ZERO] = new EDLineDetector;

	numOfBand_   = numOfBand;
	widthOfBand_ = widthOfBand;
	gaussCoefL_.resize(widthOfBand_*3);
	float u = (float)((widthOfBand_*3-1)/2);
    float sigma = (float)((widthOfBand_ * 2 + 1) / 2);// (widthOfBand_*2+1)/2;
	float invsigma2 = -1/(2*sigma*sigma);
	float dis;
	for(int i=0; i<(int)widthOfBand_*3; i++){
		dis = i-u;
		gaussCoefL_[i] = exp(dis*dis*invsigma2);
	}
	gaussCoefG_.resize(numOfBand_*widthOfBand_);
    u = (float)((numOfBand_*widthOfBand_ - 1) / 2);
	sigma = u;
	invsigma2 = -1/(2*sigma*sigma);
	for(int i=0; i<(int)(numOfBand_*widthOfBand_); i++){
		dis = i-u;
		gaussCoefG_[i] = exp(dis*dis*invsigma2);

	}

}

LineDescriptor::~LineDescriptor(){
    
    if (edLineVec_[ZERO] != NULL){
        delete edLineVec_[ZERO];
    }

}

/*Line detection method: element in keyLines[i] includes a set of lines which is the same line
 * detected in different scaled images.
 */
//int LineDescriptor::ScaledKeyLines(cv::Mat & image, ScaleLineSet &keyLines)
int LineDescriptor::ScaledKeyLines(image_int8u_p image, ScaleLineSet &keyLines)
{
	unsigned int numOfFinalLine = 0;

    if (!edLineVec_[ZERO]->EDline(image, true)){
        return -1;
    }

    numOfFinalLine += edLineVec_[ZERO]->lines_.numOfLines;

	/*lines which correspond to the same line in the scaled images will be stored in the same element of ScaleLineSet.*/
	std::vector<ScaledLine> scaledLines(numOfFinalLine);//store the lines in ScaledLine structure
	numOfFinalLine = 0;//store the number of finally accepted lines in ScaleLineSet
    unsigned int lineIDInScaleLineVec = 0;
	float dx, dy;
	for(unsigned int lineCurId=0;lineCurId<edLineVec_[0]->lines_.numOfLines;lineCurId++){//add all line detected in the original image
		scaledLines[numOfFinalLine].scaledCount    = 0;
		scaledLines[numOfFinalLine].lineIDInScaled = lineCurId;
		scaledLines[numOfFinalLine].lineIDInScaleLineVec = lineIDInScaleLineVec;
		dx = fabs(edLineVec_[0]->lineEndpoints_[lineCurId][0]-edLineVec_[0]->lineEndpoints_[lineCurId][2]);//x1-x2
		dy = fabs(edLineVec_[0]->lineEndpoints_[lineCurId][1]-edLineVec_[0]->lineEndpoints_[lineCurId][3]);//y1-y2
		scaledLines[numOfFinalLine].lineLength = sqrt(dx*dx+dy*dy);
		numOfFinalLine++;
		lineIDInScaleLineVec++;
	}

	float direction;
	unsigned int lineIDInScaled;

	//Reorganize the detected lines into keyLines
	keyLines.clear();
	keyLines.resize(lineIDInScaleLineVec);
    unsigned int tempID;
	float s1,e1,s2,e2;
	bool shouldChange;
	SingleLineInfo singleLine;
	for(unsigned int  lineID = 0;lineID < numOfFinalLine; lineID++){
		lineIDInScaled = scaledLines[lineID].lineIDInScaled;
		direction      = edLineVec_[ZERO]->lineDirection_[lineIDInScaled];
        singleLine.scaledCount = ZERO;
		singleLine.direction = direction;
		singleLine.lineLength = scaledLines[lineID].lineLength;
        singleLine.salience = edLineVec_[ZERO]->lineSalience_[lineIDInScaled];
        singleLine.numOfPixels = edLineVec_[ZERO]->lines_.sId[lineIDInScaled + 1] -
            edLineVec_[ZERO]->lines_.sId[lineIDInScaled];
		//decide the start point and end point
		shouldChange = false;
        s1 = edLineVec_[ZERO]->lineEndpoints_[lineIDInScaled][0];//sx
        s2 = edLineVec_[ZERO]->lineEndpoints_[lineIDInScaled][1];//sy
        e1 = edLineVec_[ZERO]->lineEndpoints_[lineIDInScaled][2];//ex
        e2 = edLineVec_[ZERO]->lineEndpoints_[lineIDInScaled][3];//ey
		dx = e1 - s1;//ex-sx
		dy = e2 - s2;//ey-sy
		if(direction>=-0.75f*X_PI&&direction<-0.25f*X_PI){
			if(dy>0){shouldChange = true;}
		}
		if(direction>=-0.25f*X_PI&&direction<0.25f*X_PI){
			if(dx<0){shouldChange = true;} 
		}
		if(direction>=0.25f*X_PI&&direction<0.75f*X_PI){
			if(dy<0){shouldChange = true;}
		}
		if((direction>=0.75f*X_PI&&direction<X_PI)||(direction>=-X_PI&&direction<-0.75f*X_PI)){
			if(dx>0){shouldChange = true;}
		}

		if(shouldChange){
			singleLine.sPointInScaledX = e1;
			singleLine.sPointInScaledY = e2;
			singleLine.ePointInScaledX = s1;
			singleLine.ePointInScaledY = s2;
			singleLine.startPointX = e1;
			singleLine.startPointY = e2;
			singleLine.endPointX   = s1;
			singleLine.endPointY   = s2;
		}else{
			singleLine.sPointInScaledX = s1;
			singleLine.sPointInScaledY = s2;
			singleLine.ePointInScaledX = e1;
			singleLine.ePointInScaledY = e2;
			singleLine.startPointX = s1;
			singleLine.startPointY = s2;
			singleLine.endPointX   = e1;
			singleLine.endPointY   = e2;
		}
		tempID = scaledLines[lineID].lineIDInScaleLineVec;
		keyLines[tempID].push_back(singleLine);
	}

  return 1;
}


typedef struct ntuple_list_s
{
    unsigned int size;
    unsigned int max_size;
    unsigned int dim;
    float * values;
} *ntuple_list;

/*----------------------------------------------------------------------------*/
/** Enlarge the allocated memory of an n-tuple list.
*/
static void enlarge_ntuple_list(ntuple_list n_tuple)
{
    /* duplicate number of tuples */
    n_tuple->max_size *= 2;

    /* realloc memory */
    delete[]n_tuple->values;
    n_tuple->values = new float[n_tuple->dim * n_tuple->max_size];

}

static ntuple_list new_ntuple_list(unsigned int dim)
{
    ntuple_list n_tuple = NULL;

    /* get memory for list structure */
    n_tuple = new ntuple_list_s[1];

    /* initialize list */
    n_tuple->size = 0;
    n_tuple->max_size = 1;
    n_tuple->dim = dim;

    /* get memory for tuples */
    n_tuple->values = new float[dim*n_tuple->max_size];

    return n_tuple;
}

static void free_ntuple_list(ntuple_list in)
{
    delete[]in->values;
    delete[]in;
}


static void gaussian_kernel(ntuple_list kernel, float sigma, float mean)
{
    float sum = 0.f;
    float val, isigma;
    unsigned int i;

    isigma = 1.f / sigma;
    /* compute Gaussian kernel */
    if (kernel->max_size < 1) enlarge_ntuple_list(kernel);
    kernel->size = 1;
    for (i = 0; i < kernel->dim; i++)
    {
        val = ((float)i - mean) * isigma;
        kernel->values[i] = exp(-0.5f * val * val);
        sum += kernel->values[i];
    }

    /* normalization */
    if (sum >= 0.f) for (i = 0; i < kernel->dim; i++) kernel->values[i] /= sum;
}


image_int8u_p gaussian_sampler_byte(image_int8u_p in, float_pixel_t scale,
    float sigma_scale)
{
    image_float_p aux = NULL;
    image_int8u_p out = NULL;
    ntuple_list kernel_x = NULL, kernel_y = NULL;
    unsigned int N, M, x, y, i;
    unsigned int _h, _w, nx, ny;
    int xc, yc, j, float_x_size, float_y_size;
    float xx, yy, sum;//, prec;//, iscale;
    float_pixel_t iscale, sigma;

    iscale.u = 1.f / scale.u;
    iscale.v = 1.f / scale.v;

    N = (unsigned int)ceil(in->xsize * scale.u);
    M = (unsigned int)ceil(in->ysize * scale.v);
    aux = new_image_float(N, in->ysize);
    out = new_image_int8u(N, M);

    /* sigma, kernel size and memory for the kernel */
    //sigma = scale < 1.f ? sigma_scale * iscale : sigma_scale;
    sigma.u = (scale.u < 1.f) ? (sigma_scale * iscale.u) : (sigma_scale);
    sigma.v = (scale.v < 1.f) ? (sigma_scale * iscale.v) : (sigma_scale);

    //prec = 3.f;
    _h = (unsigned int)ceil(sigma.v * 3.71692219f);//  sqrt(prec * 4.605170186f)); //log(10)*2  
    _w = (unsigned int)ceil(sigma.u * 3.71692219f);//  sqrt(prec * 4.605170186f)); //log(10)*2  

    nx = 1 + 2 * _w; /* kernel size */
    ny = 1 + 2 * _h; /* kernel size */
    kernel_x = new_ntuple_list(nx);
    kernel_y = new_ntuple_list(ny);

    /* auxiliary float image size variables */
    float_x_size = (int)(2 * in->xsize);
    float_y_size = (int)(2 * in->ysize);

    /* First subsampling: x axis */
    for (x = 0; x<aux->xsize; x++)
    {

        xx = (float)x * iscale.u;
        /* coordinate (0.0,0.0) is in the center of pixel (0,0),
        so the pixel with xc=0 get the values of xx from -0.5 to 0.5 */
        xc = (int)floor(xx + ROUND);
        gaussian_kernel(kernel_x, sigma.u, (float)_w + xx - (float)xc);
        /* the kernel must be computed for each x because the fine
        offset xx-xc is different in each case */

        for (y = 0; y<aux->ysize; y++)
        {
            sum = 0.f;
            for (i = 0; i<kernel_x->dim; i++)
            {
                j = xc - _w + i;

                /* symmetry boundary condition */
                while (j < 0) j += float_x_size;
                while (j >= float_x_size) j -= float_x_size;
                if (j >= (int)in->xsize) j = float_x_size - 1 - j;

                sum += in->data[j + y * in->xsize] * kernel_x->values[i];
            }
            aux->data[x + y * aux->xsize] = sum;
        }
    }

    /* Second subsampling: y axis */
    for (y = 0; y<out->ysize; y++)
    {
        yy = (float)y * iscale.v;
        /* coordinate (0.0,0.0) is in the center of pixel (0,0),
        so the pixel with yc=0 get the values of yy from -0.5 to 0.5 */
        yc = (int)floor(yy + ROUND);
        gaussian_kernel(kernel_y, sigma.v, (float)_h + yy - (float)yc);
        /* the kernel must be computed for each y because the fine
        offset yy-yc is different in each case */

        for (x = 0; x<out->xsize; x++)
        {
            sum = 0.0f;
            for (i = 0; i<kernel_y->dim; i++)
            {
                j = yc - _h + i;

                /* symmetry boundary condition */
                while (j < 0) j += float_y_size;
                while (j >= float_y_size) j -= float_y_size;
                if (j >= (int)in->ysize) j = float_y_size - 1 - j;

                sum += aux->data[x + j * aux->xsize] * kernel_y->values[i];
            }
            out->data[x + y * out->xsize] = (unsigned char)(sum + ROUND);
        }
    }

    /* free memory */
    free_ntuple_list(kernel_x);
    free_ntuple_list(kernel_y);
    free_image_float(aux);

    return out;
}


/*This function is used to get numbers of pixels in a line from image.*/
int LineDescriptor::GetLinePixelsNums(float startX, float startY, float endX, float endY)
{
    int num;
    int nstarX,nstartY,nendX,nendY;

    nstarX = (int)(startX + ROUND);
    nstartY = (int)(startY + ROUND);
    nendX = (int)(endX + ROUND);
    nendY = (int)(endY + ROUND);

    num = MAX(abs(nendX - nstarX) + 1, abs(nendY - nstartY) + 1);
    
    return num;
}


/*This function is used to get information of lines from downsampled image.*/
void LineDescriptor::InverseGaussianSamplerLines(float_pixel_t gs_scale, ScaleLineSet & keyLines)
{
    float iscale_u,iscale_v;
    float delta_u,delta_v;
    int k,nsize;

    iscale_u = 1.f / gs_scale.u;
    iscale_v = 1.f / gs_scale.v;

    nsize = (int)keyLines.size();

    if ((gs_scale.u != 1.f) && (gs_scale.v != 1.f))
    {
        for (k=0;k<nsize;k++)
        {
            keyLines[k][0].startPointX *= iscale_u;
            keyLines[k][0].startPointY *= iscale_v;
            keyLines[k][0].endPointX *= iscale_u;
            keyLines[k][0].endPointY *= iscale_v; 

            delta_u = keyLines[k][0].endPointX - keyLines[k][0].startPointX;
            delta_v = keyLines[k][0].endPointY - keyLines[k][0].startPointY;

            keyLines[k][0].direction = atan2(delta_v,delta_u);
            keyLines[k][0].lineLength = sqrt(delta_u*delta_u + delta_v*delta_v);
            keyLines[k][0].numOfPixels = GetLinePixelsNums(keyLines[k][0].startPointX,
                keyLines[k][0].startPointY, keyLines[k][0].endPointX, keyLines[k][0].endPointY);
            
        }
    }
    else
    if ((gs_scale.u == 1.f) && (gs_scale.v != 1.f))
    {
        for (k = 0; k < nsize; k++)
        {
            keyLines[k][0].startPointY *= iscale_v;
            keyLines[k][0].endPointY *= iscale_v;

            delta_u = keyLines[k][0].endPointX - keyLines[k][0].startPointX;
            delta_v = keyLines[k][0].endPointY - keyLines[k][0].startPointY;

            keyLines[k][0].direction = atan2(delta_v, delta_u);
            keyLines[k][0].lineLength = sqrt(delta_u*delta_u + delta_v*delta_v);
            keyLines[k][0].numOfPixels = GetLinePixelsNums(keyLines[k][0].startPointX,
                keyLines[k][0].startPointY, keyLines[k][0].endPointX, keyLines[k][0].endPointY);
        }
    }
    else
    if ((gs_scale.u != 1.f) && (gs_scale.v == 1.f))
    {
        for (k = 0; k < nsize; k++)
        {
            keyLines[k][0].startPointX *= iscale_u;
            keyLines[k][0].endPointX *= iscale_u;

            delta_u = keyLines[k][0].endPointX - keyLines[k][0].startPointX;
            delta_v = keyLines[k][0].endPointY - keyLines[k][0].startPointY;

            keyLines[k][0].direction = atan2(delta_v, delta_u);
            keyLines[k][0].lineLength = sqrt(delta_u*delta_u + delta_v*delta_v);
            keyLines[k][0].numOfPixels = GetLinePixelsNums(keyLines[k][0].startPointX,
                keyLines[k][0].startPointY, keyLines[k][0].endPointX, keyLines[k][0].endPointY);
        }
    }
}


/*----------------------------------------------------------------------------*/
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
int LineDescriptor::Run(float scaleX, float scaleY, 
    image_int8u_p image, ScaleLineSet & keyLines)
{
    image_int8u_p blur = NULL;
    float_pixel_t gs_scale;
    float sigma_scale;

    gs_scale.u = scaleX;
    gs_scale.v = scaleY;
    sigma_scale = 0.6f;

    blur = gaussian_sampler_byte(image, gs_scale, sigma_scale);
    
    if (!ScaledKeyLines(blur, keyLines)){
        cout<<"ScaledKeyLines failed"<<endl;
        return -1; 
    }

    InverseGaussianSamplerLines(gs_scale, keyLines);

    delete[]blur->data;
    delete[]blur;

    return 1;
}

