#ifndef __LSD_LINE_H_
#define __LSD_LINE_H_

#include <vector>

typedef struct
{
	int x;                               
	int y;                           
	int width;                       
	int height;                       
}boundingbox_t;


typedef struct
{
	float startx;
	float starty;
	float endx;
	float endy;
}line_float_t;

/*
@function    LsdLineDetector
@param       [in]      src:						  image,single channel
@param       [in]      w:                         width of image
@param       [in]      h:                         height of image
@param       [in]      scaleX:                    downscale factor in X-axis
@param       [in]      scaleY:                    downscale factor in Y-axis
@param       [in]      bbox:                      boundingbox to detect
@param       [in/out]  lines:                     result
@return£º										  0:ok; 1:error
@brief£º

*/
int LsdLineDetector(unsigned char *src, int w, int h,
	float scaleX, float scaleY, boundingbox_t bbox, std::vector<line_float_t> &lines);


#endif