
#ifndef __HOUGH_LINE_H_
#define __HOUGH_LINE_H_

#include <vector>


typedef enum _HOUGH_LINE_TYPE_CODE
{
	HOUGH_LINE_STANDARD = 0,				  //standad hough line
	HOUGH_LINE_PROBABILISTIC = 1,			  //probabilistic hough line

}HOUGH_LINE_TYPE_CODE;



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
@function    HoughLineDetector
@param       [in]      src:						  image,single channel
@param       [in]      w:                         width of image
@param       [in]      h:                         height of image
@param       [in]      scaleX:                    downscale factor in X-axis
@param       [in]      scaleY:                    downscale factor in Y-axis
@param       [in]      CannyLowThresh:            lower threshold for the hysteresis procedure in canny operator
@param       [in]      CannyHighThresh:           higher threshold for the hysteresis procedure in canny operator
@param       [in]      HoughRho:                  distance resolution of the accumulator in pixels
@param       [in]      HoughTheta:                angle resolution of the accumulator in radians
@param       [in]      MinThetaLinelength:        standard: for standard and multi-scale hough transform, minimum angle to check for lines.
												  propabilistic: minimum line length. Line segments shorter than that are rejected
@param       [in]      MaxThetaGap:               standard: for standard and multi-scale hough transform, maximum angle to check for lines
												  propabilistic: maximum allowed gap between points on the same line to link them
@param       [in]      HoughThresh:               accumulator threshold parameter. only those lines are returned that get enough votes ( >threshold ).
@param       [in]      _type:                     hough line method: HOUGH_LINE_STANDARD or HOUGH_LINE_PROBABILISTIC
@param       [in]      bbox:                      boundingbox to detect
@param       [in/out]  lines:                     result
@return：										  0:ok; 1:error
@brief：     _type: HOUGH_LINE_STANDARD:		  standard hough line algorithm
					HOUGH_LINE_PROBABILISTIC	  probabilistic hough line algorithm
					
When HOUGH_LINE_STANDARD runs, the line points might be the position outside the image coordinate

standard:		try (src,w,h,scalex,scaley,70,150, 1, PI/180, 0, PI, 100, HOUGH_LINE_STANDARD, bbox, line)
propabilistic:  try (src,w,h,scalex,scaley,70,150, 1, PI/180, 30, 10, 80, HOUGH_LINE_STANDARD, bbox, line)
*/
int HoughLineDetector(unsigned char *src, int w, int h,
	float scaleX, float scaleY, float CannyLowThresh, float CannyHighThresh,
	float HoughRho, float HoughTheta, float MinThetaLinelength, float MaxThetaGap, int HoughThresh,
	HOUGH_LINE_TYPE_CODE _type,
	boundingbox_t bbox, std::vector<line_float_t> &lines);


#endif /* HOUGH_H */
