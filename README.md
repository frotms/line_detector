# Three mainstream line detectors: lsd-lines, ed-lines and hough-lines

- segment line detector (lsd)
- edge drawing line detector (edlines)
- hough line detector (standard and probabilistic)

**All original dependencies have been removed. Code could be run  independently:**

- line segment detector with a scale in vertical and horizontal direction in boundingbox, respectively
- edge drawing line detector with a scale in vertical and horizontal direction in boundingbox, respectively
- hough line detector(standard and probabilistic) with a scale in vertical and horizontal direction in boundingbox, respectively

## EDLines

EDLines Simple Interface with Scale in Boundingbox

    @param src         				image,single channel.
    
    @param w           				width of image.
    
    @param h           				height of image.
    
    @param scaleX      				downscale factor in X-axis.
    
    @param scaleY      				downscale factor in Y-axis.
    
    @param bbox        				boundingbox to detect.
    
    @param lines      				result.
    
    @return            				0:ok; 1:error
    int EdgeDrawingLineDetector(unsigned char *src, int w, int h,float scaleX, scaleY, boundingbox_t bbox, std::vector<line_float_t> &lines);

## LSD

LSD Simple Interface with Scale in Boundingbox

    @param src         				image,single channel.
    
    @param w           				width of image.
    
    @param h           				height of image.
    
    @param scaleX      				downscale factor in X-axis.
    
    @param scaleY      				downscale factor in Y-axis.
    
    @param bbox       			 	boundingbox to detect.
    
    @param lines       				result.
    
    @return            				0:ok; 1:error
    int LsdLineDetector(unsigned char *src, int w, int h, float scaleX, float scaleY, boundingbox_t bbox, std::vector<line_float_t> &lines);

## Houghline

Houghline Simple Interface with Scale in Boundingbox

    @param src         				image,single channel.
    
    @param w           				width of image.
    
    @param h           				height of image.
    
    @param scaleX      			 	downscale factor in X-axis.
    
    @param scaleY      			 	downscale factor in Y-axis.
    
    @param canny_low_thresh      	lower threshold for the hysteresis procedure in canny operator.
    
    @param canny_high_thresh      	higher threshold for the hysteresis procedure in canny operator.
    
    @param hough_rho      			distance resolution of the accumulator in pixels.
    
    @param hough_theta      		angle resolution of the accumulator in radians.
    
    @param min_theta_linelength     standard: for standard and multi-scale hough transform, minimum angle to check for lines; propabilistic: minimum line length. Line segments shorter than that are rejected.
    
    @param max_theta_gap      		standard: for standard and multi-scale hough transform, maximum angle to check for lines; propabilistic: maximum allowed gap between points on the same line to link them.
    
    @param hough_thresh      		accumulator threshold parameter. only those lines are returned that get enough votes ( >threshold ).
    
    @param _type      				hough line method: HOUGH_LINE_STANDARD or HOUGH_LINE_PROBABILISTIC
    
    @param bbox        				boundingbox to detect.
    
    @param lines       				result.
    
    @return            				0:ok; 1:error
    int HoughLineDetector(unsigned char *src, int w, int h, float scaleX, float scaleY, float CannyLowThresh, float CannyHighThresh, float HoughRho, float HoughTheta, float MinThetaLinelength, float MaxThetaGap, int HoughThresh, HOUGH_LINE_TYPE_CODE _type, boundingbox_t bbox, std::vector<line_float_t> &lines);

## Reference

-  [LSD: A Line Segment Detector](http://www.ipol.im/pub/art/2012/gjmr-lsd/)
- [LBD_Descriptor](https://github.com/mtamburrano/LBD_Descriptor)
- [OpenCV](https://opencv.org/)