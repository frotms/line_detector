# three mainstream line detectors: lsd-lines, ed-lines and hough-lines

1. __segment line detector (lsd)__ 
2. __edge drawing line detector (edlines)__
3. __hough line detector (standard and probabilistic)__

__All original dependencies have been removed. Code could be run  independently.__

1. line segment detector(lsd) with a scale in vertical and horizonal direction in boundingbox, respectively
2. edge drawing line detector(EDLines) with a scale in vertical and horizonal direction in boundingbox, respectively
3. hough line detector(standard and probabilistic) with a scale in vertical and horizonal direction in boundingbox, respectively



__EDLines__ Simple Interface with Scale in Boundingbox

    @param src         				image,single channel.

    @param w           				width of image.

    @param h           				height of image.

    @param scaleX      				downscale factor in X-axis.

    @param scaleY      				downscale factor in Y-axis.

    @param bbox        				boundingbox to detect.

    @param lines      				result.

    @return            				0:ok; 1:error
                       
int __EdgeDrawingLineDetector__(unsigned char *src, int w, int h,float scaleX, scaleY, boundingbox_t bbox, std::vector<line_float_t> &lines);



__LSD__ Simple Interface with Scale in Boundingbox

    @param src         				image,single channel.

    @param w           				width of image.

    @param h           				height of image.

    @param scaleX      				downscale factor in X-axis.

    @param scaleY      				downscale factor in Y-axis.

    @param bbox       			 	boundingbox to detect.

    @param lines       				result.

    @return            				0:ok; 1:error
                       
int __LsdLineDetector__(unsigned char *src, int w, int h, float scaleX, float scaleY, boundingbox_t bbox, std::vector<line_float_t> &lines);

__Houghline__ Simple Interface with Scale in Boundingbox

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
                       
int __HoughLineDetector__(unsigned char *src, int w, int h, float scaleX, float scaleY, float CannyLowThresh, float CannyHighThresh, float HoughRho, float HoughTheta, float MinThetaLinelength, float MaxThetaGap, int HoughThresh, HOUGH_LINE_TYPE_CODE _type, boundingbox_t bbox, std::vector<line_float_t> &lines);


LSD is an implementation of the Line Segment Detector on digital
images described in the paper:

  "LSD: A Fast Line Segment Detector with a False Detection Control"
  by Rafael Grompone von Gioi, Jeremie Jakubowicz, Jean-Michel Morel,
  and Gregory Randall, IEEE Transactions on Pattern Analysis and
  Machine Intelligence, vol. 32, no. 4, pp. 722-732, April, 2010.

and in more details in the CMLA Technical Report:

  "LSD: A Line Segment Detector, Technical Report",
  by Rafael Grompone von Gioi, Jeremie Jakubowicz, Jean-Michel Morel,
  Gregory Randall, CMLA, ENS Cachan, 2010.
  
  
  To extract edge and lines, this library implements the EDLines Algorithm and the Edge Drawing detector:
  
  Akinlar C, Topal C. EDLines: A real-time line segment detector with a false detection control[J]. 
  Pattern Recognition Letters, 2011, 32(13): 1633-1642.
  
  Topal C, Akinlar C. Edge drawing: a combined real-time edge and segment detector[J]. 
  Journal of Visual Communication and Image Representation, 2012, 23(6): 862-872.
  
  Zhang L, Koch R. An efficient and robust line segment matching approach based on LBD descriptor 
  and pairwise geometric consistency[J]. Journal of Visual Communication and Image Representation, 
  2013, 24(7): 794-805.
  
  J. Canny. A Computational Approach to Edge Detection, IEEE Trans. on Pattern Analysis and Machine Intelligence, 8(6), pp. 679-698 (1986).

  Matas, J. and Galambos, C. and Kittler, J.V., Robust Detection of Lines Using the Progressive Probabilistic Hough Transform. CVIU 78 1, pp 119-137 (2000).

Copyright (c) 2016-2017 Frotms(frotms@gmail.com)
