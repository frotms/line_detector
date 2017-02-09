# edlines-lsd-line_detector
line segment line detector(lsd) &amp; edge drawing line detector(edlines)

test code: main function in TestLineDetectorAlgorithm.cpp

All original dependencies include Opencv have been removed
1.line segment detector(lsd) with a scale in vertical and horizonal direction in C, respectively
2.edge drawing line detector(EDLines) with a scale in vertical and horizonal direction in C++, respectively



EDLines Interface

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
                       
int LineDescriptor::Run(float scaleX, float scaleY, 
    image_int8u_p image, ScaleLineSet & keyLines);
    




LSD Simple Interface with Scale

    @param n_out       Pointer to an int where LSD will store the number of
                       line segments detected.

    @param img         Pointer to input image data. It must be an array of
                       doubles of size w x h, and the pixel at coordinates
                       (x,y) is obtained by img[x+y*w].

    @param w           w size of the image: the number of columns.

    @param h           h size of the image: the number of rows.

    @param scale_x     When different from 1.0, LSD will horizotally scale the input image 
                       by 'scale_x' factor by Gaussian filtering, before detecting
                       line segments.
                       Example: if scale_x=0.8, the input image will be subsampled
                       to 80% of its width, before the line segment detector
                       is applied.
                       Suggested value: 0.8
    @param scale_y     When different from 1.0, LSD will vertically scale the input image 
                       by 'scale_y' factor by Gaussian filtering, before detecting
                       line segments.
                       Example: if scale_y=0.8, the input image will be subsampled
                       to 80% of its height, before the line segment detector
                       is applied.
                       Suggested value: 0.8

    @return            A float array of size 7 x n_out, containing the list
                       of line segments detected. The array contains first
                       7 values of line segment number 1, then the 7 values
                       of line segment number 2, and so on, and it finish
                       by the 7 values of line segment number n_out.
                       The seven values are:                       
					   - x1,y1,x2,y2,width,p,-log10(NFA).

                       for a line segment from coordinates (x1,y1) to (x2,y2),
                       a width 'width', an angle precision of p in (0,1) given
                       by angle_tolerance/180 degree, and NFA value 'NFA'.
                       If 'out' is the returned pointer, the 7 values of
                       line segment number 'n+1' are obtained with
                       'out[7*n+0]' to 'out[7*n+6]'.
float * lsd_scale(int * n_out, unsigned char * img, int w, int h, float scale_x, float scale_y);


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
  

Copyright (c) 2016-2017 Frotms(frotms@gmail.com)
