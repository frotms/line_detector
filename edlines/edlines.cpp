
#include "edlines.h"

#include <list>
#include <array>

using namespace std;


#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif

#ifndef ZERO
#define ZERO (0)
#endif

#ifndef ROUND
#define ROUND (0.5F)
#endif

#ifndef MIN
#define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

//if |dx|<|dy|;
#ifndef Horizontal
#define Horizontal (255)
#endif
//if |dy|<=|dx|;
#ifndef Vertical
#define Vertical    0
#endif

#ifndef UpDir
#define UpDir       1
#endif

#ifndef RightDir
#define RightDir    2
#endif

#ifndef DownDir
#define DownDir     3
#endif

#ifndef LeftDir
#define LeftDir     4
#endif

#ifndef TryTime
#define TryTime     6
#endif

#ifndef SkipEdgePoint
#define SkipEdgePoint 2
#endif

#ifndef RELATIVE_ERROR_FACTOR
#define RELATIVE_ERROR_FACTOR   100.0f
#endif

#ifndef M_LN10
#define M_LN10   2.302585093f 
#endif

#ifndef log_gamma
#define log_gamma(x)    ((x)>15.f?log_gamma_windschitl(x):log_gamma_lanczos(x))
#endif

#ifndef SalienceScale
#define SalienceScale 0.9F//0.9
#endif

#ifndef ONE
#define ONE (1)
#endif

using namespace std;

typedef struct
{
	float u;            //col of pixel
	float v;            //row of pixel
}pixel_float_t;

typedef struct image_int8u_s
{
	unsigned char * data;
	unsigned int xsize, ysize;
} *image_int8u_p;

typedef struct image_int16s_s
{
	short * data;
	unsigned int xsize, ysize;
} *image_int16s_p;

typedef struct image_int32s_s
{
	int * data;
	unsigned int xsize, ysize;
} *image_int32s_p;

typedef struct image_float_s
{
	float * data;
	unsigned int xsize, ysize;
} *image_float_p;

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




struct ScaledLine{
	unsigned int scaledCount;//the scaled which this line is detected
	unsigned int lineIDInScaled;//the line ID in that scaled image
	unsigned int lineIDInScaleLineVec;//the line ID in Scale line vector
	float lineLength; //the length of line in original image scale
};


struct Pixel{
	unsigned int x;//X coordinate
	unsigned int y;//Y coordinate
};
struct EdgeChains{
	std::vector<unsigned int> xCors;//all the x coordinates of edge points
	std::vector<unsigned int> yCors;//all the y coordinates of edge points
	std::vector<unsigned int> sId;  //the start index of each edge in the coordinate arrays
	unsigned int numOfEdges;//the number of edges whose length are larger than minLineLen; numOfEdges < sId.size;
};
struct LineChains{
	std::vector<unsigned int> xCors;//all the x coordinates of line points
	std::vector<unsigned int> yCors;//all the y coordinates of line points
	std::vector<unsigned int> sId;  //the start index of each line in the coordinate arrays
	unsigned int numOfLines;//the number of lines whose length are larger than minLineLen; numOfLines < sId.size;
};

typedef  std::list<Pixel> PixelChain;//each edge is a pixel chain

 
struct EDLineParam{

	float gradientThreshold;
	float anchorThreshold;
	int scanIntervals;
	int minLineLen;
	float lineFitErrThreshold;
};


static image_int8u_p new_image_int8u_ptr(unsigned int xsize,
	unsigned int ysize, unsigned char * data)
{
	image_int8u_p image = NULL;

	/* get memory */
	image = new image_int8u_s[1];
	if (!image)exit(1);

	/* set image */
	image->xsize = xsize;
	image->ysize = ysize;
	image->data = data;

	return image;
}


static image_int8u_p new_image_int8u(unsigned int xsize, unsigned int ysize)
{
	image_int8u_p image = NULL;

	/* get memory */
	image = new image_int8u_s[1];
	image->data = new unsigned char[xsize*ysize];

	/* set image size */
	image->xsize = xsize;
	image->ysize = ysize;

	return image;
}

static image_int16s_p new_image_int16s(unsigned int xsize, unsigned int ysize)
{
	image_int16s_p image = NULL;

	/* get memory */
	image = new image_int16s_s[1];
	image->data = new short[xsize*ysize];

	/* set image size */
	image->xsize = xsize;
	image->ysize = ysize;

	return image;
}



static image_float_p new_image_float(unsigned int xsize, unsigned int ysize)
{
	image_float_p image = NULL;

	/* get memory */
	image = new image_float_s[1];
	image->data = new float[xsize*ysize];

	/* set image size */
	image->xsize = xsize;
	image->ysize = ysize;

	return image;
}


void free_image_float(image_float_p i)
{
	delete[]i->data;
	delete[]i;
}


void free_image_int16s(image_int16s_p i)
{
	delete[]i->data;
	delete[]i;
}


static void free_image_int32s(image_int32s_p i)
{
	delete[]i->data;
	delete[]i;
}

static void free_image_int8u(image_int8u_p i)
{
	delete[]i->data;
	delete[]i;
}


class EDLineDetector
{
public:
	EDLineDetector();
	EDLineDetector(EDLineParam param);
	~EDLineDetector();

public:

	/*extract edges from image
	*image:    In, gray image;
	*edges:    Out, store the edges, each edge is a pixel chain
	*smoothed: In, flag to mark whether the image has already been smoothed by Gaussian filter.
	*return 1: error happen
	*/
	int EdgeDrawing(image_int8u_p image, EdgeChains &edgeChains, bool smoothed = false);

	/*extract lines from image
	*image:    In, gray image;
	*lines:    Out, store the extracted lines,
	*smoothed: In, flag to mark whether the image has already been smoothed by Gaussian filter.
	*return 1: error happen
	*/
	int EDline(image_int8u_p image, LineChains &lines, bool smoothed = false);

	/*extract line from image, and store them*/
	int EDline(image_int8u_p image, bool smoothed = false);


public:
	image_int16s_p dxImg_;
	image_int16s_p dyImg_;
	//store the gradient image without threshold;
	image_int16s_p gImgWO_;

	LineChains lines_; //store the detected line chains;
	//store the line Equation coefficients, vec3=[w1,w2,w3] for line w1*x + w2*y + w3=0;
	std::vector<std::array<float, 3> > lineEquations_;
	//store the line endpoints, [x1,y1,x2,y3]
	std::vector<std::array<float, 4> > lineEndpoints_;
	//store the line direction
	std::vector<float>  lineDirection_;
	//store the line salience, which is the summation of gradients of pixels on line
	std::vector<float>  lineSalience_;
	unsigned int imageWidth;
	unsigned int imageHeight;

private:
	void InitEDLine_();
	/*For an input edge chain, find the best fit line, the default chain length is minLineLen_
	*xCors:  In, pointer to the X coordinates of pixel chain;
	*yCors:  In, pointer to the Y coordinates of pixel chain;
	*offsetS:In, start index of this chain in array;
	*lineEquation: Out, [a,b] which are the coefficient of lines y=ax+b(horizontal) or x=ay+b(vertical);
	*return:  line fit error; 1:error happens;
	*/
	float LeastSquaresLineFit_(unsigned int *xCors, unsigned int *yCors,
		unsigned int offsetS, std::array<float, 2> &lineEquation);
	/*For an input pixel chain, find the best fit line. Only do the update based on new points.
	*For A*x=v,  Least square estimation of x = Inv(A^T * A) * (A^T * v);
	*If some new observations are added, i.e, [A; A'] * x = [v; v'],
	*then x' = Inv(A^T * A + (A')^T * A') * (A^T * v + (A')^T * v');
	*xCors:  In, pointer to the X coordinates of pixel chain;
	*yCors:  In, pointer to the Y coordinates of pixel chain;
	*offsetS:In, start index of this chain in array;
	*newOffsetS: In, start index of extended part;
	*offsetE:In, end index of this chain in array;
	*lineEquation: Out, [a,b] which are the coefficient of lines y=ax+b(horizontal) or x=ay+b(vertical);
	*return:  line fit error; 1:error happens;
	*/
	float LeastSquaresLineFit_(unsigned int *xCors, unsigned int *yCors,
		unsigned int offsetS, unsigned int newOffsetS,
		unsigned int offsetE, std::array<float, 2> &lineEquation);
	/* Validate line based on the Helmholtz principle, which basically states that
	* for a structure to be perceptually meaningful, the expectation of this structure
	* by chance must be very low.
	*/
	bool LineValidation_(unsigned int *xCors, unsigned int *yCors,
		unsigned int offsetS, unsigned int offsetE,
		std::array<float, 3> &lineEquation, float &direction);


private:

	bool bValidate_;//flag to decide whether line will be validated

	/*the threshold of pixel gradient magnitude.
	*Only those pixel whose gradient magnitude are larger than this threshold will be
	*taken as possible edge points. Default value is 36*/
	short gradienThreshold_;
	/*If the pixel's gradient value is bigger than both of its neighbors by a
	*certain threshold (ANCHOR_THRESHOLD), the pixel is marked to be an anchor.
	*Default value is 8*/
	unsigned char anchorThreshold_;
	/*anchor testing can be performed at different scan intervals, i.e.,
	*every row/column, every second row/column etc.
	*Default value is 2*/
	unsigned int scanIntervals_;
	int minLineLen_;//minimal acceptable line length

	/*This type of storage order is because of the order of edge detection process.
	*For each edge, start from one anchor point, first go right, then go left or first go down, then go up*/
	unsigned int *pFirstPartEdgeX_;//store the X coordinates of the first part of the pixels for chains
	unsigned int *pFirstPartEdgeY_;//store the Y coordinates of the first part of the pixels for chains
	unsigned int *pFirstPartEdgeS_;//store the start index of every edge chain in the first part arrays
	unsigned int *pSecondPartEdgeX_;//store the X coordinates of the second part of the pixels for chains
	unsigned int *pSecondPartEdgeY_;//store the Y coordinates of the second part of the pixels for chains
	unsigned int *pSecondPartEdgeS_;//store the start index of every edge chain in the second part arrays
	unsigned int *pAnchorX_;//store the X coordinates of anchors
	unsigned int *pAnchorY_;//store the Y coordinates of anchors

	image_int8u_p edgeImage_;

	float lineFitErrThreshold_;

	//store the gradient image;
	image_int16s_p gImg_;
	//store the direction image
	image_int8u_p dirImg_;

	float logNT_;

	image_float_p ATA;	//the previous matrix of A^T * A;
	image_float_p ATV;	//the previous vector of A^T * V;
	image_float_p fitMatT;	//the matrix used in line fit function;
	image_float_p fitVec;	//the vector used in line fit function;
	image_float_p tempMatLineFit;	//the matrix used in line fit function;
	image_float_p tempVecLineFit;	//the vector used in line fit function;


	/** Compare floats by relative error.*/
	static int float_equal(float a, float b)
	{
		float abs_diff, aa, bb, abs_max;
		/* trivial case */
		if (a == b) return true;
		abs_diff = fabs(a - b);
		aa = fabs(a);
		bb = fabs(b);
		abs_max = aa > bb ? aa : bb;

		if (abs_max < FLT_MIN) abs_max = FLT_MIN;
		/* equal if relative error <= factor x eps */
		return (abs_diff / abs_max) <= (RELATIVE_ERROR_FACTOR * DBL_EPSILON);
	}
	/** Computes the natural logarithm of the absolute value of
	the gamma function of x using the Lanczos approximation.*/
	static float log_gamma_lanczos(float x)
	{
		static float q[7] = { 75122.6331530f, 80916.6278952f, 36308.2951477f,
			8687.24529705f, 1168.92649479f, 83.8676043424f,
			2.50662827511f };
		float a = (x + 0.5f) * log(x + 5.5f) - (x + 5.5f);
		float b = 0.f;
		int n;
		for (n = 0; n<7; n++){
			a -= log(x + (float)n);
			b += q[n] * pow(x, (float)n);
		}
		return a + log(b);
	}
	/** Computes the natural logarithm of the absolute value of
	the gamma function of x using Windschitl method.*/
	static float log_gamma_windschitl(float x)
	{
		return 0.918938533204673f + (x - 0.5f)*log(x) - x
			+ 0.5f*x*log(x*sinh(1 / x) + 1 / (810.f*pow(x, 6.f)));
	}
	/** Computes -log10(NFA).*/
	static float nfa(int n, int 	k, float p, float  logNT)
	{
		float tolerance = 0.1f;       /* an error of 10% in the result is accepted */
		float log1term, term, bin_term, mult_term, bin_tail, err, p_term;
		int i;

		/* check parameters */
		if (n<0 || k<0 || k>n || p <= 0.f || p >= 1.f){
			printf("nfa: wrong n, k or p values.\n");
			exit(1);
		}
		/* trivial cases */
		if (n == 0 || k == 0) return -logNT;
		if (n == k) return -logNT - (float)n * log10(p);

		/* probability term */
		p_term = p / (1.f - p);

		/* compute the first term of the series */
		log1term = log_gamma((float)n + 1.f) - log_gamma((float)k + 1.f)
			- log_gamma((float)(n - k) + 1.f)
			+ (float)k * log(p) + (float)(n - k) * log(1.f - p);
		term = exp(log1term);

		/* in some cases no more computations are needed */
		if (float_equal(term, 0.f)){  /* the first term is almost zero */
			if ((float)k > (float)n * p)     /* at begin or end of the tail?  */
				return -log1term / M_LN10 - logNT;  /* end: use just the first term  */
			else
				return -logNT;                      /* begin: the tail is roughly 1  */
		}

		/* compute more terms if needed */
		bin_tail = term;
		for (i = k + 1; i <= n; i++){

			bin_term = (float)(n - i + 1) / (float)i;
			mult_term = bin_term * p_term;
			term *= mult_term;
			bin_tail += term;
			if (bin_term<1.f){

				err = term * ((1.f - pow(mult_term, (float)(n - i + 1)))
					/ (1.f - mult_term) - 1.f);

				if (err < tolerance * fabs(-log10(bin_tail) - logNT) * bin_tail) break;
			}
		}
		return -log10(bin_tail) - logNT;
	}
};


/* This class is used to generate the line descriptors from multi-scale images  */
class LineDescriptor
{
public:
	LineDescriptor();
	LineDescriptor(unsigned int numOfBand, unsigned int widthOfBand);
	~LineDescriptor();

public:

	/* Interface.*/
	int Run(float scaleX, float scaleY, boundingbox_t bbox,
		image_int8u_p image, ScaleLineSet & keyLines);

private:

	/*This function is used to detect lines from multi-scale images.*/
	int ScaledKeyLines(image_int8u_p image, ScaleLineSet &keyLines);

	/*This function is used to get numbers of pixels in a line from image.*/
	int GetLinePixelsNums(float startX, float startY, float endX, float endY);

	/*This function is used to get information of lines from downsampled image.*/
	void InverseGaussianSamplerLines(pixel_float_t gs_scale, ScaleLineSet &keyLines);



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

EDLineDetector::EDLineDetector()
{
	//set parameters for line segment detection

	gradienThreshold_ = 80; // ***** ORIGINAL WAS 25
	anchorThreshold_ = 2;//8
	scanIntervals_ = 2;//2
	minLineLen_ = 15;
	lineFitErrThreshold_ = 1.4f;


	InitEDLine_();
}
EDLineDetector::EDLineDetector(EDLineParam param)
{
	//set parameters for line segment detection
	gradienThreshold_ = (short)param.gradientThreshold;
	anchorThreshold_ = (unsigned char)param.anchorThreshold;
	scanIntervals_ = param.scanIntervals;
	minLineLen_ = param.minLineLen;
	lineFitErrThreshold_ = param.lineFitErrThreshold;
	InitEDLine_();
}
void EDLineDetector::InitEDLine_()
{
	bValidate_ = true;

	ATA = new_image_float(2, 2);
	ATV = new_image_float(1, 2);
	tempMatLineFit = new_image_float(2, 2);
	tempVecLineFit = new_image_float(1, 2);
	fitMatT = new_image_float(minLineLen_, 2);
	fitVec = new_image_float(minLineLen_, 1);


	for (int i = 0; i<minLineLen_; i++){
		fitMatT->data[1 * fitMatT->xsize + i] = 1;
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
	if (pFirstPartEdgeX_ != NULL){
		delete[] pFirstPartEdgeX_; pFirstPartEdgeX_ = NULL;
		delete[] pFirstPartEdgeY_; pFirstPartEdgeY_ = NULL;
		delete[] pSecondPartEdgeX_; pSecondPartEdgeX_ = NULL;
		delete[] pSecondPartEdgeY_; pSecondPartEdgeY_ = NULL;
		delete[] pAnchorX_; pAnchorX_ = NULL;
		delete[] pAnchorY_; pAnchorY_ = NULL;
	}
	if (pFirstPartEdgeS_ != NULL){
		delete[] pFirstPartEdgeS_; pFirstPartEdgeS_ = NULL;
		delete[] pSecondPartEdgeS_; pSecondPartEdgeS_ = NULL;
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

	lineEquations_.clear();
	lineEndpoints_.clear();
	lineDirection_.clear();
	lineSalience_.clear();

}


typedef enum _ORIENT_CODE
{
	ORIENT_HORIZONAL = 1,       // horizotal
	ORIENT_VERTICAL = 2,        // vertical


}ORIENT_CODE;

static void sobel_edge(ORIENT_CODE oriention, image_int8u_p src, image_int16s_p dst)
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

				dst->data[_t] = -((short)psrc[_tp - 1] + (((short)psrc[_tp]) << 1) + (short)psrc[_tp + 1])
					+ ((short)psrc[_td - 1] + (((short)psrc[_td]) << 1) + (short)psrc[_td + 1]);
			}
		}
		break;

	default:
		printf("sobel oriention is wrong!");
		break;
	}

	psrc = NULL;

}


static void mcv_sobel(ORIENT_CODE oriention, image_int8u_p src, image_int16s_p dst)
{
	sobel_edge(oriention, src, dst);
}

static void array_abs(image_int16s_p src, image_int16s_p dst)
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


static void mcv_abs(image_int16s_p src, image_int16s_p dst)
{
	array_abs(src, dst);
}


static int array_add(image_float_p src1, image_float_p src2, image_float_p dst)
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

static void array_add(image_int16s_p src1, image_int16s_p src2, image_int16s_p dst)
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

static void mcv_add(image_int16s_p src1, image_int16s_p src2, image_int16s_p dst)
{
	array_add(src1, src2, dst);
}

static void mcv_add(image_float_p src1, image_float_p src2, image_float_p dst)
{
	array_add(src1, src2, dst);
}


static image_int16s_p array_threshold(short thresh, image_int16s_p src)
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


static void array_threshold(short thresh, image_int16s_p src, image_int16s_p dst)
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


static void mcv_threshold(short thresh, image_int16s_p src, image_int16s_p dst)
{
	array_threshold(thresh, src, dst);

}


static void array_compare_lt(image_int16s_p src1, image_int16s_p src2, image_int8u_p dst)
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

	psrc1 = psrc2 = NULL;
	pdst = NULL;

}

static void mcv_compare_CMP_LT(image_int16s_p src1, image_int16s_p src2, image_int8u_p dst)
{

	array_compare_lt(src1, src2, dst);

}


static image_int16s_p array_devide(int n, image_int16s_p src)
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

static void array_devide(int n, image_int16s_p src, image_int16s_p dst)
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


static void mcv_mat_divide(int n, image_int16s_p src, image_int16s_p dst)
{
	array_devide(n, src, dst);
}



//A*AT
static int array_multiply_transpose_float(image_float_p src, image_float_p dst)
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
static int array_multiply2_transpose_float(image_float_p src1, image_float_p src2, image_float_p dst)
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


static int array_multiply(image_float_p src1, image_float_p src2, image_float_p dst)
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


static void mcv_multiply_float(image_float_p src1, image_float_p src2, image_float_p dst)
{
	array_multiply(src1, src2, dst);
}


static void mcv_multiply_transpose_float(image_float_p src, image_float_p dst)
{
	array_multiply_transpose_float(src, dst);
}



static void mcv_multiply2_transpose_float(image_float_p src1, image_float_p src2, image_float_p dst)
{
	array_multiply2_transpose_float(src1, src2, dst);

}


int EDLineDetector::EdgeDrawing(image_int8u_p image, EdgeChains &edgeChains, bool smoothed)
{
	imageWidth = image->xsize;
	imageHeight = image->ysize;

	unsigned int pixelNum = imageWidth*imageHeight;
	unsigned int edgePixelArraySize = pixelNum / 5;
	unsigned int maxNumOfEdge = edgePixelArraySize / 20;
	//compute dx, dy images
	if ((gImg_ == NULL) || ((gImg_->xsize != (int)imageWidth) || (gImg_->ysize != (int)imageHeight))){
		if (pFirstPartEdgeX_ != NULL){
			delete[] pFirstPartEdgeX_; pFirstPartEdgeX_ = NULL;
			delete[] pFirstPartEdgeY_; pFirstPartEdgeY_ = NULL;
			delete[] pSecondPartEdgeX_; pSecondPartEdgeX_ = NULL;
			delete[] pSecondPartEdgeY_; pSecondPartEdgeY_ = NULL;
			delete[] pFirstPartEdgeS_; pFirstPartEdgeS_ = NULL;
			delete[] pSecondPartEdgeS_; pSecondPartEdgeS_ = NULL;
			delete[] pAnchorX_; pAnchorX_ = NULL;
			delete[] pAnchorY_; pAnchorY_ = NULL;
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
	memset(pAnchorX_, 0, edgePixelArraySize*sizeof(unsigned int));//initialization
	memset(pAnchorY_, 0, edgePixelArraySize*sizeof(unsigned int));
	unsigned int anchorsSize = 0;
	int offy;
	int indexInArray;
	unsigned char gValue1, gValue2, gValue3;
	for (unsigned int h = 1; h<imageHeight - 1; h = h + scanIntervals_){
		offy = h*imageWidth;
		for (unsigned int w = 1; w<imageWidth - 1; w = w + scanIntervals_){
			indexInArray = offy + w;
			if (pdirImg[indexInArray] == Horizontal){//if the direction of pixel is horizontal, then compare with up and down
				if (pgImg[indexInArray] >= pgImg[indexInArray - imageWidth] + anchorThreshold_
					&&pgImg[indexInArray] >= pgImg[indexInArray + imageWidth] + anchorThreshold_){// (w,h) is accepted as an anchor
					pAnchorX_[anchorsSize] = w;
					pAnchorY_[anchorsSize++] = h;
				}
			}
			else{
				if (pgImg[indexInArray] >= pgImg[indexInArray - 1] + anchorThreshold_
					&&pgImg[indexInArray] >= pgImg[indexInArray + 1] + anchorThreshold_){// (w,h) is accepted as an anchor
					pAnchorX_[anchorsSize] = w;
					pAnchorY_[anchorsSize++] = h;
				}
			}
		}
	}
	if (anchorsSize>edgePixelArraySize){
		printf("anchor size is larger than its maximal size. anchorsSize = %d, maximal size = %d\n",
			anchorsSize, edgePixelArraySize);
		return 1;
	}

	//link the anchors by smart routing
	memset(edgeImage_->data, ZERO, edgeImage_->xsize*edgeImage_->ysize*sizeof(unsigned char));
	unsigned char *pEdgeImg = edgeImage_->data;
	memset(pFirstPartEdgeX_, 0, edgePixelArraySize*sizeof(unsigned int));//initialization
	memset(pFirstPartEdgeY_, 0, edgePixelArraySize*sizeof(unsigned int));
	memset(pSecondPartEdgeX_, 0, edgePixelArraySize*sizeof(unsigned int));
	memset(pSecondPartEdgeY_, 0, edgePixelArraySize*sizeof(unsigned int));
	memset(pFirstPartEdgeS_, 0, maxNumOfEdge*sizeof(unsigned int));
	memset(pSecondPartEdgeS_, 0, maxNumOfEdge*sizeof(unsigned int));
	unsigned int offsetPFirst = 0, offsetPSecond = 0;
	unsigned int offsetPS = 0;

	unsigned int x, y;
	unsigned int lastX, lastY;
	unsigned char lastDirection;//up = 1, right = 2, down = 3, left = 4;
	unsigned char shouldGoDirection;//up = 1, right = 2, down = 3, left = 4;
	int edgeLenFirst, edgeLenSecond;

	lastX = lastY = 0;

	for (unsigned int i = 0; i<anchorsSize; i++){
		x = pAnchorX_[i];
		y = pAnchorY_[i];
		indexInArray = y*imageWidth + x;
		if (pEdgeImg[indexInArray]){//if anchor i is already been an edge pixel.
			continue;
		}

		pFirstPartEdgeS_[offsetPS] = offsetPFirst;
		if (pdirImg[indexInArray] == Horizontal){//if the direction of this pixel is horizontal, then go left and right.
			//fist go right, pixel direction may be different during linking.
			lastDirection = RightDir;
			while (pgImg[indexInArray]>0 && !pEdgeImg[indexInArray]){
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pFirstPartEdgeX_[offsetPFirst] = x;
				pFirstPartEdgeY_[offsetPFirst++] = y;
				shouldGoDirection = 0;//unknown
				if (pdirImg[indexInArray] == Horizontal){//should go left or right 
					if (lastDirection == UpDir || lastDirection == DownDir){//change the pixel direction now
						if (x>lastX){//should go right
							shouldGoDirection = RightDir;
						}
						else{//should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == RightDir || shouldGoDirection == RightDir){//go right
						if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-right
							x = x + 1;
							y = y + 1;
						}
						else{//straight-right
							x = x + 1;
						}
						lastDirection = RightDir;
					}
					else if (lastDirection == LeftDir || shouldGoDirection == LeftDir){//go left
						if (x == 0 || y == 0 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						gValue2 = (unsigned char)pgImg[indexInArray - 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-left
							x = x - 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-left
							x = x - 1;
							y = y + 1;
						}
						else{//straight-left
							x = x - 1;
						}
						lastDirection = LeftDir;
					}
				}
				else{//should go up or down.
					if (lastDirection == RightDir || lastDirection == LeftDir){//change the pixel direction now
						if (y>lastY){//should go down
							shouldGoDirection = DownDir;
						}
						else{//should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == DownDir || shouldGoDirection == DownDir){//go down
						if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//down-right
							x = x + 1;
							y = y + 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-left
							x = x - 1;
							y = y + 1;
						}
						else{//straight-down
							y = y + 1;
						}
						lastDirection = DownDir;
					}
					else if (lastDirection == UpDir || shouldGoDirection == UpDir){//go up
						if (x == 0 || x == imageWidth - 1 || y == 0){//reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//up-left
							x = x - 1;
							y = y - 1;
						}
						else{//straight-up
							y = y - 1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y*imageWidth + x;
			}//end while go right
			//then go left, pixel direction may be different during linking.
			x = pAnchorX_[i];
			y = pAnchorY_[i];
			indexInArray = y*imageWidth + x;
			pEdgeImg[indexInArray] = 0;//mark the anchor point be a non-edge pixel and
			lastDirection = LeftDir;
			pSecondPartEdgeS_[offsetPS] = offsetPSecond;
			while (pgImg[indexInArray]>0 && !pEdgeImg[indexInArray]){
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pSecondPartEdgeX_[offsetPSecond] = x;
				pSecondPartEdgeY_[offsetPSecond++] = y;
				shouldGoDirection = 0;//unknown
				if (pdirImg[indexInArray] == Horizontal){//should go left or right
					if (lastDirection == UpDir || lastDirection == DownDir){//change the pixel direction now
						if (x>lastX){//should go right
							shouldGoDirection = RightDir;
						}
						else{//should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == RightDir || shouldGoDirection == RightDir){//go right
						if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-right
							x = x + 1;
							y = y + 1;
						}
						else{//straight-right
							x = x + 1;
						}
						lastDirection = RightDir;
					}
					else	if (lastDirection == LeftDir || shouldGoDirection == LeftDir){//go left
						if (x == 0 || y == 0 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						gValue2 = (unsigned char)pgImg[indexInArray - 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-left
							x = x - 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-left
							x = x - 1;
							y = y + 1;
						}
						else{//straight-left
							x = x - 1;
						}
						lastDirection = LeftDir;
					}
				}
				else{//should go up or down.
					if (lastDirection == RightDir || lastDirection == LeftDir){//change the pixel direction now
						if (y>lastY){//should go down
							shouldGoDirection = DownDir;
						}
						else{//should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == DownDir || shouldGoDirection == DownDir){//go down
						if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//down-right
							x = x + 1;
							y = y + 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-left
							x = x - 1;
							y = y + 1;
						}
						else{//straight-down
							y = y + 1;
						}
						lastDirection = DownDir;
					}
					else	if (lastDirection == UpDir || shouldGoDirection == UpDir){//go up
						if (x == 0 || x == imageWidth - 1 || y == 0){//reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//up-left
							x = x - 1;
							y = y - 1;
						}
						else{//straight-up
							y = y - 1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y*imageWidth + x;
			}//end while go left
			//end anchor is Horizontal
		}
		else{//the direction of this pixel is vertical, go up and down
			//fist go down, pixel direction may be different during linking.
			lastDirection = DownDir;
			while (pgImg[indexInArray]>0 && !pEdgeImg[indexInArray]){
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pFirstPartEdgeX_[offsetPFirst] = x;
				pFirstPartEdgeY_[offsetPFirst++] = y;
				shouldGoDirection = 0;//unknown
				if (pdirImg[indexInArray] == Horizontal){//should go left or right
					if (lastDirection == UpDir || lastDirection == DownDir){//change the pixel direction now
						if (x>lastX){//should go right
							shouldGoDirection = RightDir;
						}
						else{//should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == RightDir || shouldGoDirection == RightDir){//go right
						if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-right
							x = x + 1;
							y = y + 1;
						}
						else{//straight-right
							x = x + 1;
						}
						lastDirection = RightDir;
					}
					else	if (lastDirection == LeftDir || shouldGoDirection == LeftDir){//go left
						if (x == 0 || y == 0 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						gValue2 = (unsigned char)pgImg[indexInArray - 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-left
							x = x - 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-left
							x = x - 1;
							y = y + 1;
						}
						else{//straight-left
							x = x - 1;
						}
						lastDirection = LeftDir;
					}
				}
				else{//should go up or down.
					if (lastDirection == RightDir || lastDirection == LeftDir){//change the pixel direction now
						if (y>lastY){//should go down
							shouldGoDirection = DownDir;
						}
						else{//should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == DownDir || shouldGoDirection == DownDir){//go down
						if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//down-right
							x = x + 1;
							y = y + 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-left
							x = x - 1;
							y = y + 1;
						}
						else{//straight-down
							y = y + 1;
						}
						lastDirection = DownDir;
					}
					else	if (lastDirection == UpDir || shouldGoDirection == UpDir){//go up
						if (x == 0 || x == imageWidth - 1 || y == 0){//reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//up-left
							x = x - 1;
							y = y - 1;
						}
						else{//straight-up
							y = y - 1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y*imageWidth + x;
			}//end while go down
			//then go up, pixel direction may be different during linking.
			lastDirection = UpDir;
			x = pAnchorX_[i];
			y = pAnchorY_[i];
			indexInArray = y*imageWidth + x;
			pEdgeImg[indexInArray] = 0;//mark the anchor point be a non-edge pixel and
			pSecondPartEdgeS_[offsetPS] = offsetPSecond;
			while (pgImg[indexInArray]>0 && !pEdgeImg[indexInArray]){
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pSecondPartEdgeX_[offsetPSecond] = x;
				pSecondPartEdgeY_[offsetPSecond++] = y;
				shouldGoDirection = 0;//unknown
				if (pdirImg[indexInArray] == Horizontal){//should go left or right
					if (lastDirection == UpDir || lastDirection == DownDir){//change the pixel direction now
						if (x>lastX){//should go right
							shouldGoDirection = RightDir;
						}
						else{//should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == RightDir || shouldGoDirection == RightDir){//go right
						if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-right
							x = x + 1;
							y = y + 1;
						}
						else{//straight-right
							x = x + 1;
						}
						lastDirection = RightDir;
					}
					else	if (lastDirection == LeftDir || shouldGoDirection == LeftDir){//go left
						if (x == 0 || y == 0 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						gValue2 = (unsigned char)pgImg[indexInArray - 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-left
							x = x - 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-left
							x = x - 1;
							y = y + 1;
						}
						else{//straight-left
							x = x - 1;
						}
						lastDirection = LeftDir;
					}
				}
				else{//should go up or down.
					if (lastDirection == RightDir || lastDirection == LeftDir){//change the pixel direction now
						if (y>lastY){//should go down
							shouldGoDirection = DownDir;
						}
						else{//should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == DownDir || shouldGoDirection == DownDir){//go down
						if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1){//reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//down-right
							x = x + 1;
							y = y + 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//down-left
							x = x - 1;
							y = y + 1;
						}
						else{//straight-down
							y = y + 1;
						}
						lastDirection = DownDir;
					}
					else	if (lastDirection == UpDir || shouldGoDirection == UpDir){//go up
						if (x == 0 || x == imageWidth - 1 || y == 0){//reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if (gValue1 >= gValue2&&gValue1 >= gValue3){//up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2&&gValue3 >= gValue1){//up-left
							x = x - 1;
							y = y - 1;
						}
						else{//straight-up
							y = y - 1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y*imageWidth + x;
			}//end while go up
		}//end anchor is Vertical
		//only keep the edge chains whose length is larger than the minLineLen_;
		edgeLenFirst = offsetPFirst - pFirstPartEdgeS_[offsetPS];
		edgeLenSecond = offsetPSecond - pSecondPartEdgeS_[offsetPS];
		if (edgeLenFirst + edgeLenSecond<minLineLen_ + 1){//short edge, drop it
			offsetPFirst = pFirstPartEdgeS_[offsetPS];
			offsetPSecond = pSecondPartEdgeS_[offsetPS];
		}
		else{
			offsetPS++;
		}
	}
	//store the last index
	pFirstPartEdgeS_[offsetPS] = offsetPFirst;
	pSecondPartEdgeS_[offsetPS] = offsetPSecond;
	if (offsetPS>maxNumOfEdge){
		printf("Edge drawing Error: The total number of edges is larger than MaxNumOfEdge, "
			"numofedge = %d, MaxNumOfEdge = %d\n", offsetPS, maxNumOfEdge);
		return 1;
	}
	if (offsetPFirst>edgePixelArraySize || offsetPSecond>edgePixelArraySize){
		printf("Edge drawing Error: The total number of edge pixels is larger than MaxNumOfEdgePixels, "
			"numofedgePixel1 = &d,  numofedgePixel2 = %d, MaxNumOfEdgePixel = %d\n", offsetPFirst, offsetPSecond, edgePixelArraySize);
		return 1;
	}

	int tempID;
	edgeChains.xCors.resize(offsetPFirst + offsetPSecond);
	edgeChains.yCors.resize(offsetPFirst + offsetPSecond);
	edgeChains.sId.resize(offsetPS + 1);
	unsigned int *pxCors = edgeChains.xCors.data();
	unsigned int *pyCors = edgeChains.yCors.data();
	unsigned int *psId = edgeChains.sId.data();
	offsetPFirst = 0;
	offsetPSecond = 0;
	unsigned int indexInCors = 0;
	unsigned int numOfEdges = 0;
	for (unsigned int edgeId = 0; edgeId<offsetPS; edgeId++){

		psId[numOfEdges++] = indexInCors;
		indexInArray = pFirstPartEdgeS_[edgeId];
		offsetPFirst = pFirstPartEdgeS_[edgeId + 1];
		for (tempID = offsetPFirst - 1; tempID >= indexInArray; tempID--){//add first part edge
			pxCors[indexInCors] = pFirstPartEdgeX_[tempID];
			pyCors[indexInCors++] = pFirstPartEdgeY_[tempID];
		}
		indexInArray = pSecondPartEdgeS_[edgeId];
		offsetPSecond = pSecondPartEdgeS_[edgeId + 1];
		for (tempID = indexInArray + 1; tempID<(int)offsetPSecond; tempID++){//add second part edge
			pxCors[indexInCors] = pSecondPartEdgeX_[tempID];
			pyCors[indexInCors++] = pSecondPartEdgeY_[tempID];
		}
	}
	psId[numOfEdges] = indexInCors;//the end index of the last edge
	edgeChains.numOfEdges = numOfEdges;


	return 0;
}


int EDLineDetector::EDline(image_int8u_p image, LineChains &lines, bool smoothed)
{
	//first, call EdgeDrawing function to extract edges
	EdgeChains edges;

	if (EdgeDrawing(image, edges, smoothed)){
		printf("Line Detection not finished\n");
		return 1;
	}

	if (edges.numOfEdges == ZERO)
		return 0;

	//detect lines
	unsigned int linePixelID = edges.sId[edges.numOfEdges];
	lines.xCors.resize(linePixelID);
	lines.yCors.resize(linePixelID);
	lines.sId.resize(5 * edges.numOfEdges);
	unsigned int *pEdgeXCors = edges.xCors.data();
	unsigned int *pEdgeYCors = edges.yCors.data();
	unsigned int *pEdgeSID = edges.sId.data();
	unsigned int *pLineXCors = lines.xCors.data();
	unsigned int *pLineYCors = lines.yCors.data();
	unsigned int *pLineSID = lines.sId.data();
	logNT_ = 2.f * (log10((float)imageWidth) + log10((float)imageHeight));
	float lineFitErr;//the line fit error;
	std::array<float, 2> lineEquation;
	lineEquations_.clear();
	lineEndpoints_.clear();
	lineDirection_.clear();
	unsigned char *pdirImg = dirImg_->data;
	unsigned int numOfLines = 0;
	unsigned int offsetInEdgeArrayS, offsetInEdgeArrayE, newOffsetS;//start index and end index
	unsigned int offsetInLineArray = 0;
	float direction;//line direction

	newOffsetS = 0;
	lineFitErr = 0.f;

	for (unsigned int edgeID = 0; edgeID<edges.numOfEdges; edgeID++)
	{
		offsetInEdgeArrayS = pEdgeSID[edgeID];
		offsetInEdgeArrayE = pEdgeSID[edgeID + 1];
		while (offsetInEdgeArrayE > offsetInEdgeArrayS + minLineLen_)
		{//extract line segments from an edge, may find more than one segments
			//find an initial line segment
			while (offsetInEdgeArrayE > offsetInEdgeArrayS + minLineLen_){
				lineFitErr = LeastSquaresLineFit_(pEdgeXCors, pEdgeYCors,
					offsetInEdgeArrayS, lineEquation);
				if (lineFitErr <= lineFitErrThreshold_) break;//ok, an initial line segment detected
				offsetInEdgeArrayS += SkipEdgePoint; //skip the first two pixel in the chain and try with the remaining pixels
			}
			if (lineFitErr>lineFitErrThreshold_) break; //no line is detected
			//An initial line segment is detected. Try to extend this line segment
			pLineSID[numOfLines] = offsetInLineArray;
			float coef1 = 0.f;//for a line ax+by+c=0, coef1 = 1/sqrt(a^2+b^2);
			float pointToLineDis;//for a line ax+by+c=0 and a point(xi, yi), pointToLineDis = coef1*|a*xi+b*yi+c|
			bool bExtended = true;
			bool bFirstTry = true;
			int numOfOutlier;//to against noise, we accept a few outlier of a line.
			int tryTimes = 0;

			if (pdirImg[pEdgeYCors[offsetInEdgeArrayS] * imageWidth + pEdgeXCors[offsetInEdgeArrayS]] == Horizontal)
			{//y=ax+b, i.e. ax-y+b=0
				while (bExtended)
				{
					tryTimes++;
					if (bFirstTry)
					{
						bFirstTry = false;
						for (int i = 0; i<minLineLen_; i++)
						{//First add the initial line segment to the line array
							pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
							pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						}
					}
					else
					{//after each try, line is extended, line equation should be re-estimated
						//adjust the line equation
						lineFitErr = LeastSquaresLineFit_(pLineXCors, pLineYCors, pLineSID[numOfLines],
							newOffsetS, offsetInLineArray, lineEquation);
					}
					coef1 = 1 / sqrt(lineEquation[0] * lineEquation[0] + 1);
					numOfOutlier = 0;
					newOffsetS = offsetInLineArray;
					while (offsetInEdgeArrayE > offsetInEdgeArrayS)
					{
						pointToLineDis = fabs(lineEquation[0] * pEdgeXCors[offsetInEdgeArrayS] -
							pEdgeYCors[offsetInEdgeArrayS] + lineEquation[1])*coef1;
						pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
						pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						if (pointToLineDis>lineFitErrThreshold_)
						{
							numOfOutlier++;
							if (numOfOutlier>3) break;
						}
						else{//we count number of connective outliers.
							numOfOutlier = 0;
						}
					}
					//pop back the last few outliers from lines and return them to edge chain
					offsetInLineArray -= numOfOutlier;
					offsetInEdgeArrayS -= numOfOutlier;
					if (offsetInLineArray - newOffsetS>0 && tryTimes<TryTime)
					{//some new pixels are added to the line
					}
					else{
						bExtended = false;//no new pixels are added.
					}
				}
				//the line equation coefficients,for line w1x+w2y+w3 =0, we normalize it to make w1^2+w2^2 = 1.
				std::array<float, 3> lineEqu = { lineEquation[0] * coef1, -1 * coef1, lineEquation[1] * coef1 };
				if (LineValidation_(pLineXCors, pLineYCors, pLineSID[numOfLines], offsetInLineArray, lineEqu, direction))
				{//check the line
					//store the line equation coefficients
					lineEquations_.push_back(lineEqu);

					std::array<float, 4> lineEndP;//line endpoints
					float a1 = lineEqu[1] * lineEqu[1];
					float a2 = lineEqu[0] * lineEqu[0];
					float a3 = lineEqu[0] * lineEqu[1];
					float a4 = lineEqu[2] * lineEqu[0];
					float a5 = lineEqu[2] * lineEqu[1];
					unsigned int Px = pLineXCors[pLineSID[numOfLines]];//first pixel
					unsigned int Py = pLineYCors[pLineSID[numOfLines]];
					lineEndP[0] = a1*Px - a3*Py - a4;//x
					lineEndP[1] = a2*Py - a3*Px - a5;//y
					Px = pLineXCors[offsetInLineArray - 1];//last pixel
					Py = pLineYCors[offsetInLineArray - 1];
					lineEndP[2] = a1*Px - a3*Py - a4;//x
					lineEndP[3] = a2*Py - a3*Px - a5;//y
					lineEndpoints_.push_back(lineEndP);
					lineDirection_.push_back(direction);
					numOfLines++;
				}
				else{
					offsetInLineArray = pLineSID[numOfLines];// line was not accepted, the offset is set back
				}
			}
			else{//x=ay+b, i.e. x-ay-b=0
				while (bExtended)
				{
					tryTimes++;
					if (bFirstTry)
					{
						bFirstTry = false;
						for (int i = 0; i<minLineLen_; i++)
						{//First add the initial line segment to the line array
							pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
							pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						}
					}
					else{//after each try, line is extended, line equation should be re-estimated
						//adjust the line equation
						lineFitErr = LeastSquaresLineFit_(pLineXCors, pLineYCors, pLineSID[numOfLines],
							newOffsetS, offsetInLineArray, lineEquation);
					}
					coef1 = 1 / sqrt(1 + lineEquation[0] * lineEquation[0]);
					numOfOutlier = 0;
					newOffsetS = offsetInLineArray;
					while (offsetInEdgeArrayE > offsetInEdgeArrayS)
					{
						pointToLineDis = fabs(pEdgeXCors[offsetInEdgeArrayS] -
							lineEquation[0] * pEdgeYCors[offsetInEdgeArrayS] - lineEquation[1])*coef1;
						pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
						pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						if (pointToLineDis>lineFitErrThreshold_)
						{
							numOfOutlier++;
							if (numOfOutlier>3) break;
						}
						else{//we count number of connective outliers.
							numOfOutlier = 0;
						}
					}
					//pop back the last few outliers from lines and return them to edge chain
					offsetInLineArray -= numOfOutlier;
					offsetInEdgeArrayS -= numOfOutlier;
					if (offsetInLineArray - newOffsetS>0 && tryTimes<TryTime)
					{//some new pixels are added to the line
					}
					else{
						bExtended = false;//no new pixels are added.
					}
				}
				//the line equation coefficients,for line w1x+w2y+w3 =0, we normalize it to make w1^2+w2^2 = 1.
				std::array<float, 3> lineEqu = { 1 * coef1, -lineEquation[0] * coef1, -lineEquation[1] * coef1 };
				if (LineValidation_(pLineXCors, pLineYCors, pLineSID[numOfLines], offsetInLineArray, lineEqu, direction))
				{//check the line
					//store the line equation coefficients
					lineEquations_.push_back(lineEqu);

					std::array<float, 4> lineEndP;//line endpoints
					float a1 = lineEqu[1] * lineEqu[1];
					float a2 = lineEqu[0] * lineEqu[0];
					float a3 = lineEqu[0] * lineEqu[1];
					float a4 = lineEqu[2] * lineEqu[0];
					float a5 = lineEqu[2] * lineEqu[1];
					unsigned int Px = pLineXCors[pLineSID[numOfLines]];//first pixel
					unsigned int Py = pLineYCors[pLineSID[numOfLines]];
					lineEndP[0] = a1*Px - a3*Py - a4;//x
					lineEndP[1] = a2*Py - a3*Px - a5;//y
					Px = pLineXCors[offsetInLineArray - 1];//last pixel
					Py = pLineYCors[offsetInLineArray - 1];
					lineEndP[2] = a1*Px - a3*Py - a4;//x
					lineEndP[3] = a2*Py - a3*Px - a5;//y
					lineEndpoints_.push_back(lineEndP);
					lineDirection_.push_back(direction);
					numOfLines++;
				}
				else
				{
					offsetInLineArray = pLineSID[numOfLines];// line was not accepted, the offset is set back
				}
			}
			//Extract line segments from the remaining pixel; Current chain has been shortened already.
		}
	}//end for(unsigned int edgeID=0; edgeID<edges.numOfEdges; edgeID++)
	pLineSID[numOfLines] = offsetInLineArray;
	lines.numOfLines = numOfLines;


	return 0;
}


float EDLineDetector::LeastSquaresLineFit_(unsigned int *xCors, unsigned int *yCors,
	unsigned int offsetS, std::array<float, 2> &lineEquation)
{
	if (lineEquation.size() != 2){
		printf("SHOULD NOT BE != 2\n");
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
	if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Horizontal){
		/*Build the system,and solve it using least square regression: mat * [a,b]^T = vec */
		pMatT = fitMatT->data;//fitMatT = [x0, x1, ... xn; 1,1,...,1];

		for (int i = 0; i<minLineLen_; i++){
			*(pMatT++) = (float)xCors[offsetS];
			fitVec->data[0 * fitVec->xsize + i] = (float)yCors[offsetS++];
		}

		mcv_multiply_transpose_float(fitMatT, ATA);
		mcv_multiply2_transpose_float(fitMatT, fitVec, ATV);

		/* [a,b]^T = Inv(mat^T * mat) * mat^T * vec */
		pATA = ATA->data;
		coef = 1.f / (float(pATA[0])*float(pATA[3]) - float(pATA[1])*float(pATA[2]));
		//		lineEquation = svd.Invert(ATA) * matT * vec;

		lineEquation[0] = coef *(float(pATA[3])*float(ATV->data[0 * ATV->xsize + 0])
			- float(pATA[1])*float(ATV->data[0 * ATV->xsize + 1]));
		lineEquation[1] = coef *(float(pATA[0])*float(ATV->data[0 * ATV->xsize + 1])
			- float(pATA[2])*float(ATV->data[0 * ATV->xsize + 0]));

		/*compute line fit error */
		for (int i = 0; i<minLineLen_; i++){
			coef = float(yCors[offset]) - float(xCors[offset++]) * lineEquation[0] - lineEquation[1];
			fitError += coef*coef;
		}
		return sqrt(fitError);
	}
	/*If the first pixel in this chain is vertical,
	*then we try to find a vertical line, x=ay+b;*/
	if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Vertical){
		/*Build the system,and solve it using least square regression: mat * [a,b]^T = vec */

		pMatT = fitMatT->data;//fitMatT = [y0, y1, ... yn; 1,1,...,1];

		for (int i = 0; i<minLineLen_; i++){

			*(pMatT++) = (float)yCors[offsetS];
			fitVec->data[0 * fitVec->xsize + i] = (float)xCors[offsetS++];

		}

		mcv_multiply_transpose_float(fitMatT, ATA);
		mcv_multiply2_transpose_float(fitMatT, fitVec, ATV);

		/* [a,b]^T = Inv(mat^T * mat) * mat^T * vec */
		pATA = ATA->data;
		coef = 1.f / (float(pATA[0])*float(pATA[3]) - float(pATA[1])*float(pATA[2]));

		lineEquation[0] = coef *(float(pATA[3])*float(ATV->data[0 * ATV->xsize + 0])
			- float(pATA[1])*float(ATV->data[0 * ATV->xsize + 1]));
		lineEquation[1] = coef *(float(pATA[0])*float(ATV->data[0 * ATV->xsize + 1])
			- float(pATA[2])*float(ATV->data[0 * ATV->xsize + 0]));


		/*compute line fit error */
		for (int i = 0; i<minLineLen_; i++){
			coef = float(xCors[offset]) - float(yCors[offset++]) * lineEquation[0] - lineEquation[1];
			fitError += coef*coef;
		}
		return sqrt(fitError);
	}
	return 0.f;
}
float EDLineDetector::LeastSquaresLineFit_(unsigned int *xCors, unsigned int *yCors,
	unsigned int offsetS, unsigned int newOffsetS,
	unsigned int offsetE, std::array<float, 2> &lineEquation)
{
	int length = offsetE - offsetS;
	int newLength = offsetE - newOffsetS;
	if (length <= 0 || newLength <= 0){
		printf("EDLineDetector::LeastSquaresLineFit_ Error:"
			" the expected line index is wrong...offsetE = %d, offsetS = %d, newOffsetS = %d\n",
			offsetE, offsetS, newOffsetS);
		return 1;
	}
	if (lineEquation.size() != 2){
		printf("SHOULD NOT BE != 2\n");
		return 1;
	}

	image_float_p matT = new_image_float(newLength, 2);
	image_float_p vec = new_image_float(1, newLength);
	float * pMatT = NULL;
	float * pATA = NULL;
	float coef;
	unsigned char *pdirImg = dirImg_->data;
	/*If the first pixel in this chain is horizontal,
	*then we try to find a horizontal line, y=ax+b;*/
	if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Horizontal){
		/*Build the new system,and solve it using least square regression: mat * [a,b]^T = vec */

		pMatT = matT->data;//matT = [x0', x1', ... xn'; 1,1,...,1]
		for (int i = 0; i<newLength; i++){
			*(pMatT + newLength) = 1;
			*(pMatT++) = (float)xCors[newOffsetS];
			vec->data[0 * vec->xsize + i] = (float)yCors[newOffsetS++];
		}
		/* [a,b]^T = Inv(ATA + mat^T * mat) * (ATV + mat^T * vec) */
		mcv_multiply_transpose_float(matT, tempMatLineFit);
		mcv_multiply_float(matT, vec, tempVecLineFit);
		mcv_add(ATA, tempMatLineFit, ATA);
		mcv_add(ATV, tempVecLineFit, ATV);


		pATA = ATA->data;
		coef = 1.f / (float(pATA[0])*float(pATA[3]) - float(pATA[1])*float(pATA[2]));

		lineEquation[0] = coef *(float(pATA[3])*float(ATV->data[0 * ATV->xsize + 0])
			- float(pATA[1])*float(ATV->data[0 * ATV->xsize + 1]));
		lineEquation[1] = coef *(float(pATA[0])*float(ATV->data[0 * ATV->xsize + 1])
			- float(pATA[2])*float(ATV->data[0 * ATV->xsize + 0]));

		goto _END;
	}
	/*If the first pixel in this chain is vertical,
	*then we try to find a vertical line, x=ay+b;*/
	if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Vertical){
		/*Build the system,and solve it using least square regression: mat * [a,b]^T = vec */

		pMatT = matT->data;//matT = [y0', y1', ... yn'; 1,1,...,1]
		for (int i = 0; i<newLength; i++){
			*(pMatT + newLength) = 1;
			*(pMatT++) = (float)yCors[newOffsetS];
			vec->data[0 * vec->xsize + i] = (float)xCors[newOffsetS++];

		}
		/* [a,b]^T = Inv(ATA + mat^T * mat) * (ATV + mat^T * vec) */
		mcv_multiply_transpose_float(matT, tempMatLineFit);
		mcv_multiply_float(matT, vec, tempVecLineFit);
		mcv_add(ATA, tempMatLineFit, ATA);
		mcv_add(ATV, tempVecLineFit, ATV);

		pATA = ATA->data;
		coef = 1.f / (float(pATA[0])*float(pATA[3]) - float(pATA[1])*float(pATA[2]));

		lineEquation[0] = coef *(float(pATA[3])*float(ATV->data[0 * ATV->xsize + 0])
			- float(pATA[1])*float(ATV->data[0 * ATV->xsize + 1]));
		lineEquation[1] = coef *(float(pATA[0])*float(ATV->data[0 * ATV->xsize + 1])
			- float(pATA[2])*float(ATV->data[0 * ATV->xsize + 0]));

	}

_END:

	free_image_float(matT);
	free_image_float(vec);
	return 0.f;
}

bool EDLineDetector::LineValidation_(unsigned int *xCors, unsigned int *yCors,
	unsigned int offsetS, unsigned int offsetE,
	std::array<float, 3> &lineEquation, float &direction)
{
	if (bValidate_){
		int n = offsetE - offsetS;
		/*first compute the direction of line, make sure that the dark side always be the
		*left side of a line.*/
		int meanGradientX = 0, meanGradientY = 0;
		short *pdxImg = dxImg_->data;
		short *pdyImg = dyImg_->data;
		float dx, dy;
		std::vector<float> pointDirection;
		int index;
		for (int i = 0; i<n; i++){
			index = yCors[offsetS] * imageWidth + xCors[offsetS++];
			meanGradientX += pdxImg[index];
			meanGradientY += pdyImg[index];
			dx = (float)pdxImg[index];
			dy = (float)pdyImg[index];
			pointDirection.push_back(atan2(-dx, dy));
		}
		dx = fabs(lineEquation[1]);
		dy = fabs(lineEquation[0]);
		if (meanGradientX == 0 && meanGradientY == 0){//not possible, if happens, it must be a wrong line,
			return false;
		}
		if (meanGradientX>0 && meanGradientY >= 0){//first quadrant, and positive direction of X axis.
			direction = atan2(-dy, dx);//line direction is in fourth quadrant
		}
		if (meanGradientX <= 0 && meanGradientY>0){//second quadrant, and positive direction of Y axis.
			direction = atan2(dy, dx);//line direction is in first quadrant
		}
		if (meanGradientX<0 && meanGradientY <= 0){//third quadrant, and negative direction of X axis.
			direction = atan2(dy, -dx);//line direction is in second quadrant
		}
		if (meanGradientX >= 0 && meanGradientY<0){//fourth quadrant, and negative direction of Y axis.
			direction = atan2(-dy, -dx);//line direction is in third quadrant
		}
		/*then check whether the line is on the border of the image. We don't keep the border line.*/
		if (fabs(direction)<0.15 || PI - fabs(direction)<0.15f){//Horizontal line
			if (fabs(lineEquation[2])<10 || fabs(imageHeight - fabs(lineEquation[2]))<10){//upper border or lower border
				return false;
			}
		}
		if (fabs(fabs(direction) - PI*0.5f)<0.15f){//Vertical line
			if (fabs(lineEquation[2])<10 || fabs(imageWidth - fabs(lineEquation[2]))<10){//left border or right border
				return false;
			}
		}
		//count the aligned points on the line which have the same direction as the line.
		float disDirection;
		int k = 0;
		for (int i = 0; i<n; i++){
			disDirection = fabs(direction - pointDirection[i]);
			if (fabs(2 * PI - disDirection)<0.392699f || disDirection<0.392699f){//same direction, pi/8 = 0.392699081698724
				k++;
			}
		}
		//now compute NFA(Number of False Alarms)
		float ret = nfa(n, k, 0.125f, logNT_);

		return (ret>0); //0 corresponds to 1 mean false alarm
	}
	else{
		return true;
	}
}


int EDLineDetector::EDline(image_int8u_p image, bool smoothed)
{
	if (EDline(image, lines_, smoothed))
		return 1;

	lineSalience_.clear();
	lineSalience_.resize(lines_.numOfLines);

	short *pgImg = gImgWO_->data;

	unsigned int indexInLineArray;
	unsigned int *pXCor = lines_.xCors.data();
	unsigned int *pYCor = lines_.yCors.data();
	unsigned int *pSID = lines_.sId.data();
	for (unsigned int i = 0; i<lineSalience_.size(); i++)
	{
		int salience = 0;
		for (indexInLineArray = pSID[i]; indexInLineArray < pSID[i + 1]; indexInLineArray++)
		{
			salience += (unsigned char)pgImg[pYCor[indexInLineArray] * imageWidth + pXCor[indexInLineArray]];
		}
		lineSalience_[i] = (float)salience;
	}

	return 0;
}


LineDescriptor::LineDescriptor()
{
	edLineVec_.resize(ONE);
	edLineVec_[ZERO] = new EDLineDetector;

	numOfBand_ = 9;//9 is a good value.
	widthOfBand_ = 7;//widthOfBand_%3 must equal to 0; 7 is a good value.
	gaussCoefL_.resize(widthOfBand_ * 3);
	float u = (float)((widthOfBand_ * 3 - 1) / 2);
	float sigma = (float)((widthOfBand_ * 2 + 1) / 2);// (widthOfBand_*2+1)/2;
	float invsigma2 = -1 / (2 * sigma*sigma);
	float dis;
	for (int i = 0; i<(int)widthOfBand_ * 3; i++){
		dis = i - u;
		gaussCoefL_[i] = exp(dis*dis*invsigma2);
	}
	gaussCoefG_.resize(numOfBand_*widthOfBand_);
	u = (float)((numOfBand_*widthOfBand_ - 1) / 2);
	sigma = u;
	invsigma2 = -1 / (2 * sigma*sigma);
	for (int i = 0; i<(int)(numOfBand_*widthOfBand_); i++){
		dis = i - u;
		gaussCoefG_[i] = exp(dis*dis*invsigma2);

	}

}

LineDescriptor::LineDescriptor(unsigned int numOfBand, unsigned int widthOfBand)
{
	edLineVec_.resize(ONE);
	edLineVec_[ZERO] = new EDLineDetector;

	numOfBand_ = numOfBand;
	widthOfBand_ = widthOfBand;
	gaussCoefL_.resize(widthOfBand_ * 3);
	float u = (float)((widthOfBand_ * 3 - 1) / 2);
	float sigma = (float)((widthOfBand_ * 2 + 1) / 2);// (widthOfBand_*2+1)/2;
	float invsigma2 = -1 / (2 * sigma*sigma);
	float dis;
	for (int i = 0; i<(int)widthOfBand_ * 3; i++){
		dis = i - u;
		gaussCoefL_[i] = exp(dis*dis*invsigma2);
	}
	gaussCoefG_.resize(numOfBand_*widthOfBand_);
	u = (float)((numOfBand_*widthOfBand_ - 1) / 2);
	sigma = u;
	invsigma2 = -1 / (2 * sigma*sigma);
	for (int i = 0; i<(int)(numOfBand_*widthOfBand_); i++){
		dis = i - u;
		gaussCoefG_[i] = exp(dis*dis*invsigma2);

	}

}

LineDescriptor::~LineDescriptor(){

	if ((edLineVec_[ZERO]->imageWidth != NULL) && (edLineVec_[ZERO]->imageHeight != NULL))
	if (edLineVec_[ZERO] != NULL){
		delete edLineVec_[ZERO];
	}

	gaussCoefL_.clear();
	gaussCoefG_.clear();

}

/*Line detection method: element in keyLines[i] includes a set of lines which is the same line
* detected in different scaled images.
*/
int LineDescriptor::ScaledKeyLines(image_int8u_p image, ScaleLineSet &keyLines)
{
	unsigned int numOfFinalLine = 0;

	if (edLineVec_[ZERO]->EDline(image, true)){
		return 1;
	}

	numOfFinalLine += edLineVec_[ZERO]->lines_.numOfLines;

	/*lines which correspond to the same line in the scaled images will be stored in the same element of ScaleLineSet.*/
	std::vector<ScaledLine> scaledLines(numOfFinalLine);//store the lines in ScaledLine structure
	numOfFinalLine = 0;//store the number of finally accepted lines in ScaleLineSet
	unsigned int lineIDInScaleLineVec = 0;
	float dx, dy;
	for (unsigned int lineCurId = 0; lineCurId<edLineVec_[0]->lines_.numOfLines; lineCurId++)
	{//add all line detected in the original image
		scaledLines[numOfFinalLine].scaledCount = 0;
		scaledLines[numOfFinalLine].lineIDInScaled = lineCurId;
		scaledLines[numOfFinalLine].lineIDInScaleLineVec = lineIDInScaleLineVec;
		dx = fabs(edLineVec_[0]->lineEndpoints_[lineCurId][0] - edLineVec_[0]->lineEndpoints_[lineCurId][2]);//x1-x2
		dy = fabs(edLineVec_[0]->lineEndpoints_[lineCurId][1] - edLineVec_[0]->lineEndpoints_[lineCurId][3]);//y1-y2
		scaledLines[numOfFinalLine].lineLength = sqrt(dx*dx + dy*dy);
		numOfFinalLine++;
		lineIDInScaleLineVec++;
	}

	float direction;
	unsigned int lineIDInScaled;
	unsigned int tempID;
	float s1, e1, s2, e2;
	bool shouldChange;
	//Reorganize the detected lines into keyLines
	keyLines.clear();
	keyLines.resize(lineIDInScaleLineVec);

	SingleLineInfo singleLine;
	for (unsigned int lineID = 0; lineID < numOfFinalLine; lineID++){
		lineIDInScaled = scaledLines[lineID].lineIDInScaled;
		direction = edLineVec_[ZERO]->lineDirection_[lineIDInScaled];
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
		if (direction >= -0.75f*PI&&direction<-0.25f*PI){
			if (dy>0){ shouldChange = true; }
		}
		if (direction >= -0.25f*PI&&direction<0.25f*PI){
			if (dx<0){ shouldChange = true; }
		}
		if (direction >= 0.25f*PI&&direction<0.75f*PI){
			if (dy<0){ shouldChange = true; }
		}
		if ((direction >= 0.75f*PI&&direction<PI) || (direction >= -PI&&direction<-0.75f*PI)){
			if (dx>0){ shouldChange = true; }
		}

		if (shouldChange){
			singleLine.sPointInScaledX = e1;
			singleLine.sPointInScaledY = e2;
			singleLine.ePointInScaledX = s1;
			singleLine.ePointInScaledY = s2;
			singleLine.startPointX = e1;
			singleLine.startPointY = e2;
			singleLine.endPointX = s1;
			singleLine.endPointY = s2;
		}
		else{
			singleLine.sPointInScaledX = s1;
			singleLine.sPointInScaledY = s2;
			singleLine.ePointInScaledX = e1;
			singleLine.ePointInScaledY = e2;
			singleLine.startPointX = s1;
			singleLine.startPointY = s2;
			singleLine.endPointX = e1;
			singleLine.endPointY = e2;
		}
		tempID = scaledLines[lineID].lineIDInScaleLineVec;
		keyLines[tempID].push_back(singleLine);
	}

	return 0;
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



static image_int8u_p gaussian_sampler_byte_bbox(image_int8u_p in, boundingbox_t bbox, pixel_float_t scale,
	float sigma_scale)
{
	image_float_p aux = NULL;
	image_int8u_p out = NULL;
	ntuple_list kernel_x = NULL, kernel_y = NULL;
	unsigned int N, M, x, y, i;
	unsigned int _h, _w, nx, ny;
	int xc, yc, j, float_x_size, float_y_size;
	int offy;
	float xx, yy, sum;
	pixel_float_t iscale, sigma;

	iscale.u = 1.f / scale.u;
	iscale.v = 1.f / scale.v;

	N = (unsigned int)ceil(bbox.width * scale.u);
	M = (unsigned int)ceil(bbox.height * scale.v);

	aux = new_image_float(N, bbox.height);
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
	float_x_size = (int)(2 * bbox.width);
	float_y_size = (int)(2 * bbox.height);

	/* First subsampling: x axis */
	for (x = 0; x < aux->xsize; x++)
	{
		xx = (float)x * iscale.u;
		xc = (int)floor(xx + ROUND);
		gaussian_kernel(kernel_x, sigma.u, (float)_w + xx - (float)xc);

		for (y = 0; y < aux->ysize; y++)
		{
			sum = 0.f;
			offy = (y + bbox.y) * in->xsize;
			for (i = 0; i < kernel_x->dim; i++)
			{
				j = xc - _w + i;
				/* symmetry boundary condition */
				while (j < 0) j += float_x_size;
				while (j >= float_x_size) j -= float_x_size;
				if (j >= (int)bbox.width) j = float_x_size - 1 - j;

				sum += in->data[(j + bbox.x) + offy] * kernel_x->values[i];
			}
			aux->data[x + y * aux->xsize] = sum;
		}
	}

	/* Second subsampling: y axis */
	for (y = 0; y < out->ysize; y++)
	{
		yy = (float)y * iscale.v;
		yc = (int)floor(yy + ROUND);
		gaussian_kernel(kernel_y, sigma.v, (float)_h + yy - (float)yc);

		for (x = 0; x < out->xsize; x++)
		{
			sum = 0.0f;
			for (i = 0; i < kernel_y->dim; i++)
			{
				j = yc - _h + i;
				/* symmetry boundary condition */
				while (j < 0) j += float_y_size;
				while (j >= float_y_size) j -= float_y_size;
				if (j >= (int)bbox.height) j = float_y_size - 1 - j;

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

static image_int8u_p gaussian_sampler_byte(image_int8u_p in, pixel_float_t scale,
	float sigma_scale)
{
	image_float_p aux = NULL;
	image_int8u_p out = NULL;
	ntuple_list kernel_x = NULL, kernel_y = NULL;
	unsigned int N, M, x, y, i;
	unsigned int _h, _w, nx, ny;
	int xc, yc, j, float_x_size, float_y_size;
	float xx, yy, sum;//, prec;//, iscale;
	pixel_float_t iscale, sigma;

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
		xc = (int)floor(xx + ROUND);
		gaussian_kernel(kernel_x, sigma.u, (float)_w + xx - (float)xc);

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
		yc = (int)floor(yy + ROUND);
		gaussian_kernel(kernel_y, sigma.v, (float)_h + yy - (float)yc);

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
	int nstarX, nstartY, nendX, nendY;

	nstarX = (int)(startX + ROUND);
	nstartY = (int)(startY + ROUND);
	nendX = (int)(endX + ROUND);
	nendY = (int)(endY + ROUND);

	num = MAX(abs(nendX - nstarX) + 1, abs(nendY - nstartY) + 1);

	return num;
}


static void InverseBoundingBoxLines(boundingbox_t bbox, ScaleLineSet & keyLines)
{
	int k, nsize;
	nsize = (int)keyLines.size();

	for (k = 0; k < nsize; k++)
	{
		keyLines[k][0].startPointX += bbox.x;
		keyLines[k][0].startPointY += bbox.y;
		keyLines[k][0].endPointX += bbox.x;
		keyLines[k][0].endPointY += bbox.y;

	}

}


/*This function is used to get information of lines from downsampled image.*/
void LineDescriptor::InverseGaussianSamplerLines(pixel_float_t gs_scale, ScaleLineSet & keyLines)
{
	float iscale_u, iscale_v;
	float delta_u, delta_v;
	int k, nsize;

	iscale_u = 1.f / gs_scale.u;
	iscale_v = 1.f / gs_scale.v;

	nsize = (int)keyLines.size();

	if ((gs_scale.u != 1.f) && (gs_scale.v != 1.f))
	{
		for (k = 0; k < nsize; k++)
		{
			keyLines[k][0].startPointX *= iscale_u;
			keyLines[k][0].startPointY *= iscale_v;
			keyLines[k][0].endPointX *= iscale_u;
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


int LineDescriptor::Run(float scaleX, float scaleY, boundingbox_t bbox,
	image_int8u_p image, ScaleLineSet & keyLines)
{
	image_int8u_p blur = NULL;
	pixel_float_t gs_scale;
	float sigma_scale;

	gs_scale.u = scaleX;
	gs_scale.v = scaleY;
	sigma_scale = 0.6f;

	blur = gaussian_sampler_byte_bbox(image, bbox, gs_scale, sigma_scale);

	if (ScaledKeyLines(blur, keyLines)){
		delete[]blur->data;
		delete[]blur;
		return 1;
	}

	InverseGaussianSamplerLines(gs_scale, keyLines);

	InverseBoundingBoxLines(bbox, keyLines);

	delete[]blur->data;
	delete[]blur;

	return 0;
}

int _edge_drawing_line_detector(unsigned char *src, int w, int h, 
	float scaleX, float scaleY, boundingbox_t bbox, std::vector<line_float_t> &lines)
{
	int k;
	image_int8u_p _src = NULL;
	LineDescriptor lineDesc;
	ScaleLineSet   lineVec;

	_src = new_image_int8u_ptr(w, h, src);

	if (lineDesc.Run(scaleX, scaleY, bbox, _src, lineVec))
		return 1;

	for (k = 0; k < lineVec.size();k++)
	{
		lines.push_back({ lineVec[k][0].startPointX, lineVec[k][0].startPointY,
			lineVec[k][0].endPointX, lineVec[k][0].endPointY });
	}

	lineVec.clear();
	delete[]_src;

	return 0;
}


/*
@function    EdgeDrawingLineDetector
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
int EdgeDrawingLineDetector(unsigned char *src, int w, int h,
	float scaleX, float scaleY, boundingbox_t bbox, std::vector<line_float_t> &lines)
{
	if (src == NULL)
		return 1;

	if (scaleX*bbox.width < 10 || scaleY*bbox.height < 10)
		return 1;

	if (bbox.x<0 || bbox.x > w - 1 || bbox.y <0 || bbox.y > h - 1
		|| bbox.width <= 0 || bbox.height <= 0
		|| bbox.x + bbox.width > w || bbox.y + bbox.height > h)
		return 1;

	return _edge_drawing_line_detector(src, w, h,
		scaleX, scaleY, bbox, lines);
}