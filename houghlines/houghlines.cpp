#include "houghlines.h"

#include "stdio.h"
#include <math.h>
#include "string.h"
#include <stdlib.h>
#include <algorithm>

#ifndef ROUND
#define ROUND (0.5F)
#endif

#ifndef MIN
#define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif

#ifndef INT_MAX
#define INT_MAX       2147483647    /* maximum (signed) int value */
#endif


typedef struct
{
	float u;            //col of pixel
	float v;            //row of pixel
}pixel_float_t;


typedef struct
{
	float a;
	float b;
}vec_float_t;

typedef struct image_int8u_s
{
	unsigned char * data;
	unsigned int xsize, ysize;
} *image_int8u_p;

typedef struct image_float_s
{
	float * data;
	unsigned int xsize, ysize;
} *image_float_p;


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

static void free_image_float(image_float_p i)
{
	delete[]i->data;
	delete[]i;
}

static void free_image_int8u(image_int8u_p i)
{
	delete[]i->data;
	delete[]i;
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


static void InverseBoundingBoxLines(boundingbox_t bbox, std::vector<line_float_t> &lines)
{
	int k, nsize;
	nsize = (int)lines.size();

	for (k = 0; k < nsize; k++)
	{
		lines[k].startx += bbox.x;
		lines[k].starty += bbox.y;
		lines[k].endx += bbox.x;
		lines[k].endy += bbox.y;

	}

}

/*This function is used to get information of lines from downsampled image.*/
static void InverseGaussianSamplerLines(pixel_float_t gs_scale, std::vector<line_float_t> &lines)
{
	float iscale_u, iscale_v;
	int k, nsize;

	iscale_u = 1.f / gs_scale.u;
	iscale_v = 1.f / gs_scale.v;

	nsize = (int)lines.size();

	if ((gs_scale.u != 1.f) && (gs_scale.v != 1.f))
	{
		for (k = 0; k < nsize; k++)
		{
			lines[k].startx *= iscale_u;
			lines[k].starty *= iscale_v;
			lines[k].endx *= iscale_u;
			lines[k].endy *= iscale_v;

		}
	}
	else
	if ((gs_scale.u == 1.f) && (gs_scale.v != 1.f))
	{
		for (k = 0; k < nsize; k++)
		{
			lines[k].starty *= iscale_v;
			lines[k].endy *= iscale_v;

		}
	}
	else
	if ((gs_scale.u != 1.f) && (gs_scale.v == 1.f))
	{
		for (k = 0; k < nsize; k++)
		{
			lines[k].startx *= iscale_u;
			lines[k].endx *= iscale_u;
		}
	}
}


//*******************************************************************************
//canny
typedef enum _ORIENT_CODE
{
	ORIENT_HORIZONTAL = 1,       // 水平方向
	ORIENT_VERTICAL = 2,        // 垂直方向

}ORIENT_CODE;


static short *sobel_edge(unsigned char *src, int w, int h, ORIENT_CODE oriention)
{
	unsigned char *psrc = NULL;
	short *dst = NULL;
	int nsize;
	int i, j, _center, offset_up, offset_down;
	int _tp, _td, _t;


	nsize = w * h;
	//dst = (short*)calloc(sizeof(short)*nsize,1);
	dst = new short[nsize]();
	if (!dst)exit(1);

	//边界不处理,置0

	psrc = src;
	switch (oriention)
	{
	case ORIENT_HORIZONTAL:
		for (i = 1; i < h - 1; i++)
		{
			_center = i*w;
			offset_up = _center - w;
			offset_down = _center + w;
			for (j = 1; j < w - 1; j++)
			{
				_tp = offset_up + j;
				_td = offset_down + j;
				_t = _center + j;
				dst[_t] = ((short)psrc[_tp + 1] - (short)psrc[_tp - 1])
					+ ((short)psrc[_td + 1] - (short)psrc[_td - 1])
					+ (((short)psrc[_t + 1] - (short)psrc[_t - 1]) << 1);
			}
		}
		break;

	case  ORIENT_VERTICAL:
		for (i = 1; i < h - 1; i++)
		{
			_center = i*w;
			offset_up = _center - w;
			offset_down = _center + w;
			for (j = 1; j < w - 1; j++)
			{
				_tp = offset_up + j;
				_td = offset_down + j;
				_t = _center + j;

				dst[_t] = -((short)psrc[_tp - 1] + (((short)psrc[_tp]) << 1) + (short)psrc[_tp + 1])
					+ ((short)psrc[_td - 1] + (((short)psrc[_td]) << 1) + (short)psrc[_td + 1]);
			}
		}
		break;

	default:
		//printf("sobel oriention is wrong!");
		break;
	}

	psrc = NULL;
	return dst;
}

static void _mcv_Canny(unsigned char *_src, unsigned char *_dst, int w, int h,
	double low_thresh, double high_thresh)
{

	const int cn = 1; //single channel

	short *dx = NULL;
	short *dy = NULL;
	dx = sobel_edge(_src, w, h, ORIENT_HORIZONTAL);
	dy = sobel_edge(_src, w, h, ORIENT_VERTICAL);

	int offy = 0;

	int low = (int)floor(low_thresh);
	int high = (int)floor(high_thresh);

	long long mapstep = w + 2;
	unsigned char *buffer = new unsigned char[(w + 2)*(h + 2) + cn * mapstep * 3 * sizeof(int)];

	int* mag_buf[3];
	mag_buf[0] = (int*)(unsigned char*)buffer;
	mag_buf[1] = mag_buf[0] + mapstep*cn;
	mag_buf[2] = mag_buf[1] + mapstep*cn;
	memset(mag_buf[0], 0, /* cn* */mapstep*sizeof(int));

	unsigned char* map = (unsigned char*)(mag_buf[2] + mapstep*cn);
	memset(map, 1, mapstep);
	memset(map + mapstep*(h + 1), 1, mapstep);

	int maxsize = MAX(1 << 10, w * h / 10);
	std::vector<unsigned char*> stack(maxsize);
	unsigned char **stack_top = &stack[0];
	unsigned char **stack_bottom = &stack[0];

	/* sector numbers
	(Top-Left Origin)

	1   2   3
	*  *  *
	* * *
	0*******0
	* * *
	*  *  *
	3   2   1
	*/

#define CANNY_PUSH(d)    *(d) = unsigned char(2), *stack_top++ = (d)
#define CANNY_POP(d)     (d) = *--stack_top

	// calculate magnitude and angle of gradient, perform non-maxima suppression.
	// fill the map with one of the following values:
	//   0 - the pixel might belong to an edge
	//   1 - the pixel can not belong to an edge
	//   2 - the pixel does belong to an edge
	for (int i = 0; i <= h; i++)
	{
		offy = i*w;
		int* _norm = mag_buf[(i > 0) + 1] + 1;
		if (i < h)
		{
			short* _dx = dx + offy;
			short* _dy = dy + offy;


			for (int j = 0; j < w; j++)
				_norm[j] = std::abs(int(_dx[j])) + std::abs(int(_dy[j]));

			_norm[-1] = _norm[w] = 0;
		}
		else
			memset(_norm - 1, 0, /* cn* */mapstep*sizeof(int));

		// at the very beginning we do not have a complete ring
		// buffer of 3 magnitude rows for non-maxima suppression
		if (i == 0)
			continue;

		unsigned char* _map = map + mapstep*i + 1;
		_map[-1] = _map[w] = 1;

		int* _mag = mag_buf[1] + 1; // take the central row
		long long  magstep1 = mag_buf[2] - mag_buf[1];
		long long  magstep2 = mag_buf[0] - mag_buf[1];

		const short* _x = dx + offy - w;
		const short* _y = dy + offy - w;

		if ((stack_top - stack_bottom) + w > maxsize)
		{
			int sz = (int)(stack_top - stack_bottom);
			maxsize = maxsize * 3 / 2;
			stack.resize(maxsize);
			stack_bottom = &stack[0];
			stack_top = stack_bottom + sz;
		}

		int prev_flag = 0;
		for (int j = 0; j < w; j++)
		{
#define CANNY_SHIFT 15
			const int TG22 = (int)(0.4142135623730950488016887242097*(1 << CANNY_SHIFT) + 0.5);

			int m = _mag[j];

			if (m > low)
			{
				int xs = _x[j];
				int ys = _y[j];
				int x = std::abs(xs);
				int y = std::abs(ys) << CANNY_SHIFT;

				int tg22x = x * TG22;

				if (y < tg22x)
				{
					if (m > _mag[j - 1] && m >= _mag[j + 1]) goto __ocv_canny_push;
				}
				else
				{
					int tg67x = tg22x + (x << (CANNY_SHIFT + 1));
					if (y > tg67x)
					{
						if (m > _mag[j + magstep2] && m >= _mag[j + magstep1]) goto __ocv_canny_push;
					}
					else
					{
						int s = (xs ^ ys) < 0 ? -1 : 1;
						if (m > _mag[j + magstep2 - s] && m > _mag[j + magstep1 + s]) goto __ocv_canny_push;
					}
				}
			}
			prev_flag = 0;
			_map[j] = unsigned char(1);
			continue;
		__ocv_canny_push:
			if (!prev_flag && m > high && _map[j - mapstep] != 2)
			{
				CANNY_PUSH(_map + j);
				prev_flag = 1;
			}
			else
				_map[j] = 0;
		}

		// scroll the ring buffer
		_mag = mag_buf[0];
		mag_buf[0] = mag_buf[1];
		mag_buf[1] = mag_buf[2];
		mag_buf[2] = _mag;
	}

	// now track the edges (hysteresis thresholding)
	while (stack_top > stack_bottom)
	{
		unsigned char* m = NULL;
		if ((stack_top - stack_bottom) + 8 > maxsize)
		{
			int sz = (int)(stack_top - stack_bottom);
			maxsize = maxsize * 3 / 2;
			stack.resize(maxsize);
			stack_bottom = &stack[0];
			stack_top = stack_bottom + sz;
		}

		CANNY_POP(m);

		if (!m[-1])         CANNY_PUSH(m - 1);
		if (!m[1])          CANNY_PUSH(m + 1);
		if (!m[-mapstep - 1]) CANNY_PUSH(m - mapstep - 1);
		if (!m[-mapstep])   CANNY_PUSH(m - mapstep);
		if (!m[-mapstep + 1]) CANNY_PUSH(m - mapstep + 1);
		if (!m[mapstep - 1])  CANNY_PUSH(m + mapstep - 1);
		if (!m[mapstep])    CANNY_PUSH(m + mapstep);
		if (!m[mapstep + 1])  CANNY_PUSH(m + mapstep + 1);
	}

	// the final pass, form the final image
	const unsigned char* pmap = map + mapstep + 1;
	//unsigned char* pdst = dst.ptr();
	unsigned char* pdst = _dst;
	for (int i = 0; i < h; i++, pmap += mapstep, pdst += w)
	{
		for (int j = 0; j < w; j++)
			pdst[j] = (unsigned char)-(pmap[j] >> 1);
	}

	if (dx)delete[]dx;
	if (dy)delete[]dy;
	delete[]buffer;
}

static void canny_detector(unsigned char *_src, unsigned char *_dst, int w, int h,
	double low_thresh, double high_thresh)
{
	_mcv_Canny(_src, _dst, w, h,
		low_thresh, high_thresh);
}
//*******************************************************************************
//*******************************************************************************
//standard hough


struct LinePolar_t
{
	float rho;
	float angle;
};


struct hough_cmp_gt_t
{
	hough_cmp_gt_t(const int* _aux) : aux(_aux) {}
	bool operator()(int l1, int l2) const
	{
		return aux[l1] > aux[l2] || (aux[l1] == aux[l2] && l1 < l2);
	}
	const int* aux;
};



//max_theta must be greater than min_theta
static void
_mcv_HoughLinesStandard(unsigned char *image, int w, int h, float rho, float theta,
int threshold, int linesMax,
double min_theta, double max_theta, std::vector<vec_float_t> &lines)
{
	int i, j;
	float irho = 1 / rho;
	int step = w;
	int offy;

	int numangle = (int)round((max_theta - min_theta) / theta);
	int numrho = (int)round(((w + h) * 2 + 1) / rho);

	std::vector<int> _sort_buf;
	int *_accum = new int[(numangle + 2) * (numrho + 2)];
	float *_tabSin = new float[numangle];
	float *_tabCos = new float[numangle];


	int *accum = _accum;
	float *tabSin = _tabSin, *tabCos = _tabCos;

	memset(accum, 0, sizeof(accum[0]) * (numangle + 2) * (numrho + 2));

	float ang = static_cast<float>(min_theta);
	for (int n = 0; n < numangle; ang += theta, n++)
	{
		tabSin[n] = (float)(sin((double)ang) * irho);
		tabCos[n] = (float)(cos((double)ang) * irho);
	}

	// stage 1. fill accumulator
	for (i = 0; i < h; i++)
	{
		offy = i*w;
		for (j = 0; j < w; j++)
		{
			if (image[offy + j] != 0)
			for (int n = 0; n < numangle; n++)
			{
				int r = (int)round(j * tabCos[n] + i * tabSin[n]);
				r += (numrho - 1) / 2;
				accum[(n + 1) * (numrho + 2) + r + 1]++;
			}
		}
	}


	// stage 2. find local maximums
	for (int r = 0; r < numrho; r++)
	for (int n = 0; n < numangle; n++)
	{
		int base = (n + 1) * (numrho + 2) + r + 1;
		if (accum[base] > threshold &&
			accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
			accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
			_sort_buf.push_back(base);
	}

	// stage 3. sort the detected lines by accumulator value
	std::sort(_sort_buf.begin(), _sort_buf.end(), hough_cmp_gt_t(accum));

	// stage 4. store the first min(total,linesMax) lines to the output buffer
	linesMax = std::min(linesMax, (int)_sort_buf.size());
	double scale = 1. / (numrho + 2);
	for (i = 0; i < linesMax; i++)
	{
		LinePolar_t line;
		int idx = _sort_buf[i];
		int n = (int)floor(idx*scale) - 1;
		int r = idx - (n + 1)*(numrho + 2) - 1;
		line.rho = (r - (numrho - 1)*0.5f) * rho;
		line.angle = static_cast<float>(min_theta)+n * theta;
		lines.push_back({ line.rho, line.angle });
	}

	delete[]_accum;
	delete[]_tabSin;
	delete[]_tabCos;

}


//max_theta must be greater than min_theta
static void hough_line_standard(unsigned char *image, int w, int h,
	double rho, double theta, int threshold,
	double min_theta, double max_theta,
	std::vector<line_float_t> &lines)
{
	int k;
	float d;
	double a, b;
	double x0, y0;
	float x1, y1, x2, y2;

	std::vector<vec_float_t> _lines;

	_mcv_HoughLinesStandard(image, w, h, (float)rho, (float)theta, threshold, INT_MAX, min_theta, max_theta, _lines);

	//transfer
	for (k = 0; k < _lines.size(); k++)
	{
		a = cos(_lines[k].b);
		b = sin(_lines[k].b);
		x0 = a*_lines[k].a;
		y0 = b*_lines[k].a;
		d = (float)sqrt(w*w + h*h);
		x1 = (float)(x0 + d * (-b));
		y1 = (float)(y0 + d * (a));
		x2 = (float)(x0 - d * (-b));
		y2 = (float)(y0 - d * (a));

		lines.push_back({ x1, y1, x2, y2 });

	}

	_lines.clear();
}

//*******************************************************************************
//*******************************************************************************
//probabilistic hough


typedef struct
{
	int x;
	int y;
}point_int_t;


static void
_mcv_HoughLinesProbabilistic(int w, int h, unsigned char *_image,
float rho, float theta, int threshold,
int lineLength, int lineGap,
int linesMax, std::vector<line_float_t>& _lines)
{
	int i, j;
	float irho = 1 / rho;
	int offset, offy;
	
	int numangle = (int)round(PI / theta);
	int numrho = (int)round(((w + h) * 2 + 1) / rho);

	int *_accum = new int[numangle*numrho]();
	unsigned char *_mask = new unsigned char[h*w];


	std::vector<float> trigtab(numangle * 2);

	for (int n = 0; n < numangle; n++)
	{
		offset = n * 2;
		trigtab[offset] = (float)(cos((double)n*theta) * irho);
		trigtab[offset + 1] = (float)(sin((double)n*theta) * irho);
	}
	const float* ttab = &trigtab[0];
	unsigned char* mdata0 = _mask;
	std::vector<point_int_t> nzloc;

	// stage 1. collect non-zero image points
	for (i = 0; i < h; i++)
	{
		offy = i*w;
		for (j = 0; j < w; j++)
		{
			if (_image[offy + j])
			{
				_mask[offy + j] = (unsigned char)1;
				nzloc.push_back({ j, i });
			}
			else
				_mask[offy + j] = 0;
		}
	}

	int count = (int)nzloc.size();

	// stage 2. process all the points in random order
	for (; count > 0; count--)
	{
		// choose random point out of the remaining ones
		int idx = rand() % count;
		int max_val = threshold - 1, max_n = 0;
		point_int_t point = nzloc[idx];
		point_int_t line_end[2];
		float a, b;
		int* adata = _accum;
		i = point.y, j = point.x;
		int k, x0, y0, dx0, dy0, xflag;
		int good_line;
		const int shift = 16;

		// "remove" it by overriding it with the last element
		nzloc[idx] = nzloc[count - 1];

		// check if it has been excluded already (i.e. belongs to some other line)
		if (!mdata0[i*w + j])
			continue;

		// update accumulator, find the most probable line
		for (int n = 0; n < numangle; n++, adata += numrho)
		{
			int r = (int)round(j * ttab[n * 2] + i * ttab[n * 2 + 1]);
			r += (numrho - 1) / 2;
			int val = ++adata[r];
			if (max_val < val)
			{
				max_val = val;
				max_n = n;
			}
		}

		// if it is too "weak" candidate, continue with another point
		if (max_val < threshold)
			continue;

		// from the current point walk in each direction
		// along the found line and extract the line segment
		a = -ttab[max_n * 2 + 1];
		b = ttab[max_n * 2];
		x0 = j;
		y0 = i;
		if (fabs(a) > fabs(b))
		{
			xflag = 1;
			dx0 = a > 0 ? 1 : -1;
			dy0 = (int)round(b*(1 << shift) / fabs(a));
			y0 = (y0 << shift) + (1 << (shift - 1));
		}
		else
		{
			xflag = 0;
			dy0 = b > 0 ? 1 : -1;
			dx0 = (int)round(a*(1 << shift) / fabs(b));
			x0 = (x0 << shift) + (1 << (shift - 1));
		}

		for (k = 0; k < 2; k++)
		{
			int gap = 0, x = x0, y = y0, dx = dx0, dy = dy0;

			if (k > 0)
				dx = -dx, dy = -dy;

			// walk along the line using fixed-point arithmetics,
			// stop at the image border or in case of too big gap
			for (;; x += dx, y += dy)
			{
				unsigned char* mdata = NULL;
				int i1, j1;

				if (xflag)
				{
					j1 = x;
					i1 = y >> shift;
				}
				else
				{
					j1 = x >> shift;
					i1 = y;
				}

				if (j1 < 0 || j1 >= w || i1 < 0 || i1 >= h)
					break;

				mdata = mdata0 + i1*w + j1;

				// for each non-zero point:
				//    update line end,
				//    clear the mask element
				//    reset the gap
				if (*mdata)
				{
					gap = 0;
					line_end[k].y = i1;
					line_end[k].x = j1;
				}
				else if (++gap > lineGap)
					break;
			}
		}

		good_line = std::abs(line_end[1].x - line_end[0].x) >= lineLength ||
			std::abs(line_end[1].y - line_end[0].y) >= lineLength;

		for (k = 0; k < 2; k++)
		{
			int x = x0, y = y0, dx = dx0, dy = dy0;

			if (k > 0)
				dx = -dx, dy = -dy;

			// walk along the line using fixed-point arithmetics,
			// stop at the image border or in case of too big gap
			for (;; x += dx, y += dy)
			{
				unsigned char* mdata = NULL;
				int i1, j1;

				if (xflag)
				{
					j1 = x;
					i1 = y >> shift;
				}
				else
				{
					j1 = x >> shift;
					i1 = y;
				}

				mdata = mdata0 + i1*w + j1;

				// for each non-zero point:
				//    update line end,
				//    clear the mask element
				//    reset the gap
				if (*mdata)
				{
					if (good_line)
					{
						adata = _accum;
						for (int n = 0; n < numangle; n++, adata += numrho)
						{
							int r = (int)round(j1 * ttab[n * 2] + i1 * ttab[n * 2 + 1]);
							r += (numrho - 1) / 2;
							adata[r]--;
						}
					}
					*mdata = 0;
				}

				if (i1 == line_end[k].y && j1 == line_end[k].x)
					break;
			}
		}

		if (good_line)
		{
			line_float_t _lr = { (float)line_end[0].x, (float)line_end[0].y, (float)line_end[1].x, (float)line_end[1].y };
			_lines.push_back(_lr);
			if ((int)_lines.size() >= linesMax)
				return;
		}
	}

	delete[]_accum;
	delete[]_mask;
}


static void hough_line_probabilistic(unsigned char *image, int w, int h,
	double rho, double theta, int threshold,
	double minLineLength, double maxGap, std::vector<line_float_t>&lines)
{
	_mcv_HoughLinesProbabilistic(w, h, image,
		(float)rho, (float)theta, threshold, (int)round(minLineLength), (int)round(maxGap), INT_MAX, lines);

}

//*******************************************************************************
//interface

static void _hough_line_detector(unsigned char *src, int w, int h,
	float scaleX, float scaleY, float canny_low_thresh, float canny_high_thresh,
	float hough_rho, float hough_theta, float min_theta_linelength, float max_theta_gap, int hough_thresh,
	HOUGH_LINE_TYPE_CODE _type,
	boundingbox_t bbox, std::vector<line_float_t> &lines)
{

	unsigned char *edge = NULL;
	float sigma = 0.6f;
	pixel_float_t scale = {scaleX,scaleY};
	image_int8u_p _src = new_image_int8u_ptr(w,h,src);
	image_int8u_p sub_src = gaussian_sampler_byte_bbox(_src, bbox, scale, sigma);

	lines.clear();
	//processing
	edge = new unsigned char[sub_src->xsize*sub_src->ysize];
	canny_detector(sub_src->data,edge,sub_src->xsize, sub_src->ysize,canny_low_thresh,canny_high_thresh);

	switch (_type)
	{
	case HOUGH_LINE_STANDARD:
		hough_line_standard(edge, sub_src->xsize, sub_src->ysize,
			(double)hough_rho, (double)hough_theta, hough_thresh, 
			(double)min_theta_linelength, (double)max_theta_gap,
			lines);

		break;

	case HOUGH_LINE_PROBABILISTIC:
		hough_line_probabilistic(edge, sub_src->xsize, sub_src->ysize,
			(double)hough_rho, (double)hough_theta, hough_thresh,
			(double)min_theta_linelength, (double)max_theta_gap,
			lines);

		break;

	default:
		break;
	}


	//inverse
	InverseGaussianSamplerLines(scale, lines);
	InverseBoundingBoxLines(bbox, lines);

	delete[]edge;
	free_image_int8u(sub_src);
	delete[]_src;
}


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
{
	if (src == NULL)
		return 1;

	if (scaleX*bbox.width < 10 || scaleY*bbox.height < 10 )
		return 1;

	if (bbox.x<0 || bbox.x > w-1 || bbox.y <0 || bbox.y > h-1
		|| bbox.width <= 0 || bbox.height <=0 
		|| bbox.x + bbox.width > w || bbox.y + bbox.height > h)
		return 1;

	_hough_line_detector(src, w, h, scaleX, scaleY, 
		CannyLowThresh, CannyHighThresh, 
		HoughRho, HoughTheta, MinThetaLinelength, MaxThetaGap, HoughThresh,
		_type,
		bbox, lines);

	return 0;
}
