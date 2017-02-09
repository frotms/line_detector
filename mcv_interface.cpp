#include "mcv_interface.h"


image_int8u_p bytemat_to_int8u_image(Mat src)
{
	image_int8u_p dst = NULL;
	int nsize, k;
	unsigned char *psrc = NULL;
	unsigned char *pdst = NULL;

    dst = new image_int8u_s[1];

	dst->xsize = src.cols;
	dst->ysize = src.rows;

	nsize = src.rows*src.cols;

    dst->data = new unsigned char[nsize];

	psrc = src.data;
	pdst = dst->data;
	for (k = 0; k < nsize; k++)
	{
		*pdst++ = *psrc++;
	}

	psrc = NULL;
	pdst = NULL;

	return dst;
}

image_float_p bytemat_to_float_image(Mat src)
{
	image_float_p dst = NULL;
	int nsize,k;
	unsigned char *psrc = NULL;
	float *pdst = NULL;

    dst = new image_float_s[1];

	dst->xsize = src.cols;
	dst->ysize = src.rows;

	nsize = src.rows*src.cols*src.channels();

    dst->data = new float[nsize];

	psrc = src.data;
	pdst = dst->data;
	for (k = 0; k < nsize; k++)
	{
		*pdst++ = *psrc++;
	}

	psrc = NULL;
	pdst = NULL;

	return dst;
}


image_int16s_p int16smat_to_int16s_image(Mat src)
{
    image_int16s_p dst = NULL;
    int nsize;

    dst = new image_int16s_s[1];

    dst->xsize = src.cols;
    dst->ysize = src.rows;

    nsize = src.rows*src.cols*src.channels();

    dst->data = new short[nsize];

    memcpy(dst->data, src.data, nsize*sizeof(short));

    return dst;
}


Mat int8u_image_to_int8umat(image_int8u_p src)
{
    int nsize;
    Mat dst;

    nsize = src->xsize*src->ysize;

    dst = Mat(src->ysize, src->xsize, CV_8UC1, src->data).clone();

    return dst;
}


Mat int16s_image_to_int16smat(image_int16s_p src)
{
	int nsize;
	Mat dst;

	nsize = src->xsize*src->ysize;

	dst = Mat(src->ysize, src->xsize, CV_16SC1, src->data).clone();

	return dst;
}

Mat float_image_to_bytemat(image_float_p src)
{
	int k,nsize;
	Mat dst;
	float *psrc = NULL;
	unsigned char *pdst = NULL, *p = NULL;

	nsize = src->xsize*src->ysize;

    pdst = new unsigned char[nsize];

	psrc = src->data;
	p = pdst;

	for (k = 0; k < nsize;k++)
	{
		*p++ = (unsigned char)(*psrc++ + ROUND);
	}

	dst = Mat(src->ysize, src->xsize, CV_8UC1, pdst).clone();

	psrc = NULL;
	p = NULL;
    delete[]pdst;

	return dst;
}


image_float_p floatmat_to_float_image(Mat src)
{
    image_float_p dst = NULL;
    int nsize;
    float *psrc = NULL;
    float *pdst = NULL;

    dst = new image_float_s[1];

    dst->xsize = src.cols;
    dst->ysize = src.rows;

    nsize = src.rows*src.cols*src.channels();

    dst->data = new float[nsize];

    psrc = src.ptr<float>();
    pdst = dst->data;
    
    memcpy(pdst,psrc,nsize*sizeof(float));

    psrc = NULL;
    pdst = NULL;

    return dst;
}


Mat float_image_to_floatmat(image_float_p src)
{
    int nsize;
    Mat dst;
    float *psrc = NULL;

    nsize = src->xsize*src->ysize;
    psrc = src->data;

    dst = Mat(src->ysize, src->xsize, CV_32FC1, psrc).clone();

    psrc = NULL;

    return dst;
}


Mat mcv_byte2cvimshow(const unsigned char *src, int w, int h, int c)
{
	if (src == NULL)
		exit(1);

	if (w <= ZERO || h <= ZERO || c <= ZERO)
		exit(1);

	if (!(c == 1 || c == 3))
		exit(1);

	int i, j, _wc;

	Mat dst;
	unsigned char *_data = NULL;

	_wc = w * 3;

	switch (c)
	{
	case 1:
		dst.create(h, w, CV_8UC3);

		for (i = 0; i < h; i++)
		{
			_data = dst.ptr<unsigned char>(i);
			for (j = 0; j < w; j++)
			{
				*_data++ = src[i*w + j];
				*_data++ = src[i*w + j];
				*_data++ = src[i*w + j];
			}
		}

		break;
	case 3:
		dst.create(h, w, CV_8UC3);

		for (i = 0; i < h; i++)
		{
			_data = dst.ptr<unsigned char>(i);
			for (j = 0; j < _wc; j++)
			{
				*_data++ = src[i*_wc + (j / c)*c + (2 - j%c)];
			}
		}
		break;
	}

	return dst;
}


void mcv_imshow(unsigned char* image, int w, int h, int c, char *winName, int _time)
{
	Mat cv_image = mcv_byte2cvimshow(image, w, h, c);

	namedWindow(winName, CV_WINDOW_FREERATIO);
	imshow(winName, cv_image);
	waitKey(_time);
	//destroyWindow(winName);
}

void mcv_imshow_mat(Mat cv_image, char *winName, int _time)
{

	namedWindow(winName, CV_WINDOW_FREERATIO);
	imshow(winName, cv_image);
	waitKey(_time);
	//destroyWindow(winName);
}


Mat mcv_bytemat_to_floatmat(Mat src)
{
	int w, h;
	int i, j;
	Mat dst;
	float *dst_data = NULL;
	unsigned char *src_data = NULL;

	w = src.cols;
	h = src.rows;

	dst.create(h, w, CV_8UC1);

	for (i = 0; i < h; i++)
	{
		dst_data = dst.ptr<float>(i);
		src_data = src.ptr<unsigned char>(i);
		for (j = 0; j < w; j++)
		{
			*dst_data++ = *src_data++;

		}
	}

	return dst;
}
