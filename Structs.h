#ifndef _STRUCTS_H_
#define _STRUCTS_H_


typedef struct image_int32s_s
{
	int * data;
	unsigned int xsize, ysize;
} *image_int32s_p;


typedef struct image_int16s_s
{
	short * data;
	unsigned int xsize, ysize;
} *image_int16s_p;

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


typedef struct
{
	float u;            //col of pixel
	float v;            //row of pixel
}float_pixel_t;


#endif