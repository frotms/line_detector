#ifndef _MCV_ALGORITHMS_H_
#define _MCV_ALGORITHMS_H_


#include "Structs.h"
#include "Defines.h"
#include <stdio.h>


image_int8u_p new_image_int8u(unsigned int xsize, unsigned int ysize);
void free_image_int8u(image_int8u_p i);

image_int16s_p new_image_int16s(unsigned int xsize, unsigned int ysize);
void free_image_int16s(image_int16s_p i);

image_float_p new_image_float(unsigned int xsize, unsigned int ysize);
void free_image_float(image_float_p i);

#endif