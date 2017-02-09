#include "mcv_algorithms.h"


image_int8u_p new_image_int8u(unsigned int xsize, unsigned int ysize)
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

image_int16s_p new_image_int16s(unsigned int xsize, unsigned int ysize)
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



image_float_p new_image_float(unsigned int xsize, unsigned int ysize)
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

void free_image_int8u(image_int8u_p i)
{
    delete[]i->data;
    delete[]i;
}
