

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include <float.h>
#include "lsd.h"

/** ln(10) */
#ifndef M_LN10
//#define M_LN10 2.30258509299404568402
#define M_LN10 2.3025851f
#endif /* !M_LN10 */

/** Label for pixels with undefined gradient. */
#define NOTDEF -1024.f

/** Label for pixels not used in yet. */
#define NOTUSED 0

/** Label for pixels already used in detection. */
#define USED    1

/*----------------------------------------------------------------------------*/
/** Chained list of coordinates.
 */
struct coorlist
{
  int x,y;
  struct coorlist * next;
};

/*----------------------------------------------------------------------------*/
/** A point (or pixel).
 */
struct point {int x,y;};


/*----------------------------------------------------------------------------*/
/*------------------------- Miscellaneous functions --------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/** Doubles relative error factor
 */
#define RELATIVE_ERROR_FACTOR 100.f


static int float_equal(float a, float b)
{
    float abs_diff,aa,bb,abs_max;

    /* trivial case */
    if( a == b ) return TRUE;

    abs_diff = fabs(a-b);
    aa = fabs(a);
    bb = fabs(b);
    abs_max = aa > bb ? aa : bb;

    if (abs_max < X_EPS) abs_max = X_EPS;

    /* equal if relative error <= factor x eps */
    return (abs_diff / abs_max) <= (RELATIVE_ERROR_FACTOR * X_EPS);
}

/*----------------------------------------------------------------------------*/
/** Computes Euclidean distance between point (x1,y1) and point (x2,y2).
 */
static float dist(float x1, float y1, float x2, float y2)
{
    return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
}


typedef struct ntuple_list_s
{
    unsigned int size;
    unsigned int max_size;
    unsigned int dim;
    float * values;
} * ntuple_list;

/*----------------------------------------------------------------------------*/
/** Free memory used in n-tuple 'in'.
 */
static void free_ntuple_list(ntuple_list in)
{
    free( (void *) in->values );
    free( (void *) in );
}

/*----------------------------------------------------------------------------*/
/** Create an n-tuple list and allocate memory for one element.
    @param dim the dimension (n) of the n-tuple.
 */
static ntuple_list new_ntuple_list(unsigned int dim)
{
    ntuple_list n_tuple;

    /* get memory for list structure */
    n_tuple = (ntuple_list) malloc( sizeof(struct ntuple_list_s) );

    /* initialize list */
    n_tuple->size = 0;
    n_tuple->max_size = 1;
    n_tuple->dim = dim;

    /* get memory for tuples */
    n_tuple->values = (float *) malloc( dim*n_tuple->max_size * sizeof(float) );

    return n_tuple;
}

/*----------------------------------------------------------------------------*/
/** Enlarge the allocated memory of an n-tuple list.
 */
static void enlarge_ntuple_list(ntuple_list n_tuple)
{
    /* duplicate number of tuples */
    n_tuple->max_size *= 2;

    /* realloc memory */
    n_tuple->values = (float *) realloc( (void *) n_tuple->values,
                        n_tuple->dim * n_tuple->max_size * sizeof(float) );
}

/*----------------------------------------------------------------------------*/
/** Add a 7-tuple to an n-tuple list.
 */
static void add_7tuple( ntuple_list out, float v1, float v2, float v3,
                        float v4, float v5, float v6, float v7 )
{
    /* if needed, alloc more tuples to 'out' */
    if( out->size == out->max_size ) enlarge_ntuple_list(out);

    /* add new 7-tuple */
    out->values[ out->size * out->dim + 0 ] = v1;
    out->values[ out->size * out->dim + 1 ] = v2;
    out->values[ out->size * out->dim + 2 ] = v3;
    out->values[ out->size * out->dim + 3 ] = v4;
    out->values[ out->size * out->dim + 4 ] = v5;
    out->values[ out->size * out->dim + 5 ] = v6;
    out->values[ out->size * out->dim + 6 ] = v7;

    /* update number of tuples counter */
    out->size++;
}


typedef struct image_char_s
{
    unsigned char * data;
    unsigned int xsize,ysize;
} * image_char;

/*----------------------------------------------------------------------------*/
/** Free memory used in image_char 'i'.
 */
static void free_image_char(image_char i)
{
    free( (void *) i->data );
    free( (void *) i );
}

/*----------------------------------------------------------------------------*/
/** Create a new image_char of size 'xsize' times 'ysize'.
 */
static image_char new_image_char(unsigned int xsize, unsigned int ysize)
{
    image_char image;

    /* get memory */
    image = (image_char) malloc( sizeof(struct image_char_s) );

    image->data = (unsigned char *) calloc( (size_t) (xsize*ysize),
                                            sizeof(unsigned char) );
    /* set image size */
    image->xsize = xsize;
    image->ysize = ysize;

    return image;
}

/*----------------------------------------------------------------------------*/
/** Create a new image_char of size 'xsize' times 'ysize',
    initialized to the value 'fill_value'.
 */
static image_char new_image_char_ini( unsigned int xsize, unsigned int ysize,
                                      unsigned char fill_value )
{
    image_char image = new_image_char(xsize,ysize); /* create image */
    unsigned int N = xsize*ysize;
    unsigned int i;

    /* initialize */
    for(i=0; i<N; i++) image->data[i] = fill_value;

    return image;
}

typedef struct image_int_s
{
    int * data;
    unsigned int xsize,ysize;
} * image_int;

/*----------------------------------------------------------------------------*/
/** Create a new image_int of size 'xsize' times 'ysize'.
 */
static image_int new_image_int(unsigned int xsize, unsigned int ysize)
{
    image_int image;

    /* get memory */
    image = (image_int) malloc( sizeof(struct image_int_s) );
    image->data = (int *) calloc( (size_t) (xsize*ysize), sizeof(int) );

    /* set image size */
    image->xsize = xsize;
    image->ysize = ysize;

    return image;
}

/*----------------------------------------------------------------------------*/
/** Create a new image_int of size 'xsize' times 'ysize',
    initialized to the value 'fill_value'.
 */
static image_int new_image_int_ini( unsigned int xsize, unsigned int ysize,
                                    int fill_value )
{
    image_int image = new_image_int(xsize,ysize); /* create image */
    unsigned int N = xsize*ysize;
    unsigned int i;

    /* initialize */
    for(i=0; i<N; i++) image->data[i] = fill_value;

    return image;
}


//typedef struct image_float_s
//{
//    float * data;
//    unsigned int xsize,ysize;
//} * image_float_p;

/*----------------------------------------------------------------------------*/
/** Free memory used in image_float_p 'i'.
 */
static void free_image_float(image_float_p i)
{
    free( (void *) i->data );
    free( (void *) i );
}

/*----------------------------------------------------------------------------*/
/** Create a new image_float_p of size 'xsize' times 'ysize'.
 */
static image_float_p new_image_float(unsigned int xsize, unsigned int ysize)
{
    image_float_p image;

    /* get memory */
    image = (image_float_p) malloc( sizeof(struct image_float_s) );
    image->data = (float *) calloc( (size_t) (xsize*ysize), sizeof(float) );

    /* set image size */
    image->xsize = xsize;
    image->ysize = ysize;

    return image;
}

/*----------------------------------------------------------------------------*/
/** Create a new image_float_p of size 'xsize' times 'ysize'
    with the data pointed by 'data'.
 */
static image_float_p new_image_float_ptr( unsigned int xsize,
                                          unsigned int ysize, float * data )
{
    image_float_p image;

    /* get memory */
    image = (image_float_p) malloc( sizeof(struct image_float_s) );

    /* set image */
    image->xsize = xsize;
    image->ysize = ysize;
    image->data = data;

    return image;
}


static void gaussian_kernel(ntuple_list kernel, float sigma, float mean)
{
    float sum = 0.f;
    float val;
    unsigned int i;

    /* compute Gaussian kernel */
    if( kernel->max_size < 1 ) enlarge_ntuple_list(kernel);
    kernel->size = 1;
    for(i=0;i<kernel->dim;i++)
    {
        val = ( (float) i - mean ) / sigma;
        kernel->values[i] = exp( -0.5f * val * val );
        sum += kernel->values[i];
    }

    /* normalization */
    if( sum >= 0.f ) for(i=0;i<kernel->dim;i++) kernel->values[i] /= sum;
}

/*----------------------------------------------------------------------------*/
/** Scale the input image 'in' by a factor 'scale' by Gaussian sub-sampling.
 */
static image_float_p gaussian_sampler(image_float_p in, float scale_x, float scale_y,
    float sigma_scale)
{
    image_float_p aux = NULL, out = NULL;
    ntuple_list kernel_x = NULL, kernel_y = NULL;
    unsigned int N, M, x, y, i;
    unsigned int _h, _w, nx, ny;
    int xc, yc, j, float_x_size, float_y_size;
    float xx, yy, sum;
    float iscale_x, iscale_y, sigma_x, sigma_y;

    iscale_x = 1.f / scale_x;
    iscale_y = 1.f / scale_y;

    N = (unsigned int)ceil(in->xsize * scale_x);
    M = (unsigned int)ceil(in->ysize * scale_y);
    aux = new_image_float(N, in->ysize);
    out = new_image_float(N, M);

    /* sigma, kernel size and memory for the kernel */
    sigma_x = (scale_x < 1.f) ? (sigma_scale * iscale_x) : (sigma_scale);
    sigma_y = (scale_y < 1.f) ? (sigma_scale * iscale_y) : (sigma_scale);

    _h = (unsigned int)ceil(sigma_y * 3.71692219f);//  sqrt(prec * 4.605170186f)); //log(10)*2  
    _w = (unsigned int)ceil(sigma_x * 3.71692219f);//  sqrt(prec * 4.605170186f)); //log(10)*2  

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

        xx = (float)x * iscale_x;
        /* coordinate (0.0,0.0) is in the center of pixel (0,0),
        so the pixel with xc=0 get the values of xx from -0.5 to 0.5 */
        xc = (int)floor(xx + ROUND);
        gaussian_kernel(kernel_x, sigma_x, (float)_w + xx - (float)xc);
        /* the kernel must be computed for each x because the fine
        offset xx-xc is different in each case */

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
        yy = (float)y * iscale_y;
        /* coordinate (0.0,0.0) is in the center of pixel (0,0),
        so the pixel with yc=0 get the values of yy from -0.5 to 0.5 */
        yc = (int)floor(yy + ROUND);
        gaussian_kernel(kernel_y, sigma_y, (float)_h + yy - (float)yc);
        /* the kernel must be computed for each y because the fine
        offset yy-yc is different in each case */

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
            out->data[x + y * out->xsize] = sum;
        }
    }

    /* free memory */
    //free_ntuple_list(kernel);
    free_ntuple_list(kernel_x);
    free_ntuple_list(kernel_y);
    free_image_float(aux);

    return out;
}

/*----------------------------------------------------------------------------*/
/** Computes the direction of the level line of 'in' at each point.
 */
static image_float_p ll_angle( image_float_p in, float threshold,
                              struct coorlist ** list_p, void ** mem_p,
                              image_float_p * modgrad, unsigned int n_bins )
{
    image_float_p g;
    unsigned int n,p,x,y,adr,i;
    float com1,com2,gx,gy,norm,norm2;
    /* the rest of the variables are used for pseudo-ordering
        the gradient magnitude values */
    int list_count = 0;
    struct coorlist * list;
    struct coorlist ** range_l_s; /* array of pointers to start of bin list */
    struct coorlist ** range_l_e; /* array of pointers to end of bin list */
    struct coorlist * start;
    struct coorlist * end;
    float max_grad = 0.f;

    /* image size shortcuts */
    n = in->ysize;
    p = in->xsize;

    /* allocate output image */
    g = new_image_float(in->xsize,in->ysize);

    /* get memory for the image of gradient modulus */
    *modgrad = new_image_float(in->xsize,in->ysize);

    /* get memory for "ordered" list of pixels */
    list = (struct coorlist *) calloc( (size_t) (n*p), sizeof(struct coorlist) );
    *mem_p = (void *) list;
    range_l_s = (struct coorlist **) calloc( (size_t) n_bins,
                                            sizeof(struct coorlist *) );
    range_l_e = (struct coorlist **) calloc( (size_t) n_bins,
                                            sizeof(struct coorlist *) );

    for(i=0;i<n_bins;i++) range_l_s[i] = range_l_e[i] = NULL;

    /* 'undefined' on the down and right boundaries */
    for(x=0;x<p;x++) g->data[(n-1)*p+x] = NOTDEF;
    for(y=0;y<n;y++) g->data[p*y+p-1]   = NOTDEF;

    /* compute gradient on the remaining pixels */
    for(x=0;x<p-1;x++)
        for(y=0;y<n-1;y++)
        {
            adr = y*p+x;

            com1 = in->data[adr+p+1] - in->data[adr];
            com2 = in->data[adr+1]   - in->data[adr+p];

            gx = com1+com2; /* gradient x component */
            gy = com1-com2; /* gradient y component */
            norm2 = gx*gx+gy*gy;
            norm = sqrt( norm2 / 4.f ); /* gradient norm */

            (*modgrad)->data[adr] = norm; /* store gradient norm */

            if( norm <= threshold ) /* norm too small, gradient no defined */
                g->data[adr] = NOTDEF; /* gradient angle not defined */
            else
            {
                /* gradient angle computation */
                g->data[adr] = atan2(gx,-gy);

                /* look for the maximum of the gradient */
                if( norm > max_grad ) max_grad = norm;
            }
        }

    /* compute histogram of gradient values */
    for(x=0;x<p-1;x++)
    for(y=0;y<n-1;y++)
        {
        norm = (*modgrad)->data[y*p+x];

        /* store the point in the right bin according to its norm */
        i = (unsigned int) (norm * (float) n_bins / max_grad);
        if( i >= n_bins ) i = n_bins-1;
        if( range_l_e[i] == NULL )
            range_l_s[i] = range_l_e[i] = list+list_count++;
        else
            {
            range_l_e[i]->next = list+list_count;
            range_l_e[i] = list+list_count++;
            }
        range_l_e[i]->x = (int) x;
        range_l_e[i]->y = (int) y;
        range_l_e[i]->next = NULL;
        }

    for(i=n_bins-1; i>0 && range_l_s[i]==NULL; i--);
    start = range_l_s[i];
    end = range_l_e[i];
    if( start != NULL )
    while(i>0)
        {
        --i;
        if( range_l_s[i] != NULL )
            {
            end->next = range_l_s[i];
            end = range_l_e[i];
            }
        }
    *list_p = start;

    /* free memory */
    free( (void *) range_l_s );
    free( (void *) range_l_e );

    return g;
}

/*----------------------------------------------------------------------------*/
/** Is point (x,y) aligned to angle theta, up to precision 'prec'?
 */
static int isaligned( int x, int y, image_float_p angles, float theta,
                      float prec )
{
    float a;

    /* angle at pixel (x,y) */
    a = angles->data[ x + y * angles->xsize ];

    /* pixels whose level-line angle is not defined
        are considered as NON-aligned */
    if( a == NOTDEF ) return FALSE;  /* there is no need to call the function
                                        'float_equal' here because there is
                                        no risk of problems related to the
                                        comparison doubles, we are only
                                        interested in the exact NOTDEF value */

    /* it is assumed that 'theta' and 'a' are in the range [-pi,pi] */
    theta -= a;
    if( theta < 0.f ) theta = -theta;
    if( theta > PI_3_2 )
    {
        theta -= PI_2;
        if( theta < 0.f ) theta = -theta;
    }

    return theta <= prec;
}

/*----------------------------------------------------------------------------*/
/** Absolute value angle difference.
 */
static float angle_diff(float a, float b)
{
    a -= b;
    while( a <= -PI ) a += PI_2;
    while( a >   PI ) a -= PI_2;
    if( a < 0.f ) a = -a;
    return a;
}

/*----------------------------------------------------------------------------*/
/** Signed angle difference.
 */
static float angle_diff_signed(float a, float b)
{
    a -= b;
    while( a <= -PI ) a += PI_2;
    while( a >   PI ) a -= PI_2;
    return a;
}


/*----------------------------------------------------------------------------*/
/*----------------------------- NFA computation ------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/** Computes the natural logarithm of the absolute value of
    the gamma function of x using the Lanczos approximation.
    See http://www.rskey.org/gamma.htm
 */
static float log_gamma_lanczos(float x)
{
    static float q[7] = { 75122.6331530f, 80916.6278952f, 36308.2951477f,
                            8687.24529705f, 1168.92649479f, 83.8676043424f,
                            2.50662827511f };
    float a = (x+0.5f) * log(x+5.5f) - (x+5.5f);
    float b = 0.f;
    int n;

    for(n=0;n<7;n++)
    {
        a -= log( x + (float) n );
        b += q[n] * pow( x, (float) n );
    }
    return a + log(b);
}

/*----------------------------------------------------------------------------*/
/** Computes the natural logarithm of the absolute value of
    the gamma function of x using Windschitl method.
    See http://www.rskey.org/gamma.htm
 */
static float log_gamma_windschitl(float x)
{
    return 0.918938533204673f + (x-0.5f)*log(x) - x
            + 0.5f*x*log( x*sinh(1/x) + 1/(810.f*pow(x,6.f)) );
}

/*----------------------------------------------------------------------------*/
/** Computes the natural logarithm of the absolute value of
    the gamma function of x. When x>15 use log_gamma_windschitl(),
    otherwise use log_gamma_lanczos().
 */
#define log_gamma(x) ((x)>15.f?log_gamma_windschitl(x):log_gamma_lanczos(x))

/*----------------------------------------------------------------------------*/
/** Size of the table to store already computed inverse values.
 */
#define TABSIZE 100000

/*----------------------------------------------------------------------------*/
/** Computes -log10(NFA).
 */
static float nfa(int n, int k, float p, float logNT)
{
    static float inv[TABSIZE];   /* table to keep computed inverse values */
    float tolerance = 0.1f;       /* an error of 10% in the result is accepted */
    float log1term,term,bin_term,mult_term,bin_tail,err,p_term;
    int i;

    /* trivial cases */
    if( n==0 || k==0 ) return -logNT;
    if( n==k ) return -logNT - (float) n * log10(p);

    /* probability term */
    p_term = p / (1.f-p);

    log1term = log_gamma( (float) n + 1.f ) - log_gamma( (float) k + 1.f )
            - log_gamma( (float) (n-k) + 1.f )
            + (float) k * log(p) + (float) (n-k) * log(1.f-p);
    term = exp(log1term);

    /* in some cases no more computations are needed */
    if( float_equal(term,0.f) )              /* the first term is almost zero */
    {
        if( (float) k > (float) n * p )     /* at begin or end of the tail?  */
        return -log1term / M_LN10 - logNT;  /* end: use just the first term  */
        else
        return -logNT;                      /* begin: the tail is roughly 1  */
    }

    /* compute more terms if needed */
    bin_tail = term;
    for(i=k+1;i<=n;i++)
    {
        bin_term = (float) (n-i+1) * ( i<TABSIZE ?
                    ( inv[i]!=0.f ? inv[i] : ( inv[i] = 1.f / (float) i ) ) :
                    1.f / (float) i );

        mult_term = bin_term * p_term;
        term *= mult_term;
        bin_tail += term;
        if(bin_term<1.f)
        {
            err = term * ( ( 1.f - pow( mult_term, (float) (n-i+1) ) ) /
                            (1.f-mult_term) - 1.f );

            if( err < tolerance * fabs(-log10(bin_tail)-logNT) * bin_tail ) break;
        }
    }
    return -log10(bin_tail) - logNT;
}


/*----------------------------------------------------------------------------*/
/** Rectangle structure: line segment with width.
 */
struct rect
{
    float x1,y1,x2,y2;  /* first and second point of the line segment */
    float width;        /* rectangle width */
    float x,y;          /* center of the rectangle */
    float theta;        /* angle */
    float dx,dy;        /* (dx,dy) is vector oriented as the line segment */
    float prec;         /* tolerance angle */
    float p;            /* probability of a point with angle within 'prec' */
};

/*----------------------------------------------------------------------------*/
/** Copy one rectangle structure to another.
 */
static void rect_copy(struct rect * in, struct rect * out)
{
    /* copy values */
    out->x1 = in->x1;
    out->y1 = in->y1;
    out->x2 = in->x2;
    out->y2 = in->y2;
    out->width = in->width;
    out->x = in->x;
    out->y = in->y;
    out->theta = in->theta;
    out->dx = in->dx;
    out->dy = in->dy;
    out->prec = in->prec;
    out->p = in->p;
}

/*----------------------------------------------------------------------------*/
/** Rectangle points iterator.
 */
typedef struct
{
    float vx[4];  /* rectangle's corner X coordinates in circular order */
    float vy[4];  /* rectangle's corner Y coordinates in circular order */
    float ys,ye;  /* start and end Y values of current 'column' */
    int x,y;       /* coordinates of currently explored pixel */
} rect_iter;

/*----------------------------------------------------------------------------*/
/** Interpolate y value corresponding to 'x' value given, in
 */
static float inter_low(float x, float x1, float y1, float x2, float y2)
{
    /* interpolation */
    if( float_equal(x1,x2) && y1<y2 ) return y1;
    if( float_equal(x1,x2) && y1>y2 ) return y2;
    return y1 + (x-x1) * (y2-y1) / (x2-x1);
}

/*----------------------------------------------------------------------------*/
/** Interpolate y value corresponding to 'x' value given, in
 */
static float inter_hi(float x, float x1, float y1, float x2, float y2)
{
    /* interpolation */
    if( float_equal(x1,x2) && y1<y2 ) return y2;
    if( float_equal(x1,x2) && y1>y2 ) return y1;
    return y1 + (x-x1) * (y2-y1) / (x2-x1);
}

/*----------------------------------------------------------------------------*/
/** Free memory used by a rectangle iterator.
 */
static void ri_del(rect_iter * iter)
{
    free( (void *) iter );
}

/*----------------------------------------------------------------------------*/
/** Check if the iterator finished the full iteration.

    See details in \ref rect_iter
 */
static int ri_end(rect_iter * i)
{
    /* if the current x value is larger than the largest
        x value in the rectangle (vx[2]), we know the full
        exploration of the rectangle is finished. */
    return (float)(i->x) > i->vx[2];
}

/*----------------------------------------------------------------------------*/
/** Increment a rectangle iterator.

    See details in \ref rect_iter
 */
static void ri_inc(rect_iter * i)
{
    /* if not at end of exploration,
        increase y value for next pixel in the 'column' */
    if( !ri_end(i) ) i->y++;

    /* if the end of the current 'column' is reached,
        and it is not the end of exploration,
        advance to the next 'column' */
    while( (float) (i->y) > i->ye && !ri_end(i) )
    {
        /* increase x, next 'column' */
        i->x++;

        /* if end of exploration, return */
        if( ri_end(i) ) return;

        if( (float) i->x < i->vx[3] )
        i->ys = inter_low((float)i->x,i->vx[0],i->vy[0],i->vx[3],i->vy[3]);
        else
        i->ys = inter_low((float)i->x,i->vx[3],i->vy[3],i->vx[2],i->vy[2]);

        if( (float)i->x < i->vx[1] )
        i->ye = inter_hi((float)i->x,i->vx[0],i->vy[0],i->vx[1],i->vy[1]);
        else
        i->ye = inter_hi((float)i->x,i->vx[1],i->vy[1],i->vx[2],i->vy[2]);

        /* new y */
        i->y = (int) ceil(i->ys);
    }
}

/*----------------------------------------------------------------------------*/
/** Create and initialize a rectangle iterator.
 */
static rect_iter * ri_ini(struct rect * r)
{
    float vx[4],vy[4];
    int n,offset;
    rect_iter * i;

    /* get memory */
    i = (rect_iter *) malloc(sizeof(rect_iter));

    /* build list of rectangle corners ordered
        in a circular way around the rectangle */
    vx[0] = r->x1 - r->dy * r->width * 0.5f;
    vy[0] = r->y1 + r->dx * r->width * 0.5f;
    vx[1] = r->x2 - r->dy * r->width * 0.5f;
    vy[1] = r->y2 + r->dx * r->width * 0.5f;
    vx[2] = r->x2 + r->dy * r->width * 0.5f;
    vy[2] = r->y2 - r->dx * r->width * 0.5f;
    vx[3] = r->x1 + r->dy * r->width * 0.5f;
    vy[3] = r->y1 - r->dx * r->width * 0.5f;

    if( r->x1 < r->x2 && r->y1 <= r->y2 ) offset = 0;
    else if( r->x1 >= r->x2 && r->y1 < r->y2 ) offset = 1;
    else if( r->x1 > r->x2 && r->y1 >= r->y2 ) offset = 2;
    else offset = 3;

    /* apply rotation of index. */
    for(n=0; n<4; n++)
    {
        i->vx[n] = vx[(offset+n)%4];
        i->vy[n] = vy[(offset+n)%4];
    }

    i->x = (int) ceil(i->vx[0]) - 1;
    i->y = (int) ceil(i->vy[0]);
    i->ys = i->ye = -INF;

    /* advance to the first pixel */
    ri_inc(i);

    return i;
}

/*----------------------------------------------------------------------------*/
/** Compute a rectangle's NFA value.
 */
static float rect_nfa(struct rect * rec, image_float_p angles, float logNT)
{
    rect_iter * i;
    int pts = 0;
    int alg = 0;

    /* compute the total number of pixels and of aligned points in 'rec' */
    for(i=ri_ini(rec); !ri_end(i); ri_inc(i)) /* rectangle iterator */
    if( i->x >= 0 && i->y >= 0 &&
        i->x < (int) angles->xsize && i->y < (int) angles->ysize )
        {
        ++pts; /* total number of pixels counter */
        if( isaligned(i->x, i->y, angles, rec->theta, rec->prec) )
            ++alg; /* aligned points counter */
        }
    ri_del(i); /* delete iterator */

    return nfa(pts,alg,rec->p,logNT); /* compute NFA value */
}


/*----------------------------------------------------------------------------*/
/** Compute region's angle as the principal inertia axis of the region.
 */
static float get_theta( struct point * reg, int reg_size, float x, float y,
                         image_float_p modgrad, float reg_angle, float prec )
{
    float lambda,theta,weight;
    float Ixx = 0.f;
    float Iyy = 0.f;
    float Ixy = 0.f;
    int i;

    /* compute inertia matrix */
    for(i=0; i<reg_size; i++)
    {
        weight = modgrad->data[ reg[i].x + reg[i].y * modgrad->xsize ];
        Ixx += ( (float) reg[i].y - y ) * ( (float) reg[i].y - y ) * weight;
        Iyy += ( (float) reg[i].x - x ) * ( (float) reg[i].x - x ) * weight;
        Ixy -= ( (float) reg[i].x - x ) * ( (float) reg[i].y - y ) * weight;
    }

    /* compute smallest eigenvalue */
    lambda = 0.5f * ( Ixx + Iyy - sqrt( (Ixx-Iyy)*(Ixx-Iyy) + 4.f*Ixy*Ixy ) );

    /* compute angle */
    theta = fabs(Ixx)>fabs(Iyy) ? atan2(lambda-Ixx,Ixy) : atan2(Ixy,lambda-Iyy);

    /* The previous procedure doesn't cares about orientation,
        so it could be wrong by 180 degrees. Here is corrected if necessary. */
    if( angle_diff(theta,reg_angle) > prec ) theta += PI;

    return theta;
}

/*----------------------------------------------------------------------------*/
/** Computes a rectangle that covers a region of points.
 */
static void region2rect( struct point * reg, int reg_size,
                         image_float_p modgrad, float reg_angle,
                         float prec, float p, struct rect * rec )
{
    float x,y,dx,dy,l,w,theta,weight,sum,l_min,l_max,w_min,w_max;
    int i;

    x = y = sum = 0.f;
    for(i=0; i<reg_size; i++)
    {
        weight = modgrad->data[ reg[i].x + reg[i].y * modgrad->xsize ];
        x += (float) reg[i].x * weight;
        y += (float) reg[i].y * weight;
        sum += weight;
    }

    x /= sum;
    y /= sum;

    /* theta */
    theta = get_theta(reg,reg_size,x,y,modgrad,reg_angle,prec);

    dx = cos(theta);
    dy = sin(theta);
    l_min = l_max = w_min = w_max = 0.f;
    for(i=0; i<reg_size; i++)
    {
        l =  ( (float) reg[i].x - x) * dx + ( (float) reg[i].y - y) * dy;
        w = -( (float) reg[i].x - x) * dy + ( (float) reg[i].y - y) * dx;

        if( l > l_max ) l_max = l;
        if( l < l_min ) l_min = l;
        if( w > w_max ) w_max = w;
        if( w < w_min ) w_min = w;
    }

    /* store values */
    rec->x1 = x + l_min * dx;
    rec->y1 = y + l_min * dy;
    rec->x2 = x + l_max * dx;
    rec->y2 = y + l_max * dy;
    rec->width = w_max - w_min;
    rec->x = x;
    rec->y = y;
    rec->theta = theta;
    rec->dx = dx;
    rec->dy = dy;
    rec->prec = prec;
    rec->p = p;

    if( rec->width < 1.f ) rec->width = 1.f;
}

/*----------------------------------------------------------------------------*/
/** Build a region of pixels that share the same angle, up to a
    tolerance 'prec', starting at point (x,y).
 */
static void region_grow( int x, int y, image_float_p angles, struct point * reg,
                         int * reg_size, float * reg_angle, image_char used,
                         float prec )
{
    float sumdx,sumdy;
    int xx,yy,i;

    /* first point of the region */
    *reg_size = 1;
    reg[0].x = x;
    reg[0].y = y;
    *reg_angle = angles->data[x+y*angles->xsize];  /* region's angle */
    sumdx = cos(*reg_angle);
    sumdy = sin(*reg_angle);
    used->data[x+y*used->xsize] = USED;

    /* try neighbors as new region points */
    for(i=0; i<*reg_size; i++)
        for(xx=reg[i].x-1; xx<=reg[i].x+1; xx++)
            for(yy=reg[i].y-1; yy<=reg[i].y+1; yy++)
                if( xx>=0 && yy>=0 && xx<(int)used->xsize && yy<(int)used->ysize &&
                    used->data[xx+yy*used->xsize] != USED &&
                    isaligned(xx,yy,angles,*reg_angle,prec) )
                {
                    /* add point */
                    used->data[xx+yy*used->xsize] = USED;
                    reg[*reg_size].x = xx;
                    reg[*reg_size].y = yy;
                    ++(*reg_size);

                    /* update region's angle */
                    sumdx += cos( angles->data[xx+yy*angles->xsize] );
                    sumdy += sin( angles->data[xx+yy*angles->xsize] );
                    *reg_angle = atan2(sumdy,sumdx);
                }
}

/*----------------------------------------------------------------------------*/
/** Try some rectangles variations to improve NFA value. Only if the
    rectangle is not meaningful (i.e., log_nfa <= log_eps).
 */
static float rect_improve( struct rect * rec, image_float_p angles,
                            float logNT, float log_eps )
{
    struct rect r;
    float log_nfa,log_nfa_new;
    float delta = 0.5f;
    float delta_2 = delta * 0.5f;
    int n;

    log_nfa = rect_nfa(rec,angles,logNT);

    if( log_nfa > log_eps ) return log_nfa;

    /* try finer precisions */
    rect_copy(rec,&r);
    for(n=0; n<5; n++)
    {
        r.p /= 2.f;
        r.prec = r.p * PI;
        log_nfa_new = rect_nfa(&r,angles,logNT);
        if( log_nfa_new > log_nfa )
        {
            log_nfa = log_nfa_new;
            rect_copy(&r,rec);
        }
    }

    if( log_nfa > log_eps ) return log_nfa;

    /* try to reduce width */
    rect_copy(rec,&r);
    for(n=0; n<5; n++)
    {
        if( (r.width - delta) >= 0.5f )
        {
            r.width -= delta;
            log_nfa_new = rect_nfa(&r,angles,logNT);
            if( log_nfa_new > log_nfa )
            {
                rect_copy(&r,rec);
                log_nfa = log_nfa_new;
            }
        }
    }

    if( log_nfa > log_eps ) return log_nfa;

    /* try to reduce one side of the rectangle */
    rect_copy(rec,&r);
    for(n=0; n<5; n++)
    {
        if( (r.width - delta) >= 0.5f )
        {
            r.x1 += -r.dy * delta_2;
            r.y1 +=  r.dx * delta_2;
            r.x2 += -r.dy * delta_2;
            r.y2 +=  r.dx * delta_2;
            r.width -= delta;
            log_nfa_new = rect_nfa(&r,angles,logNT);
            if( log_nfa_new > log_nfa )
            {
                rect_copy(&r,rec);
                log_nfa = log_nfa_new;
            }
        }
    }

    if( log_nfa > log_eps ) return log_nfa;

    /* try to reduce the other side of the rectangle */
    rect_copy(rec,&r);
    for(n=0; n<5; n++)
    {
        if( (r.width - delta) >= 0.5f )
        {
            r.x1 -= -r.dy * delta_2;
            r.y1 -=  r.dx * delta_2;
            r.x2 -= -r.dy * delta_2;
            r.y2 -=  r.dx * delta_2;
            r.width -= delta;
            log_nfa_new = rect_nfa(&r,angles,logNT);
            if( log_nfa_new > log_nfa )
            {
                rect_copy(&r,rec);
                log_nfa = log_nfa_new;
            }
        }
    }

    if( log_nfa > log_eps ) return log_nfa;

    /* try even finer precisions */
    rect_copy(rec,&r);
    for(n=0; n<5; n++)
    {
        r.p *= 0.5f;
        r.prec = r.p * PI;
        log_nfa_new = rect_nfa(&r,angles,logNT);
        if( log_nfa_new > log_nfa )
        {
            log_nfa = log_nfa_new;
            rect_copy(&r,rec);
        }
    }

    return log_nfa;
}

/*----------------------------------------------------------------------------*/
/** Reduce the region size, by elimination the points far from the
    starting point, until that leads to rectangle with the right
    density of region points or to discard the region if too small.
 */
static int reduce_region_radius( struct point * reg, int * reg_size,
                                 image_float_p modgrad, float reg_angle,
                                 float prec, float p, struct rect * rec,
                                 image_char used, 
                                 float density_th )
{
    float density,rad1,rad2,rad,xc,yc;
    int i;

    /* compute region points density */
    density = (float) *reg_size /
                            ( dist(rec->x1,rec->y1,rec->x2,rec->y2) * rec->width );

    /* if the density criterion is satisfied there is nothing to do */
    if( density >= density_th ) return TRUE;

    /* compute region's radius */
    xc = (float) reg[0].x;
    yc = (float) reg[0].y;
    rad1 = dist( xc, yc, rec->x1, rec->y1 );
    rad2 = dist( xc, yc, rec->x2, rec->y2 );
    rad = rad1 > rad2 ? rad1 : rad2;

    /* while the density criterion is not satisfied, remove farther pixels */
    while( density < density_th )
    {
        rad *= 0.75f; /* reduce region's radius to 75% of its value */

        /* remove points from the region and update 'used' map */
        for(i=0; i<*reg_size; i++)
        if( dist( xc, yc, (float) reg[i].x, (float) reg[i].y ) > rad )
            {
            /* point not kept, mark it as NOTUSED */
            used->data[ reg[i].x + reg[i].y * used->xsize ] = NOTUSED;
            /* remove point from the region */
            reg[i].x = reg[*reg_size-1].x; /* if i==*reg_size-1 copy itself */
            reg[i].y = reg[*reg_size-1].y;
            --(*reg_size);
            --i; /* to avoid skipping one point */
            }

        /* reject if the region is too small.
            2 is the minimal region size for 'region2rect' to work. */
        if( *reg_size < 2 ) return FALSE; 

        /* re-compute rectangle */
        region2rect(reg,*reg_size,modgrad,reg_angle,prec,p,rec);

        /* re-compute region points density */
        density = (float) *reg_size /
                            ( dist(rec->x1,rec->y1,rec->x2,rec->y2) * rec->width );
    }

    /* if this point is reached, the density criterion is satisfied */
    return TRUE;
}

/*----------------------------------------------------------------------------*/
/** Refine a rectangle.
 */
static int refine( struct point * reg, int * reg_size, image_float_p modgrad,
                   float reg_angle, float prec, float p, struct rect * rec,
                   image_char used, image_float_p angles, float density_th )
{
    float angle,ang_d,mean_angle,tau,density,xc,yc,ang_c,sum,s_sum;
    int i,n;

    /* compute region points density */
    density = (float) *reg_size /
                            ( dist(rec->x1,rec->y1,rec->x2,rec->y2) * rec->width );

    /* if the density criterion is satisfied there is nothing to do */
    if( density >= density_th ) return TRUE;

    /*------ First try: reduce angle tolerance ------*/
    /* compute the new mean angle and tolerance */
    xc = (float) reg[0].x;
    yc = (float) reg[0].y;
    ang_c = angles->data[ reg[0].x + reg[0].y * angles->xsize ];
    sum = s_sum = 0.f;
    n = 0;
    for(i=0; i<*reg_size; i++)
    {
        used->data[ reg[i].x + reg[i].y * used->xsize ] = NOTUSED;
        if( dist( xc, yc, (float) reg[i].x, (float) reg[i].y ) < rec->width )
        {
            angle = angles->data[ reg[i].x + reg[i].y * angles->xsize ];
            ang_d = angle_diff_signed(angle,ang_c);
            sum += ang_d;
            s_sum += ang_d * ang_d;
            ++n;
        }
    }
    mean_angle = sum / (float) n;
    tau = 2.f * sqrt( (s_sum - 2.f * mean_angle * sum) / (float) n
                            + mean_angle*mean_angle ); /* 2 * standard deviation */

    /* find a new region from the same starting point and new angle tolerance */
    region_grow(reg[0].x,reg[0].y,angles,reg,reg_size,&reg_angle,used,tau);

    /* if the region is too small, reject */
    if( *reg_size < 2 ) return FALSE;

    /* re-compute rectangle */
    region2rect(reg,*reg_size,modgrad,reg_angle,prec,p,rec);

    /* re-compute region points density */
    density = (float) *reg_size /
                        ( dist(rec->x1,rec->y1,rec->x2,rec->y2) * rec->width );

    /*------ Second try: reduce region radius ------*/
    if( density < density_th )
    return reduce_region_radius( reg, reg_size, modgrad, reg_angle, prec, p,
                                    rec, used, density_th );

    /* if this point is reached, the density criterion is satisfied */
    return TRUE;
}


/*----------------------------------------------------------------------------*/
/*-------------------------- Line Segment Detector ---------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/** LSD full interface.
 */
float * LineSegmentDetection(int * n_out,
    unsigned char * src, int w, int h,
    float scale_x, float scale_y, float sigma_scale, float quant,
    float ang_th, float log_eps, float density_th,
    int n_bins,
    int ** reg_img, int * reg_x, int * reg_y)
{
    image_float_p image = NULL;
    ntuple_list out = new_ntuple_list(7);
    float * return_value = NULL;
    image_float_p scaled_image = NULL, angles = NULL, modgrad = NULL;
    image_char used = NULL;
    image_int region = NULL;
    struct coorlist * list_p = NULL;
    void * mem_p = NULL;
    struct rect rec;
    struct point * reg = NULL;
    int reg_size, min_reg_size, i;
    unsigned int xsize, ysize;
    float rho, reg_angle, prec, p, log_nfa, logNT;//, iscale;
    float iscale_x,iscale_y;
    int ls_count = 0;                   /* line segments are numbered 1,2,3,... */

    //unsigned char transform to float
    unsigned char *psrc = NULL;
    float *img = NULL, *pimg = NULL;
    img = (float*)malloc(w*h*sizeof(float));
    if (!img)exit(1);

    pimg = img;
    psrc = src;

    for (i = 0; i < w*h; i++)
        *pimg++ = (float)*psrc++;

    psrc = NULL;
    pimg = NULL;

    /* angle tolerance */
    prec = RADIAN_1 * ang_th;
    p = ang_th * 0.0055556f;   // 1/180
    rho = quant / sin(prec); /* gradient magnitude threshold */

    iscale_x = 1.f / scale_x;
    iscale_y = 1.f / scale_y;

    /* load and scale image (if necessary) and compute angle at each pixel */
    image = new_image_float_ptr((unsigned int)w, (unsigned int)h, img);
    if ((scale_x != 1.f) || (scale_y != 1.f))
    {
        scaled_image = gaussian_sampler(image, scale_x,scale_y, sigma_scale);

        angles = ll_angle(scaled_image, rho, &list_p, &mem_p,
            &modgrad, (unsigned int)n_bins);
        free_image_float(scaled_image);
    }
    else
        angles = ll_angle(image, rho, &list_p, &mem_p, &modgrad,
        (unsigned int)n_bins);
    xsize = angles->xsize;
    ysize = angles->ysize;

    logNT = 2.5f * (log10((float)xsize) + log10((float)ysize))
        + log10(11.f);
    min_reg_size = (int)(-logNT / log10(p)); /* minimal number of points in region
                                                that can give a meaningful event */

    /* initialize some structures */
    if (reg_img != NULL && reg_x != NULL && reg_y != NULL) /* save region data */
        region = new_image_int_ini(angles->xsize, angles->ysize, 0);
    used = new_image_char_ini(xsize, ysize, NOTUSED);
    reg = (struct point *) calloc((size_t)(xsize*ysize), sizeof(struct point));
    if (!reg)exit(1);

    /* search for line segments */
    for (; list_p != NULL; list_p = list_p->next)
    if (used->data[list_p->x + list_p->y * used->xsize] == NOTUSED &&
        angles->data[list_p->x + list_p->y * angles->xsize] != NOTDEF)
    {
        /* find the region of connected point and ~equal angle */
        region_grow(list_p->x, list_p->y, angles, reg, &reg_size,
            &reg_angle, used, prec);

        /* reject small regions */
        if (reg_size < min_reg_size) continue;

        /* construct rectangular approximation for the region */
        region2rect(reg, reg_size, modgrad, reg_angle, prec, p, &rec);

        if (!refine(reg, &reg_size, modgrad, reg_angle,
            prec, p, &rec, used, angles, density_th)) continue;

        /* compute NFA value */
        log_nfa = rect_improve(&rec, angles, logNT, log_eps);
        if (log_nfa <= log_eps) continue;

        /* A New Line Segment was found! */
        ++ls_count;  /* increase line segment counter */

        rec.x1 += 0.5f; rec.y1 += 0.5f;
        rec.x2 += 0.5f; rec.y2 += 0.5f;

        /* scale the result values if a subsampling was performed */

        if ((scale_x != 1.f) && (scale_y != 1.f))
        {
            rec.x1 *= iscale_x; rec.y1 *= iscale_y;
            rec.x2 *= iscale_x; rec.y2 *= iscale_y;
            rec.width *= iscale_x;
        }
        else
        if ((scale_x == 1.f) && (scale_y != 1.f))
        {
            rec.y1 *= iscale_y;
            rec.y2 *= iscale_y;
        }
        else
        if ((scale_x != 1.f) && (scale_y == 1.f))
        {
            rec.x1 *= iscale_x;
            rec.x2 *= iscale_x;
            rec.width *= iscale_x;
        }

        /* add line segment found to output */
        add_7tuple(out, rec.x1, rec.y1, rec.x2, rec.y2,
            rec.width, rec.p, log_nfa);

        /* add region number to 'region' image if needed */
        if (region != NULL)
        for (i = 0; i<reg_size; i++)
            region->data[reg[i].x + reg[i].y * region->xsize] = ls_count;
    }


    /* free memory */
    if (image){ free((void *)image); image = NULL; }   /* only the float_image structure should be freed,
                                                       the data pointer was provided to this functions
                                                       and should not be destroyed.                 */
    free_image_float(angles);
    free_image_float(modgrad);
    free_image_char(used);
    if (reg){ free((void *)reg); reg = NULL; }
    if (mem_p){ free((void *)mem_p); mem_p = NULL; }

    /* return the result */
    if (reg_img != NULL && reg_x != NULL && reg_y != NULL)
    {
        *reg_img = region->data;

        *reg_x = (int)(region->xsize);
        *reg_y = (int)(region->ysize);

        if (region){ free((void *)region); region = NULL; }
    }

    *n_out = (int)(out->size);

    return_value = out->values;
    if (out){ free((void *)out); out = NULL; } /* only the 'ntuple_list' structure must be freed,
                                               but the 'values' pointer must be keep to return
                                               as a result. */

    if (img){ free(img); img = NULL; }

    return return_value;
}

/*----------------------------------------------------------------------------*/
/** LSD Simple Interface with Scale and Region output.
 */
float * lsd_scale_region( int * n_out,
                           unsigned char * image, int w, int h, float scale_x, float scale_y,
                           int ** reg_img, int * reg_x, int * reg_y )
{
    /* LSD parameters */
    float sigma_scale = 0.6f; /* Sigma for Gaussian filter is computed as
                                sigma = sigma_scale/scale.                    */
    float quant = 2.0f;       /* Bound to the quantization error on the
                                gradient norm.                                */
    float ang_th = 22.5f;     /* Gradient angle tolerance in degrees.           */
    float log_eps = 0.f;     /* Detection threshold: -log10(NFA) > log_eps     */
    float density_th = 0.7f;  /* Minimal density of region points in rectangle. */
    int n_bins = 1024;        /* Number of bins in pseudo-ordering of gradient
                                modulus.                                       */

    return LineSegmentDetection( n_out, image, w, h, scale_x, scale_y, sigma_scale, quant,
                                ang_th, log_eps, density_th, n_bins,
                                reg_img, reg_x, reg_y );
}

/*----------------------------------------------------------------------------*/
/** LSD Simple Interface with Scale.
 */
float * lsd_scale(int * n_out, unsigned char * image, int w, int h, float scale_x, float scale_y)
{
    return lsd_scale_region(n_out, image, w, h, scale_x, scale_y, NULL, NULL, NULL);
}

/*----------------------------------------------------------------------------*/
/** LSD Simple Interface.
 */
float * lsd(int * n_out, unsigned char * image, int w, int h)
{
    /* LSD parameters */
    float scale_x = 1.f;       /* Scale the image by Gaussian filter to 'scale'. */
    float scale_y = 1.f;

    return lsd_scale(n_out, image, w, h, scale_x, scale_y);
}
/*----------------------------------------------------------------------------*/


