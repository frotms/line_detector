#ifndef _DEFINES_H_
#define _DEFINES_H_

#ifndef TRUE
#define  TRUE			    (1)
#endif

#ifndef FALSE
#define  FALSE			    (0)
#endif

#ifndef MIN
#define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

#ifndef X_EPS
#define X_EPS (1.192092896e-07F)
#endif /* NULL */

//angle to arc
#ifndef RADIAN_1
#define  RADIAN_1 (1.745329252e-2F)
#endif

/** 1.5pi */
#ifndef PI_3_2
#define  PI_3_2 (4.712388980F) 
#endif

/** 2pi */
#ifndef PI_2
#define  PI_2 (6.283185307F) 
#endif

/* 0.5pi */
#ifndef PI_1_2
//#define PI_1_2 (1.5707963F)  //1.5707963267948966192313216916398
#define PI_1_2 (1.570796327F) 
#endif

#ifndef ROUND
#define ROUND (0.5F)
#endif


/* Define NULL pointer value */
#ifndef NULL
#ifdef __cplusplus
#define NULL    0
#else  /* __cplusplus */
#define NULL    ((void *)0)
#endif  /* __cplusplus */
#endif  /* NULL */


#ifndef INF
#define INF         (3.402823466e+38F)        /* max value */
#endif /* INF */

#ifndef PI
#define PI (3.141592654F)
#endif


#ifndef ZERO
#define ZERO (0)
#endif
//
#ifndef ONE
#define ONE (1)
#endif


#endif