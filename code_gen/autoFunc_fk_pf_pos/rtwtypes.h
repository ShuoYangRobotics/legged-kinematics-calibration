//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rtwtypes.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 08-Feb-2022 22:07:25
//

#ifndef RTWTYPES_H
#define RTWTYPES_H

/*=======================================================================*
 * Fixed width word size data types:                                     *
 *   int64_T                      - signed 64 bit integers               *
 *   uint64_T                     - unsigned 64 bit integers             *
 *=======================================================================*/

#if defined(__APPLE__)
#ifndef INT64_T
#define INT64_T long
#define FMT64 "l"
#if defined(__LP64__) && !defined(INT_TYPE_64_IS_LONG)
#define INT_TYPE_64_IS_LONG
#endif
#endif
#endif

#if defined(__APPLE__)
#ifndef UINT64_T
#define UINT64_T unsigned long
#define FMT64 "l"
#if defined(__LP64__) && !defined(INT_TYPE_64_IS_LONG)
#define INT_TYPE_64_IS_LONG
#endif
#endif
#endif

// Include Files
#include "tmwtypes.h"

#endif
//
// File trailer for rtwtypes.h
//
// [EOF]
//
