//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_autoFunc_dJ_dt_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 17-Feb-2022 16:35:53
//

#ifndef _CODER_AUTOFUNC_DJ_DT_API_H
#define _CODER_AUTOFUNC_DJ_DT_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void autoFunc_dJ_dt(real_T in1[3], real_T lc, real_T in3[4], real_T dJ_dq[27]);

void autoFunc_dJ_dt_api(const mxArray *const prhs[3], const mxArray **plhs);

void autoFunc_dJ_dt_atexit();

void autoFunc_dJ_dt_initialize();

void autoFunc_dJ_dt_terminate();

void autoFunc_dJ_dt_xil_shutdown();

void autoFunc_dJ_dt_xil_terminate();

#endif
//
// File trailer for _coder_autoFunc_dJ_dt_api.h
//
// [EOF]
//
