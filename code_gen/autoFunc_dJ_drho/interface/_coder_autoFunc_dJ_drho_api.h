//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_autoFunc_dJ_drho_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 08-Feb-2022 22:07:34
//

#ifndef _CODER_AUTOFUNC_DJ_DRHO_API_H
#define _CODER_AUTOFUNC_DJ_DRHO_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void autoFunc_dJ_drho(real_T in1[3], real_T lc, real_T in3[4],
                      real_T dJ_drho[9]);

void autoFunc_dJ_drho_api(const mxArray *const prhs[3], const mxArray **plhs);

void autoFunc_dJ_drho_atexit();

void autoFunc_dJ_drho_initialize();

void autoFunc_dJ_drho_terminate();

void autoFunc_dJ_drho_xil_shutdown();

void autoFunc_dJ_drho_xil_terminate();

#endif
//
// File trailer for _coder_autoFunc_dJ_drho_api.h
//
// [EOF]
//
