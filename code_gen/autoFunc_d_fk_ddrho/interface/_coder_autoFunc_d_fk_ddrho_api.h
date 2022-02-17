//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_autoFunc_d_fk_ddrho_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 17-Feb-2022 16:35:52
//

#ifndef _CODER_AUTOFUNC_D_FK_DDRHO_API_H
#define _CODER_AUTOFUNC_D_FK_DDRHO_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void autoFunc_d_fk_ddrho(real_T in1[3], real_T lc, real_T in3[4],
                         real_T d_fk_ddrho[3]);

void autoFunc_d_fk_ddrho_api(const mxArray *const prhs[3],
                             const mxArray **plhs);

void autoFunc_d_fk_ddrho_atexit();

void autoFunc_d_fk_ddrho_initialize();

void autoFunc_d_fk_ddrho_terminate();

void autoFunc_d_fk_ddrho_xil_shutdown();

void autoFunc_d_fk_ddrho_xil_terminate();

#endif
//
// File trailer for _coder_autoFunc_d_fk_ddrho_api.h
//
// [EOF]
//
