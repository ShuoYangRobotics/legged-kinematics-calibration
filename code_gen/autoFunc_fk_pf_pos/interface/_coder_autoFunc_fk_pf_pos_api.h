//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_autoFunc_fk_pf_pos_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 17-Feb-2022 16:35:43
//

#ifndef _CODER_AUTOFUNC_FK_PF_POS_API_H
#define _CODER_AUTOFUNC_FK_PF_POS_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void autoFunc_fk_pf_pos(real_T in1[3], real_T lc, real_T in3[4],
                        real_T p_bf[3]);

void autoFunc_fk_pf_pos_api(const mxArray *const prhs[3], const mxArray **plhs);

void autoFunc_fk_pf_pos_atexit();

void autoFunc_fk_pf_pos_initialize();

void autoFunc_fk_pf_pos_terminate();

void autoFunc_fk_pf_pos_xil_shutdown();

void autoFunc_fk_pf_pos_xil_terminate();

#endif
//
// File trailer for _coder_autoFunc_fk_pf_pos_api.h
//
// [EOF]
//
