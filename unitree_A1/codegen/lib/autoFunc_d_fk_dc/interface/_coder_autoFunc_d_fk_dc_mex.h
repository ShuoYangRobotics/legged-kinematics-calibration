/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_autoFunc_d_fk_dc_mex.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 10-Aug-2021 14:50:52
 */

#ifndef _CODER_AUTOFUNC_D_FK_DC_MEX_H
#define _CODER_AUTOFUNC_D_FK_DC_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void autoFunc_dJ_dpho_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
    const mxArray *prhs[3]);
  void autoFunc_dJ_dq_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
    const mxArray *prhs[3]);
  void autoFunc_d_fk_dc_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
    const mxArray *prhs[3]);
  void autoFunc_d_fk_dq_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
    const mxArray *prhs[3]);
  void autoFunc_fk_derive_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T
    nrhs, const mxArray *prhs[3]);
  MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T
    nrhs, const mxArray *prhs[]);
  emlrtCTX mexFunctionCreateRootTLS(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_autoFunc_d_fk_dc_mex.h
 *
 * [EOF]
 */
