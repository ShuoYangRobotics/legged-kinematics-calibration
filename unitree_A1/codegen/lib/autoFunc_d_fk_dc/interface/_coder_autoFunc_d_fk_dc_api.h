/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_autoFunc_d_fk_dc_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 10-Aug-2021 14:50:52
 */

#ifndef _CODER_AUTOFUNC_D_FK_DC_API_H
#define _CODER_AUTOFUNC_D_FK_DC_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void autoFunc_dJ_dpho(real_T in1[3], real_T in2[3], real_T in3[5], real_T
                        dJ_dpho[27]);
  void autoFunc_dJ_dpho_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
  void autoFunc_dJ_dq(real_T in1[3], real_T in2[3], real_T in3[5], real_T dJ_dq
                      [27]);
  void autoFunc_dJ_dq_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
  void autoFunc_d_fk_dc(real_T in1[3], real_T in2[3], real_T in3[5], real_T
                        d_fk_dc[9]);
  void autoFunc_d_fk_dc_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
  void autoFunc_d_fk_dc_atexit(void);
  void autoFunc_d_fk_dc_initialize(void);
  void autoFunc_d_fk_dc_terminate(void);
  void autoFunc_d_fk_dc_xil_shutdown(void);
  void autoFunc_d_fk_dc_xil_terminate(void);
  void autoFunc_d_fk_dq(real_T in1[3], real_T in2[3], real_T in3[5], real_T
                        jacobian[9]);
  void autoFunc_d_fk_dq_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
  void autoFunc_fk_derive(real_T in1[3], real_T in2[3], real_T in3[5], real_T
    p_bf[3]);
  void autoFunc_fk_derive_api(const mxArray * const prhs[3], const mxArray *
    plhs[1]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_autoFunc_d_fk_dc_api.h
 *
 * [EOF]
 */
