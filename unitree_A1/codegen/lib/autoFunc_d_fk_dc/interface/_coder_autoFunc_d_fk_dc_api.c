/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_autoFunc_d_fk_dc_api.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 10-Aug-2021 14:50:52
 */

/* Include Files */
#include "_coder_autoFunc_d_fk_dc_api.h"
#include "_coder_autoFunc_d_fk_dc_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131595U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "autoFunc_d_fk_dc",                  /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static const mxArray *b_emlrt_marshallOut(const real_T u[27]);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *in3,
  const char_T *identifier))[5];
static const mxArray *c_emlrt_marshallOut(const real_T u[3]);
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[5];
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *in1, const
  char_T *identifier))[3];
static const mxArray *emlrt_marshallOut(const real_T u[9]);
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[5];

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[3]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3]
{
  real_T (*y)[3];
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const real_T u[27]
 * Return Type  : const mxArray *
 */
  static const mxArray *b_emlrt_marshallOut(const real_T u[27])
{
  static const int32_T iv[2] = { 0, 0 };

  static const int32_T iv1[2] = { 9, 3 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, iv1, 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *in3
 *                const char_T *identifier
 * Return Type  : real_T (*)[5]
 */
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *in3,
  const char_T *identifier))[5]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[5];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(in3), &thisId);
  emlrtDestroyArray(&in3);
  return y;
}
/*
 * Arguments    : const real_T u[3]
 * Return Type  : const mxArray *
 */
  static const mxArray *c_emlrt_marshallOut(const real_T u[3])
{
  static const int32_T iv[1] = { 0 };

  static const int32_T iv1[1] = { 3 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, iv1, 1);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[5]
 */
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[5]
{
  real_T (*y)[5];
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[3]
 */
  static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims[1] = { 3 };

  real_T (*ret)[3];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *in1
 *                const char_T *identifier
 * Return Type  : real_T (*)[3]
 */
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *in1, const
  char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(in1), &thisId);
  emlrtDestroyArray(&in1);
  return y;
}
/*
 * Arguments    : const real_T u[9]
 * Return Type  : const mxArray *
 */
  static const mxArray *emlrt_marshallOut(const real_T u[9])
{
  static const int32_T iv[2] = { 0, 0 };

  static const int32_T iv1[2] = { 3, 3 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, iv1, 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[5]
 */
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[5]
{
  static const int32_T dims[1] = { 5 };

  real_T (*ret)[5];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[5])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
  void autoFunc_dJ_dpho_api(const mxArray * const prhs[3], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*dJ_dpho)[27];
  real_T (*in3)[5];
  real_T (*in1)[3];
  real_T (*in2)[3];
  st.tls = emlrtRootTLSGlobal;
  dJ_dpho = (real_T (*)[27])mxMalloc(sizeof(real_T [27]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "in1");
  in2 = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "in2");
  in3 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "in3");

  /* Invoke the target function */
  autoFunc_dJ_dpho(*in1, *in2, *in3, *dJ_dpho);

  /* Marshall function outputs */
  plhs[0] = b_emlrt_marshallOut(*dJ_dpho);
}

/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void autoFunc_dJ_dq_api(const mxArray * const prhs[3], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*dJ_dq)[27];
  real_T (*in3)[5];
  real_T (*in1)[3];
  real_T (*in2)[3];
  st.tls = emlrtRootTLSGlobal;
  dJ_dq = (real_T (*)[27])mxMalloc(sizeof(real_T [27]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "in1");
  in2 = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "in2");
  in3 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "in3");

  /* Invoke the target function */
  autoFunc_dJ_dq(*in1, *in2, *in3, *dJ_dq);

  /* Marshall function outputs */
  plhs[0] = b_emlrt_marshallOut(*dJ_dq);
}

/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void autoFunc_d_fk_dc_api(const mxArray * const prhs[3], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*d_fk_dc)[9];
  real_T (*in3)[5];
  real_T (*in1)[3];
  real_T (*in2)[3];
  st.tls = emlrtRootTLSGlobal;
  d_fk_dc = (real_T (*)[9])mxMalloc(sizeof(real_T [9]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "in1");
  in2 = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "in2");
  in3 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "in3");

  /* Invoke the target function */
  autoFunc_d_fk_dc(*in1, *in2, *in3, *d_fk_dc);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*d_fk_dc);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void autoFunc_d_fk_dc_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  autoFunc_d_fk_dc_xil_terminate();
  autoFunc_d_fk_dc_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void autoFunc_d_fk_dc_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void autoFunc_d_fk_dc_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void autoFunc_d_fk_dq_api(const mxArray * const prhs[3], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*jacobian)[9];
  real_T (*in3)[5];
  real_T (*in1)[3];
  real_T (*in2)[3];
  st.tls = emlrtRootTLSGlobal;
  jacobian = (real_T (*)[9])mxMalloc(sizeof(real_T [9]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "in1");
  in2 = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "in2");
  in3 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "in3");

  /* Invoke the target function */
  autoFunc_d_fk_dq(*in1, *in2, *in3, *jacobian);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*jacobian);
}

/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void autoFunc_fk_derive_api(const mxArray * const prhs[3], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*in3)[5];
  real_T (*in1)[3];
  real_T (*in2)[3];
  real_T (*p_bf)[3];
  st.tls = emlrtRootTLSGlobal;
  p_bf = (real_T (*)[3])mxMalloc(sizeof(real_T [3]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "in1");
  in2 = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "in2");
  in3 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "in3");

  /* Invoke the target function */
  autoFunc_fk_derive(*in1, *in2, *in3, *p_bf);

  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*p_bf);
}

/*
 * File trailer for _coder_autoFunc_d_fk_dc_api.c
 *
 * [EOF]
 */
