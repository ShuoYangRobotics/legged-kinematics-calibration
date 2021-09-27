//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 10-Aug-2021 14:50:52
//

//***********************************************************************
// This automatically generated example C++ main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************

// Include Files
#include "main.h"
#include "autoFunc_dJ_dpho.h"
#include "autoFunc_dJ_dq.h"
#include "autoFunc_d_fk_dc.h"
#include "autoFunc_d_fk_dc_terminate.h"
#include "autoFunc_d_fk_dq.h"
#include "autoFunc_fk_derive.h"

// Function Declarations
static void argInit_3x1_real_T(double result[3]);
static void argInit_5x1_real_T(double result[5]);
static double argInit_real_T();
static void main_autoFunc_dJ_dpho();
static void main_autoFunc_dJ_dq();
static void main_autoFunc_d_fk_dc();
static void main_autoFunc_d_fk_dq();
static void main_autoFunc_fk_derive();

// Function Definitions
//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_3x1_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[5]
// Return Type  : void
//
static void argInit_5x1_real_T(double result[5])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 5; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_autoFunc_dJ_dpho()
{
  double dJ_dpho[27];
  double dv[5];
  double in1_tmp[3];

  // Initialize function 'autoFunc_dJ_dpho' input arguments.
  // Initialize function input argument 'in1'.
  argInit_3x1_real_T(in1_tmp);

  // Initialize function input argument 'in2'.
  // Initialize function input argument 'in3'.
  // Call the entry-point 'autoFunc_dJ_dpho'.
  argInit_5x1_real_T(dv);
  autoFunc_dJ_dpho(in1_tmp, in1_tmp, dv, dJ_dpho);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_autoFunc_dJ_dq()
{
  double dJ_dq[27];
  double dv[5];
  double in1_tmp[3];

  // Initialize function 'autoFunc_dJ_dq' input arguments.
  // Initialize function input argument 'in1'.
  argInit_3x1_real_T(in1_tmp);

  // Initialize function input argument 'in2'.
  // Initialize function input argument 'in3'.
  // Call the entry-point 'autoFunc_dJ_dq'.
  argInit_5x1_real_T(dv);
  autoFunc_dJ_dq(in1_tmp, in1_tmp, dv, dJ_dq);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_autoFunc_d_fk_dc()
{
  double d_fk_dc[9];
  double dv[5];
  double in1_tmp[3];

  // Initialize function 'autoFunc_d_fk_dc' input arguments.
  // Initialize function input argument 'in1'.
  argInit_3x1_real_T(in1_tmp);

  // Initialize function input argument 'in2'.
  // Initialize function input argument 'in3'.
  // Call the entry-point 'autoFunc_d_fk_dc'.
  argInit_5x1_real_T(dv);
  autoFunc_d_fk_dc(in1_tmp, in1_tmp, dv, d_fk_dc);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_autoFunc_d_fk_dq()
{
  double jacobian[9];
  double dv[5];
  double in1_tmp[3];

  // Initialize function 'autoFunc_d_fk_dq' input arguments.
  // Initialize function input argument 'in1'.
  argInit_3x1_real_T(in1_tmp);

  // Initialize function input argument 'in2'.
  // Initialize function input argument 'in3'.
  // Call the entry-point 'autoFunc_d_fk_dq'.
  argInit_5x1_real_T(dv);
  autoFunc_d_fk_dq(in1_tmp, in1_tmp, dv, jacobian);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_autoFunc_fk_derive()
{
  double dv[5];
  double in1_tmp[3];
  double p_bf[3];

  // Initialize function 'autoFunc_fk_derive' input arguments.
  // Initialize function input argument 'in1'.
  argInit_3x1_real_T(in1_tmp);

  // Initialize function input argument 'in2'.
  // Initialize function input argument 'in3'.
  // Call the entry-point 'autoFunc_fk_derive'.
  argInit_5x1_real_T(dv);
  autoFunc_fk_derive(in1_tmp, in1_tmp, dv, p_bf);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. 
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_autoFunc_d_fk_dc();
  main_autoFunc_d_fk_dq();
  main_autoFunc_dJ_dpho();
  main_autoFunc_dJ_dq();
  main_autoFunc_fk_derive();

  // Terminate the application.
  // You do not need to do this more than one time.
  autoFunc_d_fk_dc_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
