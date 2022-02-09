//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: autoFunc_d_fk_drho.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 08-Feb-2022 22:07:29
//

// Include Files
#include "autoFunc_d_fk_drho.h"
#include <cmath>

// Function Definitions
//
// AUTOFUNC_D_FK_DRHO
//     D_FK_DRHO = AUTOFUNC_D_FK_DRHO(IN1,LC,IN3)
//
// Arguments    : const double in1[3]
//                double lc
//                const double in3[4]
//                double d_fk_drho[3]
// Return Type  : void
//
void autoFunc_d_fk_drho(const double in1[3], double, const double[4],
                        double d_fk_drho[3])
{
  double t5;
  double t6;
  //     This function was generated by the Symbolic Math Toolbox version 8.7.
  //     08-Feb-2022 22:07:24
  t5 = in1[1] + in1[2];
  t6 = std::cos(t5);
  d_fk_drho[0] = -std::sin(t5);
  d_fk_drho[1] = t6 * std::sin(in1[0]);
  d_fk_drho[2] = -t6 * std::cos(in1[0]);
}

//
// File trailer for autoFunc_d_fk_drho.cpp
//
// [EOF]
//
