//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: autoFunc_fk_pf_pos.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 17-Feb-2022 16:35:43
//

// Include Files
#include "autoFunc_fk_pf_pos.h"
#include <cmath>

// Function Definitions
//
// AUTOFUNC_FK_PF_POS
//     P_BF = AUTOFUNC_FK_PF_POS(IN1,LC,IN3)
//
// Arguments    : const double in1[3]
//                double lc
//                const double in3[4]
//                double p_bf[3]
// Return Type  : void
//
void autoFunc_fk_pf_pos(const double in1[3], double lc, const double in3[4],
                        double p_bf[3])
{
  double p_bf_tmp;
  double t10;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  //     This function was generated by the Symbolic Math Toolbox version 8.7.
  //     17-Feb-2022 16:35:31
  t5 = std::cos(in1[0]);
  t6 = std::cos(in1[1]);
  t7 = std::cos(in1[2]);
  t8 = std::sin(in1[0]);
  t9 = std::sin(in1[1]);
  t10 = std::sin(in1[2]);
  p_bf[0] = (in3[0] - in3[3] * t9) - lc * std::sin(in1[1] + in1[2]);
  p_bf[1] = (((in3[1] + in3[2] * t5) + in3[3] * t6 * t8) + lc * t6 * t7 * t8) -
            lc * t8 * t9 * t10;
  p_bf_tmp = lc * t5;
  p_bf[2] = ((in3[2] * t8 - in3[3] * t5 * t6) - p_bf_tmp * t6 * t7) +
            p_bf_tmp * t9 * t10;
}

//
// File trailer for autoFunc_fk_pf_pos.cpp
//
// [EOF]
//
