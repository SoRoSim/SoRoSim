/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dinamico_coadjbar.c
 *
 * Code generation for function 'dinamico_coadjbar'
 *
 */

/* Include files */
#include "dinamico_coadjbar.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtBCInfo e_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    2,                   /* lineNo */
    25,                  /* colNo */
    "screw",             /* aName */
    "dinamico_coadjbar", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\dinamico_"
    "coadjbar.m", /* pName */
    0             /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    2,                   /* lineNo */
    46,                  /* colNo */
    "screw",             /* aName */
    "dinamico_coadjbar", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\dinamico_"
    "coadjbar.m", /* pName */
    0             /* checkKind */
};

/* Function Definitions */
void dinamico_coadjbar(const emlrtStack *sp, const real_T screw_data[],
                       int32_T screw_size, real_T coadjbar[36])
{
  if (screw_size < 3) {
    emlrtDynamicBoundsCheckR2012b(3, 1, screw_size, &e_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (screw_size < 6) {
    emlrtDynamicBoundsCheckR2012b(6, 1, screw_size, &f_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  coadjbar[0] = -0.0;
  coadjbar[6] = screw_data[2];
  coadjbar[12] = -screw_data[1];
  coadjbar[18] = -0.0;
  coadjbar[24] = screw_data[5];
  coadjbar[30] = -screw_data[4];
  coadjbar[1] = -screw_data[2];
  coadjbar[7] = -0.0;
  coadjbar[13] = screw_data[0];
  coadjbar[19] = -screw_data[5];
  coadjbar[25] = -0.0;
  coadjbar[31] = screw_data[3];
  coadjbar[2] = screw_data[1];
  coadjbar[8] = -screw_data[0];
  coadjbar[14] = -0.0;
  coadjbar[20] = screw_data[4];
  coadjbar[26] = -screw_data[3];
  coadjbar[32] = -0.0;
  coadjbar[3] = -0.0;
  coadjbar[9] = screw_data[5];
  coadjbar[15] = -screw_data[4];
  coadjbar[21] = -0.0;
  coadjbar[27] = -0.0;
  coadjbar[33] = -0.0;
  coadjbar[4] = -screw_data[5];
  coadjbar[10] = -0.0;
  coadjbar[16] = screw_data[3];
  coadjbar[22] = -0.0;
  coadjbar[28] = -0.0;
  coadjbar[34] = -0.0;
  coadjbar[5] = screw_data[4];
  coadjbar[11] = -screw_data[3];
  coadjbar[17] = -0.0;
  coadjbar[23] = -0.0;
  coadjbar[29] = -0.0;
  coadjbar[35] = -0.0;
}

/* End of code generation (dinamico_coadjbar.c) */
