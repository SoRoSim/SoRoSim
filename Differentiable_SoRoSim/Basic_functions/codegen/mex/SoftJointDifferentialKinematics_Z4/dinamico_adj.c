/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dinamico_adj.c
 *
 * Code generation for function 'dinamico_adj'
 *
 */

/* Include files */
#include "dinamico_adj.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtBCInfo m_emlrtBCI =
    {
        -1,             /* iFirst */
        -1,             /* iLast */
        2,              /* lineNo */
        17,             /* colNo */
        "screw",        /* aName */
        "dinamico_adj", /* fName */
        "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
        "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\dinamico_"
        "adj.m", /* pName */
        0        /* checkKind */
};

static emlrtBCInfo n_emlrtBCI =
    {
        -1,             /* iFirst */
        -1,             /* iLast */
        3,              /* lineNo */
        17,             /* colNo */
        "screw",        /* aName */
        "dinamico_adj", /* fName */
        "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
        "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_functions\\dinamico_"
        "adj.m", /* pName */
        0        /* checkKind */
};

/* Function Definitions */
void dinamico_adj(const emlrtStack *sp, const real_T screw_data[],
                  int32_T screw_size, real_T adj[36])
{
  /*  optimized on 30.05.2022 */
  if (screw_size < 3) {
    emlrtDynamicBoundsCheckR2012b(3, 1, screw_size, &m_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (screw_size < 6) {
    emlrtDynamicBoundsCheckR2012b(6, 1, screw_size, &n_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  adj[0] = 0.0;
  adj[6] = -screw_data[2];
  adj[12] = screw_data[1];
  adj[18] = 0.0;
  adj[24] = 0.0;
  adj[30] = 0.0;
  adj[1] = screw_data[2];
  adj[7] = 0.0;
  adj[13] = -screw_data[0];
  adj[19] = 0.0;
  adj[25] = 0.0;
  adj[31] = 0.0;
  adj[2] = -screw_data[1];
  adj[8] = screw_data[0];
  adj[14] = 0.0;
  adj[20] = 0.0;
  adj[26] = 0.0;
  adj[32] = 0.0;
  adj[3] = 0.0;
  adj[9] = -screw_data[5];
  adj[15] = screw_data[4];
  adj[21] = 0.0;
  adj[27] = -screw_data[2];
  adj[33] = screw_data[1];
  adj[4] = screw_data[5];
  adj[10] = 0.0;
  adj[16] = -screw_data[3];
  adj[22] = screw_data[2];
  adj[28] = 0.0;
  adj[34] = -screw_data[0];
  adj[5] = -screw_data[4];
  adj[11] = screw_data[3];
  adj[17] = 0.0;
  adj[23] = -screw_data[1];
  adj[29] = screw_data[0];
  adj[35] = 0.0;
}

/* End of code generation (dinamico_adj.c) */
