/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RigidJointDifferentialKinematics.c
 *
 * Code generation for function 'RigidJointDifferentialKinematics'
 *
 */

/* Include files */
#include "RigidJointDifferentialKinematics.h"
#include "RigidJointDifferentialKinematics_data.h"
#include "dinamico_adj.h"
#include "eml_mtimes_helper.h"
#include "eye.h"
#include "mtimes.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    5,                                  /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    7,                                  /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    98,                                 /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    99,                                 /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    105,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    123,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    124,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    138,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    154,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    163,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    164,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    165,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    166,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    181,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    182,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    183,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    197,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    213,                                /* lineNo */
    "RigidJointDifferentialKinematics", /* fcnName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pathName */
};

static emlrtRSInfo s_emlrtRSI =
    {
        94,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo t_emlrtRSI =
    {
        69,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtBCInfo emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    120,                                /* lineNo */
    44,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    120,                                /* lineNo */
    56,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtECInfo emlrtECI = {
    1,                                  /* nDims */
    123,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo b_emlrtECI = {
    2,                                  /* nDims */
    123,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo c_emlrtECI = {
    1,                                  /* nDims */
    124,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo d_emlrtECI = {
    2,                                  /* nDims */
    124,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtBCInfo c_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    136,                                /* lineNo */
    48,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    136,                                /* lineNo */
    60,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtECInfo e_emlrtECI = {
    1,                                  /* nDims */
    138,                                /* lineNo */
    32,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo f_emlrtECI = {
    2,                                  /* nDims */
    138,                                /* lineNo */
    32,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtBCInfo e_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    152,                                /* lineNo */
    48,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    152,                                /* lineNo */
    62,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtECInfo g_emlrtECI = {
    1,                                  /* nDims */
    154,                                /* lineNo */
    32,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo h_emlrtECI = {
    2,                                  /* nDims */
    154,                                /* lineNo */
    32,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo i_emlrtECI = {
    1,                                  /* nDims */
    163,                                /* lineNo */
    24,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo j_emlrtECI = {
    2,                                  /* nDims */
    163,                                /* lineNo */
    24,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo k_emlrtECI = {
    1,                                  /* nDims */
    164,                                /* lineNo */
    24,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo l_emlrtECI = {
    2,                                  /* nDims */
    164,                                /* lineNo */
    24,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo m_emlrtECI = {
    1,                                  /* nDims */
    165,                                /* lineNo */
    44,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo n_emlrtECI = {
    1,                                  /* nDims */
    165,                                /* lineNo */
    24,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo o_emlrtECI = {
    2,                                  /* nDims */
    165,                                /* lineNo */
    24,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtBCInfo g_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    178,                                /* lineNo */
    44,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    178,                                /* lineNo */
    56,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtECInfo p_emlrtECI = {
    1,                                  /* nDims */
    181,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo q_emlrtECI = {
    2,                                  /* nDims */
    181,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo r_emlrtECI = {
    1,                                  /* nDims */
    182,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo s_emlrtECI = {
    2,                                  /* nDims */
    182,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo t_emlrtECI = {
    1,                                  /* nDims */
    183,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo u_emlrtECI = {
    2,                                  /* nDims */
    183,                                /* lineNo */
    28,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtBCInfo i_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    195,                                /* lineNo */
    48,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    195,                                /* lineNo */
    60,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtECInfo v_emlrtECI = {
    1,                                  /* nDims */
    197,                                /* lineNo */
    32,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo w_emlrtECI = {
    2,                                  /* nDims */
    197,                                /* lineNo */
    32,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtBCInfo k_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    211,                                /* lineNo */
    48,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = {
    1,                                  /* iFirst */
    24,                                 /* iLast */
    211,                                /* lineNo */
    62,                                 /* colNo */
    "adjOmegap",                        /* aName */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m", /* pName */
    0            /* checkKind */
};

static emlrtECInfo x_emlrtECI = {
    1,                                  /* nDims */
    213,                                /* lineNo */
    32,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

static emlrtECInfo y_emlrtECI = {
    2,                                  /* nDims */
    213,                                /* lineNo */
    32,                                 /* colNo */
    "RigidJointDifferentialKinematics", /* fName */
    "D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
    "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
    "functions\\RigidJointDifferentialKin"
    "ematics.m" /* pName */
};

/* Function Declarations */
static void binary_expand_op(real_T in1_data[], int32_T in1_size[2],
                             const real_T in2_data[],
                             const int32_T in2_size[2]);

static void plus(real_T in1_data[], int32_T in1_size[2],
                 const real_T in2_data[], const int32_T in2_size[2]);

/* Function Definitions */
static void binary_expand_op(real_T in1_data[], int32_T in1_size[2],
                             const real_T in2_data[], const int32_T in2_size[2])
{
  real_T b_in1_data[600];
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_0;
  int32_T stride_1_1;
  if (in2_size[1] == 1) {
    loop_ub = in1_size[1];
  } else {
    loop_ub = in2_size[1];
  }
  stride_0_1 = (in1_size[1] != 1);
  stride_1_0 = (in2_size[0] != 1);
  stride_1_1 = (in2_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_in1_data[i1 + 6 * i] =
          in1_data[i1 + 6 * aux_0_1] -
          in2_data[i1 * stride_1_0 + in2_size[0] * aux_1_1];
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  in1_size[0] = 6;
  in1_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      stride_0_1 = i1 + 6 * i;
      in1_data[stride_0_1] = b_in1_data[stride_0_1];
    }
  }
}

static void plus(real_T in1_data[], int32_T in1_size[2],
                 const real_T in2_data[], const int32_T in2_size[2])
{
  real_T b_in1_data[600];
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_0;
  int32_T stride_1_1;
  if (in2_size[1] == 1) {
    loop_ub = in1_size[1];
  } else {
    loop_ub = in2_size[1];
  }
  stride_0_1 = (in1_size[1] != 1);
  stride_1_0 = (in2_size[0] != 1);
  stride_1_1 = (in2_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_in1_data[i1 + 6 * i] =
          in1_data[i1 + 6 * aux_0_1] +
          in2_data[i1 * stride_1_0 + in2_size[0] * aux_1_1];
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  in1_size[0] = 6;
  in1_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      stride_0_1 = i1 + 6 * i;
      in1_data[stride_0_1] = b_in1_data[stride_0_1];
    }
  }
}

void RigidJointDifferentialKinematics(
    const emlrtStack *sp, const real_T Phi_data[], const int32_T Phi_size[2],
    const real_T xi_star[6], const real_T q_data[], const int32_T q_size[1],
    const real_T qd_data[], const int32_T qd_size[1], const real_T qdd_data[],
    const int32_T qdd_size[1], real_T Omega[6], real_T Z_data[],
    int32_T Z_size[2], real_T g[16], real_T T[36], real_T S_data[],
    int32_T S_size[2], real_T Sd_data[], int32_T Sd_size[2], real_T f[4],
    real_T fd[4], real_T adjOmegap[144], real_T dSdq_qd_data[],
    int32_T dSdq_qd_size[2], real_T dSdq_qdd_data[], int32_T dSdq_qdd_size[2],
    real_T dSddq_qd_data[], int32_T dSddq_qd_size[2])
{
  static const int8_T b[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                               0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int8_T iv1[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  static const int8_T iv[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  __m128d r;
  __m128d r1;
  emlrtStack b_st;
  emlrtStack st;
  real_T b_tmp_data[2400];
  real_T e_y_data[2400];
  real_T a_data[144];
  real_T adjOmegapd[144];
  real_T b_y_data[144];
  real_T tmp_data[144];
  real_T adjOmegap1_data[108];
  real_T adjOmegap2_data[108];
  real_T f_data[108];
  real_T adjOmegap3_data[72];
  real_T adjOmegap4_data[72];
  real_T Td[36];
  real_T b_adjOmegapd[36];
  real_T y_data[24];
  real_T Omegahatp2[16];
  real_T Omegahatp3[16];
  real_T Omegad[6];
  real_T Zqdd[6];
  real_T b_Omegad[6];
  real_T fdd[4];
  real_T a;
  real_T sintheta;
  real_T t1;
  real_T t2;
  real_T theta;
  real_T thetad;
  real_T tp2;
  real_T tp4;
  int32_T a_size[2];
  int32_T adjOmegap2_size[2];
  int32_T adjOmegap3_size[2];
  int32_T adjOmegap4_size[2];
  int32_T b_tmp_size[2];
  int32_T f_size[2];
  int32_T tmp_size[2];
  int32_T T_tmp;
  int32_T Td_tmp;
  int32_T b_r;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub_tmp;
  int32_T p;
  int32_T u;
  int32_T vectorUB_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  st.site = &emlrtRSI;
  b_st.site = &t_emlrtRSI;
  dynamic_size_checks(&b_st, q_size[0], Phi_size[1], q_size[0]);
  mtimes(Phi_data, Phi_size, q_data, q_size[0], Omega);
  r = _mm_loadu_pd(&Omega[0]);
  _mm_storeu_pd(&Omega[0], _mm_add_pd(r, _mm_loadu_pd(&xi_star[0])));
  r = _mm_loadu_pd(&Omega[2]);
  _mm_storeu_pd(&Omega[2], _mm_add_pd(r, _mm_loadu_pd(&xi_star[2])));
  r = _mm_loadu_pd(&Omega[4]);
  _mm_storeu_pd(&Omega[4], _mm_add_pd(r, _mm_loadu_pd(&xi_star[4])));
  Z_size[0] = 6;
  Z_size[1] = Phi_size[1];
  loop_ub_tmp = 6 * Phi_size[1];
  if (loop_ub_tmp - 1 >= 0) {
    memcpy(&Z_data[0], &Phi_data[0], (uint32_T)loop_ub_tmp * sizeof(real_T));
  }
  st.site = &b_emlrtRSI;
  b_st.site = &t_emlrtRSI;
  dynamic_size_checks(&b_st, qd_size[0], Phi_size[1], qd_size[0]);
  mtimes(Phi_data, Phi_size, qd_data, qd_size[0], Omegad);
  /*     %% defining T, Td, f, fd, fdd, adjOmegap (powers of adjOmega) */
  fd[0] = 0.0;
  fdd[0] = 0.0;
  fd[1] = 0.0;
  fdd[1] = 0.0;
  fd[2] = 0.0;
  fdd[2] = 0.0;
  fd[3] = 0.0;
  fdd[3] = 0.0;
  /* all powers of adjGamma || INCLUDE POWER OF 0 to avoid an if */
  memset(&adjOmegap[0], 0, 144U * sizeof(real_T));
  memset(&adjOmegapd[0], 0, 144U * sizeof(real_T));
  /* derivative of adjGammap */
  theta = b_norm(&Omega[0]);
  thetad =
      ((Omegad[0] * Omega[0] + Omegad[1] * Omega[1]) + Omegad[2] * Omega[2]) /
      theta;
  /*  optimized on 30.05.2022 */
  g[0] = 0.0;
  g[4] = -Omega[2];
  g[8] = Omega[1];
  g[12] = Omega[3];
  g[1] = Omega[2];
  g[5] = 0.0;
  g[9] = -Omega[0];
  g[13] = Omega[4];
  g[2] = -Omega[1];
  g[6] = Omega[0];
  g[10] = 0.0;
  g[14] = Omega[5];
  g[3] = 0.0;
  g[7] = 0.0;
  g[11] = 0.0;
  g[15] = 0.0;
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      Omegahatp2[i + i2] =
          ((g[i] * g[i2] + g[i + 4] * g[i2 + 1]) + g[i + 8] * g[i2 + 2]) +
          g[i + 12] * g[i2 + 3];
    }
    t1 = Omegahatp2[i];
    t2 = Omegahatp2[i + 4];
    tp4 = Omegahatp2[i + 8];
    tp2 = Omegahatp2[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      Omegahatp3[i + i2] =
          ((t1 * g[i2] + t2 * g[i2 + 1]) + tp4 * g[i2 + 2]) + tp2 * g[i2 + 3];
    }
  }
  /*  optimized on 30.05.2022 */
  adjOmegap[0] = 0.0;
  adjOmegap[24] = -Omega[2];
  adjOmegap[48] = Omega[1];
  adjOmegap[72] = 0.0;
  adjOmegap[96] = 0.0;
  adjOmegap[120] = 0.0;
  adjOmegap[1] = Omega[2];
  adjOmegap[25] = 0.0;
  adjOmegap[49] = -Omega[0];
  adjOmegap[73] = 0.0;
  adjOmegap[97] = 0.0;
  adjOmegap[121] = 0.0;
  adjOmegap[2] = -Omega[1];
  adjOmegap[26] = Omega[0];
  adjOmegap[50] = 0.0;
  adjOmegap[74] = 0.0;
  adjOmegap[98] = 0.0;
  adjOmegap[122] = 0.0;
  adjOmegap[3] = 0.0;
  adjOmegap[27] = -Omega[5];
  adjOmegap[51] = Omega[4];
  adjOmegap[75] = 0.0;
  adjOmegap[99] = -Omega[2];
  adjOmegap[123] = Omega[1];
  adjOmegap[4] = Omega[5];
  adjOmegap[28] = 0.0;
  adjOmegap[52] = -Omega[3];
  adjOmegap[76] = Omega[2];
  adjOmegap[100] = 0.0;
  adjOmegap[124] = -Omega[0];
  adjOmegap[5] = -Omega[4];
  adjOmegap[29] = Omega[3];
  adjOmegap[53] = 0.0;
  adjOmegap[77] = -Omega[1];
  adjOmegap[101] = Omega[0];
  adjOmegap[125] = 0.0;
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      t1 = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        t1 += adjOmegap[i + 24 * i2] * adjOmegap[i2 + 24 * i1];
      }
      Td[i + 6 * i1] = t1;
    }
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      adjOmegap[(i1 + 24 * i) + 6] = Td[i1 + 6 * i];
    }
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      t1 = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        t1 += adjOmegap[(i + 24 * i2) + 6] * adjOmegap[i2 + 24 * i1];
      }
      Td[i + 6 * i1] = t1;
    }
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      adjOmegap[(i1 + 24 * i) + 12] = Td[i1 + 6 * i];
    }
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      t1 = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        t1 += adjOmegap[(i + 24 * i2) + 12] * adjOmegap[i2 + 24 * i1];
      }
      Td[i + 6 * i1] = t1;
    }
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      adjOmegap[(i1 + 24 * i) + 18] = Td[i1 + 6 * i];
    }
  }
  /*  optimized on 30.05.2022 */
  adjOmegapd[0] = 0.0;
  adjOmegapd[24] = -Omegad[2];
  adjOmegapd[48] = Omegad[1];
  adjOmegapd[72] = 0.0;
  adjOmegapd[96] = 0.0;
  adjOmegapd[120] = 0.0;
  adjOmegapd[1] = Omegad[2];
  adjOmegapd[25] = 0.0;
  adjOmegapd[49] = -Omegad[0];
  adjOmegapd[73] = 0.0;
  adjOmegapd[97] = 0.0;
  adjOmegapd[121] = 0.0;
  adjOmegapd[2] = -Omegad[1];
  adjOmegapd[26] = Omegad[0];
  adjOmegapd[50] = 0.0;
  adjOmegapd[74] = 0.0;
  adjOmegapd[98] = 0.0;
  adjOmegapd[122] = 0.0;
  adjOmegapd[3] = 0.0;
  adjOmegapd[27] = -Omegad[5];
  adjOmegapd[51] = Omegad[4];
  adjOmegapd[75] = 0.0;
  adjOmegapd[99] = -Omegad[2];
  adjOmegapd[123] = Omegad[1];
  adjOmegapd[4] = Omegad[5];
  adjOmegapd[28] = 0.0;
  adjOmegapd[52] = -Omegad[3];
  adjOmegapd[76] = Omegad[2];
  adjOmegapd[100] = 0.0;
  adjOmegapd[124] = -Omegad[0];
  adjOmegapd[5] = -Omegad[4];
  adjOmegapd[29] = Omegad[3];
  adjOmegapd[53] = 0.0;
  adjOmegapd[77] = -Omegad[1];
  adjOmegapd[101] = Omegad[0];
  adjOmegapd[125] = 0.0;
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      t1 = 0.0;
      t2 = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        i3 = i + 24 * i2;
        u = i2 + 24 * i1;
        t1 += adjOmegapd[i3] * adjOmegap[u];
        t2 += adjOmegap[i3] * adjOmegapd[u];
      }
      Td_tmp = i + 6 * i1;
      Td[Td_tmp] = t2;
      b_adjOmegapd[Td_tmp] = t1;
    }
  }
  for (i = 0; i < 6; i++) {
    r = _mm_loadu_pd(&b_adjOmegapd[6 * i]);
    r1 = _mm_loadu_pd(&Td[6 * i]);
    _mm_storeu_pd(&adjOmegapd[24 * i + 6], _mm_add_pd(r, r1));
    i1 = 6 * i + 2;
    r = _mm_loadu_pd(&b_adjOmegapd[i1]);
    r1 = _mm_loadu_pd(&Td[i1]);
    _mm_storeu_pd(&adjOmegapd[24 * i + 8], _mm_add_pd(r, r1));
    i1 = 6 * i + 4;
    r = _mm_loadu_pd(&b_adjOmegapd[i1]);
    r1 = _mm_loadu_pd(&Td[i1]);
    _mm_storeu_pd(&adjOmegapd[24 * i + 10], _mm_add_pd(r, r1));
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      t1 = 0.0;
      t2 = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        i3 = (i + 24 * i2) + 6;
        u = i2 + 24 * i1;
        t1 += adjOmegapd[i3] * adjOmegap[u];
        t2 += adjOmegap[i3] * adjOmegapd[u];
      }
      Td_tmp = i + 6 * i1;
      Td[Td_tmp] = t2;
      b_adjOmegapd[Td_tmp] = t1;
    }
  }
  for (i = 0; i < 6; i++) {
    r = _mm_loadu_pd(&b_adjOmegapd[6 * i]);
    r1 = _mm_loadu_pd(&Td[6 * i]);
    _mm_storeu_pd(&adjOmegapd[24 * i + 12], _mm_add_pd(r, r1));
    i1 = 6 * i + 2;
    r = _mm_loadu_pd(&b_adjOmegapd[i1]);
    r1 = _mm_loadu_pd(&Td[i1]);
    _mm_storeu_pd(&adjOmegapd[24 * i + 14], _mm_add_pd(r, r1));
    i1 = 6 * i + 4;
    r = _mm_loadu_pd(&b_adjOmegapd[i1]);
    r1 = _mm_loadu_pd(&Td[i1]);
    _mm_storeu_pd(&adjOmegapd[24 * i + 16], _mm_add_pd(r, r1));
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      t1 = 0.0;
      t2 = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        i3 = (i + 24 * i2) + 12;
        u = i2 + 24 * i1;
        t1 += adjOmegapd[i3] * adjOmegap[u];
        t2 += adjOmegap[i3] * adjOmegapd[u];
      }
      Td_tmp = i + 6 * i1;
      Td[Td_tmp] = t2;
      b_adjOmegapd[Td_tmp] = t1;
    }
  }
  for (i = 0; i < 6; i++) {
    r = _mm_loadu_pd(&b_adjOmegapd[6 * i]);
    r1 = _mm_loadu_pd(&Td[6 * i]);
    _mm_storeu_pd(&adjOmegapd[24 * i + 18], _mm_add_pd(r, r1));
    i1 = 6 * i + 2;
    r = _mm_loadu_pd(&b_adjOmegapd[i1]);
    r1 = _mm_loadu_pd(&Td[i1]);
    _mm_storeu_pd(&adjOmegapd[24 * i + 20], _mm_add_pd(r, r1));
    i1 = 6 * i + 4;
    r = _mm_loadu_pd(&b_adjOmegapd[i1]);
    r1 = _mm_loadu_pd(&Td[i1]);
    _mm_storeu_pd(&adjOmegapd[24 * i + 22], _mm_add_pd(r, r1));
  }
  if (theta <= 0.01) {
    for (i = 0; i < 16; i++) {
      g[i] =
          ((g[i] + (real_T)iv[i]) + Omegahatp2[i] / 2.0) + Omegahatp3[i] / 6.0;
    }
    f[0] = 0.5;
    f[1] = 0.16666666666666666;
    f[2] = 0.041666666666666664;
    f[3] = 0.0083333333333333332;
    for (i = 0; i < 6; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        T_tmp = i1 + 24 * i;
        Td_tmp = i1 + 6 * i;
        T[Td_tmp] = (((0.5 * adjOmegap[T_tmp] + (real_T)iv1[Td_tmp]) +
                      0.16666666666666666 * adjOmegap[T_tmp + 6]) +
                     0.041666666666666664 * adjOmegap[T_tmp + 12]) +
                    0.0083333333333333332 * adjOmegap[T_tmp + 18];
        Td[Td_tmp] = ((0.5 * adjOmegapd[T_tmp] +
                       0.16666666666666666 * adjOmegapd[T_tmp + 6]) +
                      0.041666666666666664 * adjOmegapd[T_tmp + 12]) +
                     0.0083333333333333332 * adjOmegapd[T_tmp + 18];
      }
    }
  } else {
    real_T costheta;
    real_T t3;
    real_T t4;
    real_T tp3;
    real_T tp5;
    real_T tp6;
    tp2 = theta * theta;
    tp3 = tp2 * theta;
    tp4 = tp3 * theta;
    tp5 = tp4 * theta;
    tp6 = tp5 * theta;
    sintheta = muDoubleScalarSin(theta);
    costheta = muDoubleScalarCos(theta);
    t1 = theta * sintheta;
    t2 = theta * costheta;
    t3 = ((8.0 - tp2) * costheta - 8.0) + 5.0 * t1;
    t4 = (-8.0 * theta + (15.0 - tp2) * sintheta) - 7.0 * t2;
    f[0] = ((4.0 - 4.0 * costheta) - t1) / (2.0 * tp2);
    f[1] = ((4.0 * theta - 5.0 * sintheta) + t2) / (2.0 * tp3);
    f[2] = ((2.0 - 2.0 * costheta) - t1) / (2.0 * tp4);
    f[3] = ((2.0 * theta - 3.0 * sintheta) + t2) / (2.0 * tp5);
    fd[0] = t3 / (2.0 * tp3);
    fd[1] = t4 / (2.0 * tp4);
    fd[2] = t3 / (2.0 * tp5);
    fd[3] = t4 / (2.0 * tp6);
    t1 = theta *
         ((5.0 * sintheta + sintheta * (tp2 - 8.0)) + 3.0 * theta * costheta);
    fdd[0] = (t1 - 3.0 * t3) / (2.0 * tp4);
    t2 = theta * (((5.0 * theta * sintheta - 8.0) + (15.0 - tp2) * costheta) -
                  7.0 * costheta);
    fdd[1] = (t2 - 4.0 * t4) / (2.0 * tp5);
    fdd[2] = (t1 - 5.0 * t3) / (2.0 * tp6);
    fdd[3] = (t2 - 6.0 * t4) / (2.0 * (tp6 * theta));
    a = (1.0 - costheta) / tp2;
    sintheta = (theta - sintheta) / tp3;
    for (i = 0; i < 16; i++) {
      g[i] = ((g[i] + (real_T)iv[i]) + a * Omegahatp2[i]) +
             sintheta * Omegahatp3[i];
    }
    a = fd[0] * thetad;
    sintheta = fd[1] * thetad;
    tp5 = fd[2] * thetad;
    tp6 = fd[3] * thetad;
    t1 = f[0];
    t2 = f[1];
    tp4 = f[2];
    tp2 = f[3];
    for (i = 0; i < 6; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        i2 = i1 + 24 * i;
        costheta = adjOmegap[i2];
        t3 = adjOmegap[i2 + 6];
        t4 = adjOmegap[i2 + 12];
        tp3 = adjOmegap[i2 + 18];
        T_tmp = i1 + 6 * i;
        T[T_tmp] =
            (((t1 * costheta + (real_T)iv1[T_tmp]) + t2 * t3) + tp4 * t4) +
            tp2 * tp3;
        Td[T_tmp] = ((((((a * costheta + t1 * adjOmegapd[i2]) + sintheta * t3) +
                        t2 * adjOmegapd[i2 + 6]) +
                       tp5 * t4) +
                      tp4 * adjOmegapd[i2 + 12]) +
                     tp6 * tp3) +
                    tp2 * adjOmegapd[i2 + 18];
      }
    }
  }
  /*  computation of dSdq_qd, dSdq_qdd, dSddq_qd, dSddqd_qd */
  st.site = &c_emlrtRSI;
  b_mtimes(T, Phi_data, Phi_size, S_data, S_size);
  st.site = &d_emlrtRSI;
  b_mtimes(Td, Phi_data, Phi_size, Sd_data, Sd_size);
  dSdq_qd_size[0] = 6;
  dSdq_qd_size[1] = Phi_size[1];
  dSdq_qdd_size[0] = 6;
  dSdq_qdd_size[1] = Phi_size[1];
  dSddq_qd_size[0] = 6;
  dSddq_qd_size[1] = Phi_size[1];
  if (loop_ub_tmp - 1 >= 0) {
    memset(&dSdq_qd_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
    memset(&dSdq_qdd_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
    memset(&dSddq_qd_data[0], 0, (uint32_T)loop_ub_tmp * sizeof(real_T));
  }
  st.site = &e_emlrtRSI;
  b_st.site = &t_emlrtRSI;
  dynamic_size_checks(&b_st, qdd_size[0], Phi_size[1], qdd_size[0]);
  mtimes(Phi_data, Phi_size, qdd_data, qdd_size[0], Zqdd);
  if (theta <= 0.01) {
    for (b_r = 0; b_r < 4; b_r++) {
      /* 1st for loop */
      /* nothing here */
      t2 = f[b_r];
      for (u = 0; u <= b_r; u++) {
        real_T d_y_data[108];
        int32_T b_y_size[2];
        int32_T y_size[2];
        int32_T b_loop_ub;
        int32_T loop_ub;
        int32_T scalarLB;
        int32_T vectorUB;
        /* 2nd for loop */
        if (u + 1 == 1) {
          eye(Td);
          loop_ub = 6;
          memcpy(&adjOmegap1_data[0], &Td[0], 36U * sizeof(real_T));
        } else {
          i = (u - 1) * 6;
          i1 = 6 * u;
          if (i + 1 > i1) {
            i = 0;
            i1 = 0;
          }
          loop_ub = i1 - i;
          for (i1 = 0; i1 < 6; i1++) {
            for (i2 = 0; i2 < loop_ub; i2++) {
              adjOmegap1_data[i2 + loop_ub * i1] =
                  adjOmegap[(i + i2) + 24 * i1];
            }
          }
          /* adjOmega^(u-1) */
        }
        if (u == b_r) {
          eye(Td);
          adjOmegap2_size[0] = 6;
          adjOmegap2_size[1] = 6;
          memcpy(&adjOmegap2_data[0], &Td[0], 36U * sizeof(real_T));
        } else {
          i = b_r - u;
          i1 = (i - 1) * 6 + 1;
          i *= 6;
          if (i1 > i) {
            i1 = 0;
            i = 0;
          } else {
            if (i1 < 1) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
            i1--;
            if (i < 1) {
              emlrtDynamicBoundsCheckR2012b(i, 1, 24, &b_emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
          }
          b_loop_ub = i - i1;
          adjOmegap2_size[0] = b_loop_ub;
          adjOmegap2_size[1] = 6;
          for (i = 0; i < 6; i++) {
            for (i2 = 0; i2 < b_loop_ub; i2++) {
              adjOmegap2_data[i2 + b_loop_ub * i] =
                  adjOmegap[(i1 + i2) + 24 * i];
            }
          }
          /* adjOmega^(r-u) */
        }
        st.site = &f_emlrtRSI;
        Td_tmp = c_mtimes(adjOmegap2_data, adjOmegap2_size, Omegad, y_data);
        st.site = &f_emlrtRSI;
        b_st.site = &f_emlrtRSI;
        dinamico_adj(&b_st, y_data, Td_tmp, Td);
        f_size[0] = loop_ub;
        f_size[1] = 6;
        loop_ub_tmp = loop_ub * 6;
        T_tmp = (loop_ub_tmp / 2) << 1;
        vectorUB_tmp = T_tmp - 2;
        for (i = 0; i <= vectorUB_tmp; i += 2) {
          r = _mm_loadu_pd(&adjOmegap1_data[i]);
          _mm_storeu_pd(&f_data[i], _mm_mul_pd(_mm_set1_pd(t2), r));
        }
        for (i = T_tmp; i < loop_ub_tmp; i++) {
          f_data[i] = t2 * adjOmegap1_data[i];
        }
        d_mtimes(f_data, f_size, Td, d_y_data, y_size);
        st.site = &f_emlrtRSI;
        e_mtimes(d_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
        if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
          emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &emlrtECI,
                                      (emlrtConstCTX)sp);
        }
        if ((dSdq_qd_size[1] != b_y_size[1]) &&
            ((dSdq_qd_size[1] != 1) && (b_y_size[1] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSdq_qd_size[1], b_y_size[1], &b_emlrtECI,
                                      (emlrtConstCTX)sp);
        }
        if ((b_y_size[0] == 6) && (dSdq_qd_size[1] == b_y_size[1])) {
          b_loop_ub = 6 * dSdq_qd_size[1];
          dSdq_qd_size[0] = 6;
          scalarLB = (b_loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r = _mm_loadu_pd(&dSdq_qd_data[i]);
            r1 = _mm_loadu_pd(&e_y_data[i]);
            _mm_storeu_pd(&dSdq_qd_data[i], _mm_sub_pd(r, r1));
          }
          for (i = scalarLB; i < b_loop_ub; i++) {
            dSdq_qd_data[i] -= e_y_data[i];
          }
        } else {
          binary_expand_op(dSdq_qd_data, dSdq_qd_size, e_y_data, b_y_size);
        }
        st.site = &g_emlrtRSI;
        Td_tmp = c_mtimes(adjOmegap2_data, adjOmegap2_size, Zqdd, y_data);
        st.site = &g_emlrtRSI;
        b_st.site = &g_emlrtRSI;
        dinamico_adj(&b_st, y_data, Td_tmp, Td);
        f_size[0] = loop_ub;
        f_size[1] = 6;
        for (i = 0; i <= vectorUB_tmp; i += 2) {
          r = _mm_loadu_pd(&adjOmegap1_data[i]);
          _mm_storeu_pd(&f_data[i], _mm_mul_pd(_mm_set1_pd(t2), r));
        }
        for (i = T_tmp; i < loop_ub_tmp; i++) {
          f_data[i] = t2 * adjOmegap1_data[i];
        }
        d_mtimes(f_data, f_size, Td, d_y_data, y_size);
        st.site = &g_emlrtRSI;
        e_mtimes(d_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
        if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
          emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &c_emlrtECI,
                                      (emlrtConstCTX)sp);
        }
        if ((dSdq_qdd_size[1] != b_y_size[1]) &&
            ((dSdq_qdd_size[1] != 1) && (b_y_size[1] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSdq_qdd_size[1], b_y_size[1],
                                      &d_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((b_y_size[0] == 6) && (dSdq_qdd_size[1] == b_y_size[1])) {
          b_loop_ub = 6 * dSdq_qdd_size[1];
          dSdq_qdd_size[0] = 6;
          scalarLB = (b_loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r = _mm_loadu_pd(&dSdq_qdd_data[i]);
            r1 = _mm_loadu_pd(&e_y_data[i]);
            _mm_storeu_pd(&dSdq_qdd_data[i], _mm_sub_pd(r, r1));
          }
          for (i = scalarLB; i < b_loop_ub; i++) {
            dSdq_qdd_data[i] -= e_y_data[i];
          }
        } else {
          binary_expand_op(dSdq_qdd_data, dSdq_qdd_size, e_y_data, b_y_size);
        }
        if (u - 1 >= 0) {
          for (i = 0; i < 6; i++) {
            for (i1 = 0; i1 < 6; i1++) {
              b_adjOmegapd[i1 + 6 * i] = adjOmegapd[i1 + 24 * i];
            }
          }
        }
        for (p = 0; p < u; p++) {
          /* 3rd for loop */
          if (p + 1 == 1) {
            eye(Td);
            adjOmegap3_size[0] = 6;
            memcpy(&adjOmegap3_data[0], &Td[0], 36U * sizeof(real_T));
          } else {
            i = (p - 1) * 6;
            i1 = 6 * p;
            if (i + 1 > i1) {
              i = 0;
              i1 = 0;
            }
            b_loop_ub = i1 - i;
            adjOmegap3_size[0] = b_loop_ub;
            for (i1 = 0; i1 < 6; i1++) {
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                adjOmegap3_data[i2 + b_loop_ub * i1] =
                    adjOmegap[(i + i2) + 24 * i1];
              }
            }
            /* adjOmega^(p-1) */
          }
          if (p + 1 == u) {
            eye(Td);
            adjOmegap4_size[0] = 6;
            adjOmegap4_size[1] = 6;
            memcpy(&adjOmegap4_data[0], &Td[0], 36U * sizeof(real_T));
          } else {
            i = u - p;
            i1 = (i - 2) * 6 + 1;
            i = 6 * (i - 1);
            if (i1 > i) {
              i1 = 0;
              i = 0;
            } else {
              if (i1 < 1) {
                emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &c_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              i1--;
              if (i < 1) {
                emlrtDynamicBoundsCheckR2012b(i, 1, 24, &d_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
            }
            b_loop_ub = i - i1;
            adjOmegap4_size[0] = b_loop_ub;
            adjOmegap4_size[1] = 6;
            for (i = 0; i < 6; i++) {
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                adjOmegap4_data[i2 + b_loop_ub * i] =
                    adjOmegap[(i1 + i2) + 24 * i];
              }
            }
            /* adjOmega^(u-p-1) */
          }
          st.site = &h_emlrtRSI;
          d_mtimes(adjOmegap4_data, adjOmegap4_size, b_adjOmegapd, d_y_data,
                   y_size);
          st.site = &h_emlrtRSI;
          b_st.site = &t_emlrtRSI;
          if (adjOmegap2_size[0] != 6) {
            emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                          "MATLAB:innerdim", 0);
          }
          f_mtimes(d_y_data, y_size, adjOmegap2_data, adjOmegap2_size, f_data,
                   f_size);
          st.site = &h_emlrtRSI;
          Td_tmp = c_mtimes(f_data, f_size, Omegad, y_data);
          st.site = &h_emlrtRSI;
          b_st.site = &h_emlrtRSI;
          dinamico_adj(&b_st, y_data, Td_tmp, Td);
          t1 = f[b_r];
          f_size[0] = adjOmegap3_size[0];
          f_size[1] = 6;
          b_loop_ub = adjOmegap3_size[0] * 6;
          scalarLB = (b_loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r = _mm_loadu_pd(&adjOmegap3_data[i]);
            _mm_storeu_pd(&f_data[i], _mm_mul_pd(_mm_set1_pd(t1), r));
          }
          for (i = scalarLB; i < b_loop_ub; i++) {
            f_data[i] = t1 * adjOmegap3_data[i];
          }
          d_mtimes(f_data, f_size, Td, d_y_data, y_size);
          st.site = &h_emlrtRSI;
          e_mtimes(d_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
          if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
            emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &e_emlrtECI,
                                        (emlrtConstCTX)sp);
          }
          if ((dSddq_qd_size[1] != b_y_size[1]) &&
              ((dSddq_qd_size[1] != 1) && (b_y_size[1] != 1))) {
            emlrtDimSizeImpxCheckR2021b(dSddq_qd_size[1], b_y_size[1],
                                        &f_emlrtECI, (emlrtConstCTX)sp);
          }
          if ((b_y_size[0] == 6) && (dSddq_qd_size[1] == b_y_size[1])) {
            b_loop_ub = 6 * dSddq_qd_size[1];
            dSddq_qd_size[0] = 6;
            scalarLB = (b_loop_ub / 2) << 1;
            vectorUB = scalarLB - 2;
            for (i = 0; i <= vectorUB; i += 2) {
              r = _mm_loadu_pd(&dSddq_qd_data[i]);
              r1 = _mm_loadu_pd(&e_y_data[i]);
              _mm_storeu_pd(&dSddq_qd_data[i], _mm_sub_pd(r, r1));
            }
            for (i = scalarLB; i < b_loop_ub; i++) {
              dSddq_qd_data[i] -= e_y_data[i];
            }
          } else {
            binary_expand_op(dSddq_qd_data, dSddq_qd_size, e_y_data, b_y_size);
          }
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b((emlrtConstCTX)sp);
          }
        }
        i = b_r - u;
        if (i - 1 >= 0) {
          for (i1 = 0; i1 < 6; i1++) {
            for (i2 = 0; i2 < 6; i2++) {
              b_adjOmegapd[i2 + 6 * i1] = adjOmegapd[i2 + 24 * i1];
            }
          }
        }
        for (p = 0; p < i; p++) {
          /* 3rd for loop */
          if (p + 1 == 1) {
            eye(Td);
            adjOmegap3_size[0] = 6;
            adjOmegap3_size[1] = 6;
            memcpy(&adjOmegap3_data[0], &Td[0], 36U * sizeof(real_T));
          } else {
            i1 = (p - 1) * 6;
            i2 = 6 * p;
            if (i1 + 1 > i2) {
              i1 = 0;
              i2 = 0;
            }
            b_loop_ub = i2 - i1;
            adjOmegap3_size[0] = b_loop_ub;
            adjOmegap3_size[1] = 6;
            for (i2 = 0; i2 < 6; i2++) {
              for (i3 = 0; i3 < b_loop_ub; i3++) {
                adjOmegap3_data[i3 + b_loop_ub * i2] =
                    adjOmegap[(i1 + i3) + 24 * i2];
              }
            }
            /* adjOmega^(p-1) */
          }
          if (p + 1 == i) {
            eye(Td);
            adjOmegap4_size[0] = 6;
            adjOmegap4_size[1] = 6;
            memcpy(&adjOmegap4_data[0], &Td[0], 36U * sizeof(real_T));
          } else {
            i1 = i - p;
            i2 = (i1 - 2) * 6 + 1;
            i1 = 6 * (i1 - 1);
            if (i2 > i1) {
              i2 = 0;
              i1 = 0;
            } else {
              if (i2 < 1) {
                emlrtDynamicBoundsCheckR2012b(i2, 1, 24, &e_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              i2--;
              if (i1 < 1) {
                emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &f_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
            }
            b_loop_ub = i1 - i2;
            adjOmegap4_size[0] = b_loop_ub;
            adjOmegap4_size[1] = 6;
            for (i1 = 0; i1 < 6; i1++) {
              for (i3 = 0; i3 < b_loop_ub; i3++) {
                adjOmegap4_data[i3 + b_loop_ub * i1] =
                    adjOmegap[(i2 + i3) + 24 * i1];
              }
            }
            /* adjOmega^(r-u-p) */
          }
          st.site = &i_emlrtRSI;
          t1 = f[b_r];
          f_size[0] = loop_ub;
          f_size[1] = 6;
          for (i1 = 0; i1 <= vectorUB_tmp; i1 += 2) {
            r = _mm_loadu_pd(&adjOmegap1_data[i1]);
            _mm_storeu_pd(&f_data[i1], _mm_mul_pd(_mm_set1_pd(t1), r));
          }
          for (i1 = T_tmp; i1 < loop_ub_tmp; i1++) {
            f_data[i1] = t1 * adjOmegap1_data[i1];
          }
          d_mtimes(f_data, f_size, b_adjOmegapd, d_y_data, y_size);
          st.site = &i_emlrtRSI;
          b_st.site = &t_emlrtRSI;
          if (adjOmegap3_size[0] != 6) {
            emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                          "MATLAB:innerdim", 0);
          }
          f_mtimes(d_y_data, y_size, adjOmegap3_data, adjOmegap3_size, f_data,
                   f_size);
          st.site = &i_emlrtRSI;
          Td_tmp = c_mtimes(adjOmegap4_data, adjOmegap4_size, Omegad, y_data);
          st.site = &i_emlrtRSI;
          b_st.site = &i_emlrtRSI;
          dinamico_adj(&b_st, y_data, Td_tmp, Td);
          d_mtimes(f_data, f_size, Td, d_y_data, y_size);
          st.site = &i_emlrtRSI;
          e_mtimes(d_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
          if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
            emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &g_emlrtECI,
                                        (emlrtConstCTX)sp);
          }
          if ((dSddq_qd_size[1] != b_y_size[1]) &&
              ((dSddq_qd_size[1] != 1) && (b_y_size[1] != 1))) {
            emlrtDimSizeImpxCheckR2021b(dSddq_qd_size[1], b_y_size[1],
                                        &h_emlrtECI, (emlrtConstCTX)sp);
          }
          if ((b_y_size[0] == 6) && (dSddq_qd_size[1] == b_y_size[1])) {
            b_loop_ub = 6 * dSddq_qd_size[1];
            dSddq_qd_size[0] = 6;
            scalarLB = (b_loop_ub / 2) << 1;
            vectorUB = scalarLB - 2;
            for (i1 = 0; i1 <= vectorUB; i1 += 2) {
              r = _mm_loadu_pd(&dSddq_qd_data[i1]);
              r1 = _mm_loadu_pd(&e_y_data[i1]);
              _mm_storeu_pd(&dSddq_qd_data[i1], _mm_sub_pd(r, r1));
            }
            for (i1 = scalarLB; i1 < b_loop_ub; i1++) {
              dSddq_qd_data[i1] -= e_y_data[i1];
            }
          } else {
            binary_expand_op(dSddq_qd_data, dSddq_qd_size, e_y_data, b_y_size);
          }
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b((emlrtConstCTX)sp);
          }
        }
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b((emlrtConstCTX)sp);
        }
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtConstCTX)sp);
      }
    }
  } else {
    a = 1.0 / theta;
    t2 = thetad / theta;
    tmp_size[1] = 6;
    r = _mm_set1_pd(t2);
    for (b_r = 0; b_r < 4; b_r++) {
      __m128d r2;
      real_T c_y_data[100];
      real_T c_Omegad[6];
      int32_T b_y_size[2];
      int32_T y_size[2];
      int32_T b_loop_ub;
      int32_T loop_ub;
      int32_T scalarLB;
      int32_T vectorUB;
      /* 1st for loop */
      i = b_r * 6;
      i1 = 6 * (b_r + 1);
      if (i + 1 > i1) {
        i2 = 0;
        i3 = 0;
      } else {
        i2 = i;
        i3 = i1;
      }
      t2 = 1.0 / theta * fd[b_r];
      st.site = &j_emlrtRSI;
      b_loop_ub = i3 - i2;
      a_size[0] = b_loop_ub;
      a_size[1] = 6;
      scalarLB = (b_loop_ub / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i3 = 0; i3 < 6; i3++) {
        for (u = 0; u <= vectorUB; u += 2) {
          r1 = _mm_loadu_pd(&adjOmegap[(i2 + u) + 24 * i3]);
          _mm_storeu_pd(&a_data[u + b_loop_ub * i3],
                        _mm_mul_pd(_mm_set1_pd(t2), r1));
        }
        for (u = scalarLB; u < b_loop_ub; u++) {
          a_data[u + b_loop_ub * i3] = t2 * adjOmegap[(i2 + u) + 24 * i3];
        }
      }
      Td_tmp = c_mtimes(a_data, a_size, Omegad, y_data);
      st.site = &j_emlrtRSI;
      a_size[0] = Td_tmp;
      a_size[1] = 6;
      scalarLB = (Td_tmp / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i2 = 0; i2 < 6; i2++) {
        for (i3 = 0; i3 <= vectorUB; i3 += 2) {
          r1 = _mm_loadu_pd(&y_data[i3]);
          _mm_storeu_pd(&a_data[i3 + Td_tmp * i2],
                        _mm_mul_pd(r1, _mm_set1_pd(Omega[i2])));
        }
        for (i3 = scalarLB; i3 < Td_tmp; i3++) {
          a_data[i3 + Td_tmp * i2] = y_data[i3] * Omega[i2];
        }
      }
      g_mtimes(a_data, a_size, b_y_data, y_size);
      st.site = &j_emlrtRSI;
      e_mtimes(b_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
      if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
        emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &i_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((dSdq_qd_size[1] != b_y_size[1]) &&
          ((dSdq_qd_size[1] != 1) && (b_y_size[1] != 1))) {
        emlrtDimSizeImpxCheckR2021b(dSdq_qd_size[1], b_y_size[1], &j_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((b_y_size[0] == 6) && (dSdq_qd_size[1] == b_y_size[1])) {
        b_loop_ub = 6 * dSdq_qd_size[1];
        dSdq_qd_size[0] = 6;
        scalarLB = (b_loop_ub / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i2 = 0; i2 <= vectorUB; i2 += 2) {
          r1 = _mm_loadu_pd(&dSdq_qd_data[i2]);
          r2 = _mm_loadu_pd(&e_y_data[i2]);
          _mm_storeu_pd(&dSdq_qd_data[i2], _mm_add_pd(r1, r2));
        }
        for (i2 = scalarLB; i2 < b_loop_ub; i2++) {
          dSdq_qd_data[i2] += e_y_data[i2];
        }
      } else {
        plus(dSdq_qd_data, dSdq_qd_size, e_y_data, b_y_size);
      }
      if (i + 1 > i1) {
        i2 = 0;
        i3 = 0;
      } else {
        i2 = i;
        i3 = i1;
      }
      st.site = &k_emlrtRSI;
      b_loop_ub = i3 - i2;
      a_size[0] = b_loop_ub;
      a_size[1] = 6;
      scalarLB = (b_loop_ub / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i3 = 0; i3 < 6; i3++) {
        for (u = 0; u <= vectorUB; u += 2) {
          r1 = _mm_loadu_pd(&adjOmegap[(i2 + u) + 24 * i3]);
          _mm_storeu_pd(&a_data[u + b_loop_ub * i3],
                        _mm_mul_pd(_mm_set1_pd(t2), r1));
        }
        for (u = scalarLB; u < b_loop_ub; u++) {
          a_data[u + b_loop_ub * i3] = t2 * adjOmegap[(i2 + u) + 24 * i3];
        }
      }
      Td_tmp = c_mtimes(a_data, a_size, Zqdd, y_data);
      st.site = &k_emlrtRSI;
      a_size[0] = Td_tmp;
      a_size[1] = 6;
      scalarLB = (Td_tmp / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i2 = 0; i2 < 6; i2++) {
        for (i3 = 0; i3 <= vectorUB; i3 += 2) {
          r1 = _mm_loadu_pd(&y_data[i3]);
          _mm_storeu_pd(&a_data[i3 + Td_tmp * i2],
                        _mm_mul_pd(r1, _mm_set1_pd(Omega[i2])));
        }
        for (i3 = scalarLB; i3 < Td_tmp; i3++) {
          a_data[i3 + Td_tmp * i2] = y_data[i3] * Omega[i2];
        }
      }
      g_mtimes(a_data, a_size, b_y_data, y_size);
      st.site = &k_emlrtRSI;
      e_mtimes(b_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
      if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
        emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &k_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((dSdq_qdd_size[1] != b_y_size[1]) &&
          ((dSdq_qdd_size[1] != 1) && (b_y_size[1] != 1))) {
        emlrtDimSizeImpxCheckR2021b(dSdq_qdd_size[1], b_y_size[1], &l_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((b_y_size[0] == 6) && (dSdq_qdd_size[1] == b_y_size[1])) {
        b_loop_ub = 6 * dSdq_qdd_size[1];
        dSdq_qdd_size[0] = 6;
        scalarLB = (b_loop_ub / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i2 = 0; i2 <= vectorUB; i2 += 2) {
          r1 = _mm_loadu_pd(&dSdq_qdd_data[i2]);
          r2 = _mm_loadu_pd(&e_y_data[i2]);
          _mm_storeu_pd(&dSdq_qdd_data[i2], _mm_add_pd(r1, r2));
        }
        for (i2 = scalarLB; i2 < b_loop_ub; i2++) {
          dSdq_qdd_data[i2] += e_y_data[i2];
        }
      } else {
        plus(dSdq_qdd_data, dSdq_qdd_size, e_y_data, b_y_size);
      }
      if (i + 1 > i1) {
        i2 = 0;
        i3 = 0;
        u = 0;
        vectorUB_tmp = 0;
      } else {
        i2 = i;
        i3 = i1;
        u = i;
        vectorUB_tmp = i1;
      }
      sintheta = fdd[b_r] * thetad;
      b_loop_ub = i3 - i2;
      y_size[0] = b_loop_ub;
      y_size[1] = 6;
      t1 = fd[b_r];
      loop_ub = vectorUB_tmp - u;
      tmp_size[0] = loop_ub;
      scalarLB = (b_loop_ub / 2) << 1;
      vectorUB = scalarLB - 2;
      Td_tmp = (loop_ub / 2) << 1;
      T_tmp = Td_tmp - 2;
      for (i3 = 0; i3 < 6; i3++) {
        for (vectorUB_tmp = 0; vectorUB_tmp <= vectorUB; vectorUB_tmp += 2) {
          r1 = _mm_loadu_pd(&adjOmegap[(i2 + vectorUB_tmp) + 24 * i3]);
          _mm_storeu_pd(&b_y_data[vectorUB_tmp + b_loop_ub * i3],
                        _mm_mul_pd(_mm_set1_pd(sintheta), r1));
        }
        for (vectorUB_tmp = scalarLB; vectorUB_tmp < b_loop_ub;
             vectorUB_tmp++) {
          b_y_data[vectorUB_tmp + b_loop_ub * i3] =
              sintheta * adjOmegap[(i2 + vectorUB_tmp) + 24 * i3];
        }
        for (vectorUB_tmp = 0; vectorUB_tmp <= T_tmp; vectorUB_tmp += 2) {
          r1 = _mm_loadu_pd(&adjOmegapd[(u + vectorUB_tmp) + 24 * i3]);
          _mm_storeu_pd(&tmp_data[vectorUB_tmp + loop_ub * i3],
                        _mm_mul_pd(_mm_set1_pd(t1), r1));
        }
        for (vectorUB_tmp = Td_tmp; vectorUB_tmp < loop_ub; vectorUB_tmp++) {
          tmp_data[vectorUB_tmp + loop_ub * i3] =
              t1 * adjOmegapd[(u + vectorUB_tmp) + 24 * i3];
        }
      }
      if ((b_loop_ub != loop_ub) && ((b_loop_ub != 1) && (loop_ub != 1))) {
        emlrtDimSizeImpxCheckR2021b(b_loop_ub, loop_ub, &m_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      st.site = &l_emlrtRSI;
      if (b_loop_ub == loop_ub) {
        a_size[0] = b_loop_ub;
        a_size[1] = 6;
        b_loop_ub *= 6;
        scalarLB = (b_loop_ub / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i2 = 0; i2 <= vectorUB; i2 += 2) {
          r1 = _mm_loadu_pd(&b_y_data[i2]);
          r2 = _mm_loadu_pd(&tmp_data[i2]);
          _mm_storeu_pd(&a_data[i2],
                        _mm_mul_pd(_mm_set1_pd(a), _mm_add_pd(r1, r2)));
        }
        for (i2 = scalarLB; i2 < b_loop_ub; i2++) {
          a_data[i2] = a * (b_y_data[i2] + tmp_data[i2]);
        }
        Td_tmp = c_mtimes(a_data, a_size, Omegad, y_data);
      } else {
        Td_tmp = binary_expand_op_9(y_data, s_emlrtRSI, a, b_y_data, y_size,
                                    tmp_data, tmp_size, Omegad);
      }
      st.site = &l_emlrtRSI;
      a_size[0] = Td_tmp;
      a_size[1] = 6;
      scalarLB = (Td_tmp / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i2 = 0; i2 < 6; i2++) {
        for (i3 = 0; i3 <= vectorUB; i3 += 2) {
          r1 = _mm_loadu_pd(&y_data[i3]);
          _mm_storeu_pd(&a_data[i3 + Td_tmp * i2],
                        _mm_mul_pd(r1, _mm_set1_pd(Omega[i2])));
        }
        for (i3 = scalarLB; i3 < Td_tmp; i3++) {
          a_data[i3 + Td_tmp * i2] = y_data[i3] * Omega[i2];
        }
      }
      g_mtimes(a_data, a_size, b_y_data, y_size);
      st.site = &l_emlrtRSI;
      e_mtimes(b_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
      if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
        emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &n_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((dSddq_qd_size[1] != b_y_size[1]) &&
          ((dSddq_qd_size[1] != 1) && (b_y_size[1] != 1))) {
        emlrtDimSizeImpxCheckR2021b(dSddq_qd_size[1], b_y_size[1], &o_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if (i + 1 > i1) {
        i = 0;
        i1 = 0;
      }
      st.site = &m_emlrtRSI;
      b_loop_ub = i1 - i;
      a_size[0] = b_loop_ub;
      a_size[1] = 6;
      scalarLB = (b_loop_ub / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i1 = 0; i1 < 6; i1++) {
        for (i2 = 0; i2 <= vectorUB; i2 += 2) {
          r1 = _mm_loadu_pd(&adjOmegap[(i + i2) + 24 * i1]);
          _mm_storeu_pd(&a_data[i2 + b_loop_ub * i1],
                        _mm_mul_pd(_mm_set1_pd(t2), r1));
        }
        for (i2 = scalarLB; i2 < b_loop_ub; i2++) {
          a_data[i2 + b_loop_ub * i1] = t2 * adjOmegap[(i + i2) + 24 * i1];
        }
      }
      Td_tmp = c_mtimes(a_data, a_size, Omegad, y_data);
      st.site = &m_emlrtRSI;
      r1 = _mm_loadu_pd(&Omega[0]);
      r2 = _mm_loadu_pd(&Omegad[0]);
      _mm_storeu_pd(&b_Omegad[0], _mm_sub_pd(r2, _mm_mul_pd(r, r1)));
      r1 = _mm_loadu_pd(&Omega[2]);
      r2 = _mm_loadu_pd(&Omegad[2]);
      _mm_storeu_pd(&b_Omegad[2], _mm_sub_pd(r2, _mm_mul_pd(r, r1)));
      r1 = _mm_loadu_pd(&Omega[4]);
      r2 = _mm_loadu_pd(&Omegad[4]);
      _mm_storeu_pd(&b_Omegad[4], _mm_sub_pd(r2, _mm_mul_pd(r, r1)));
      for (i = 0; i < 6; i++) {
        t1 = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          t1 += b_Omegad[i1] * (real_T)b[i1 + 6 * i];
        }
        c_Omegad[i] = t1;
      }
      h_mtimes(c_Omegad, Phi_data, Phi_size, c_y_data, y_size);
      b_tmp_size[0] = Td_tmp;
      b_loop_ub = y_size[1];
      b_tmp_size[1] = y_size[1];
      for (i = 0; i < b_loop_ub; i++) {
        scalarLB = (Td_tmp / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i1 = 0; i1 <= vectorUB; i1 += 2) {
          r1 = _mm_loadu_pd(&y_data[i1]);
          _mm_storeu_pd(&b_tmp_data[i1 + Td_tmp * i],
                        _mm_mul_pd(r1, _mm_set1_pd(c_y_data[i])));
        }
        for (i1 = scalarLB; i1 < Td_tmp; i1++) {
          b_tmp_data[i1 + Td_tmp * i] = y_data[i1] * c_y_data[i];
        }
      }
      if ((Td_tmp != 6) && (Td_tmp != 1)) {
        emlrtDimSizeImpxCheckR2021b(6, Td_tmp, &n_emlrtECI, (emlrtConstCTX)sp);
      }
      if ((b_y_size[0] == 6) && (dSddq_qd_size[1] == b_y_size[1])) {
        b_loop_ub = 6 * dSddq_qd_size[1];
        dSddq_qd_size[0] = 6;
        scalarLB = (b_loop_ub / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i = 0; i <= vectorUB; i += 2) {
          r1 = _mm_loadu_pd(&dSddq_qd_data[i]);
          r2 = _mm_loadu_pd(&e_y_data[i]);
          _mm_storeu_pd(&dSddq_qd_data[i], _mm_add_pd(r1, r2));
        }
        for (i = scalarLB; i < b_loop_ub; i++) {
          dSddq_qd_data[i] += e_y_data[i];
        }
      } else {
        plus(dSddq_qd_data, dSddq_qd_size, e_y_data, b_y_size);
      }
      if ((dSddq_qd_size[1] != y_size[1]) &&
          ((dSddq_qd_size[1] != 1) && (y_size[1] != 1))) {
        emlrtDimSizeImpxCheckR2021b(dSddq_qd_size[1], y_size[1], &o_emlrtECI,
                                    (emlrtConstCTX)sp);
      }
      if ((Td_tmp == 6) && (dSddq_qd_size[1] == y_size[1])) {
        b_loop_ub = 6 * dSddq_qd_size[1];
        dSddq_qd_size[0] = 6;
        scalarLB = (b_loop_ub / 2) << 1;
        vectorUB = scalarLB - 2;
        for (i = 0; i <= vectorUB; i += 2) {
          r1 = _mm_loadu_pd(&dSddq_qd_data[i]);
          r2 = _mm_loadu_pd(&b_tmp_data[i]);
          _mm_storeu_pd(&dSddq_qd_data[i], _mm_add_pd(r1, r2));
        }
        for (i = scalarLB; i < b_loop_ub; i++) {
          dSddq_qd_data[i] += b_tmp_data[i];
        }
      } else {
        plus(dSddq_qd_data, dSddq_qd_size, b_tmp_data, b_tmp_size);
      }
      t2 = f[b_r];
      sintheta = fd[b_r] * thetad;
      for (u = 0; u <= b_r; u++) {
        real_T d_y_data[108];
        /* 2nd for loop */
        if (u + 1 == 1) {
          eye(Td);
          loop_ub = 6;
          memcpy(&adjOmegap1_data[0], &Td[0], 36U * sizeof(real_T));
        } else {
          i = (u - 1) * 6;
          i1 = 6 * u;
          if (i + 1 > i1) {
            i = 0;
            i1 = 0;
          }
          loop_ub = i1 - i;
          for (i1 = 0; i1 < 6; i1++) {
            for (i2 = 0; i2 < loop_ub; i2++) {
              adjOmegap1_data[i2 + loop_ub * i1] =
                  adjOmegap[(i + i2) + 24 * i1];
            }
          }
          /* adjOmega^(u-1) */
        }
        if (u == b_r) {
          eye(Td);
          adjOmegap2_size[0] = 6;
          adjOmegap2_size[1] = 6;
          memcpy(&adjOmegap2_data[0], &Td[0], 36U * sizeof(real_T));
        } else {
          i = b_r - u;
          i1 = (i - 1) * 6 + 1;
          i *= 6;
          if (i1 > i) {
            i1 = 0;
            i = 0;
          } else {
            if (i1 < 1) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &g_emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
            i1--;
            if (i < 1) {
              emlrtDynamicBoundsCheckR2012b(i, 1, 24, &h_emlrtBCI,
                                            (emlrtConstCTX)sp);
            }
          }
          b_loop_ub = i - i1;
          adjOmegap2_size[0] = b_loop_ub;
          adjOmegap2_size[1] = 6;
          for (i = 0; i < 6; i++) {
            for (i2 = 0; i2 < b_loop_ub; i2++) {
              adjOmegap2_data[i2 + b_loop_ub * i] =
                  adjOmegap[(i1 + i2) + 24 * i];
            }
          }
          /* adjOmega^(r-u) */
        }
        st.site = &n_emlrtRSI;
        Td_tmp = c_mtimes(adjOmegap2_data, adjOmegap2_size, Omegad, y_data);
        st.site = &n_emlrtRSI;
        b_st.site = &n_emlrtRSI;
        dinamico_adj(&b_st, y_data, Td_tmp, Td);
        f_size[0] = loop_ub;
        f_size[1] = 6;
        loop_ub_tmp = loop_ub * 6;
        T_tmp = (loop_ub_tmp / 2) << 1;
        vectorUB_tmp = T_tmp - 2;
        for (i = 0; i <= vectorUB_tmp; i += 2) {
          r1 = _mm_loadu_pd(&adjOmegap1_data[i]);
          _mm_storeu_pd(&f_data[i], _mm_mul_pd(_mm_set1_pd(t2), r1));
        }
        for (i = T_tmp; i < loop_ub_tmp; i++) {
          f_data[i] = t2 * adjOmegap1_data[i];
        }
        d_mtimes(f_data, f_size, Td, d_y_data, y_size);
        st.site = &n_emlrtRSI;
        e_mtimes(d_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
        if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
          emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &p_emlrtECI,
                                      (emlrtConstCTX)sp);
        }
        if ((dSdq_qd_size[1] != b_y_size[1]) &&
            ((dSdq_qd_size[1] != 1) && (b_y_size[1] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSdq_qd_size[1], b_y_size[1], &q_emlrtECI,
                                      (emlrtConstCTX)sp);
        }
        if ((b_y_size[0] == 6) && (dSdq_qd_size[1] == b_y_size[1])) {
          b_loop_ub = 6 * dSdq_qd_size[1];
          dSdq_qd_size[0] = 6;
          scalarLB = (b_loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r1 = _mm_loadu_pd(&dSdq_qd_data[i]);
            r2 = _mm_loadu_pd(&e_y_data[i]);
            _mm_storeu_pd(&dSdq_qd_data[i], _mm_sub_pd(r1, r2));
          }
          for (i = scalarLB; i < b_loop_ub; i++) {
            dSdq_qd_data[i] -= e_y_data[i];
          }
        } else {
          binary_expand_op(dSdq_qd_data, dSdq_qd_size, e_y_data, b_y_size);
        }
        st.site = &o_emlrtRSI;
        Td_tmp = c_mtimes(adjOmegap2_data, adjOmegap2_size, Zqdd, y_data);
        st.site = &o_emlrtRSI;
        b_st.site = &o_emlrtRSI;
        dinamico_adj(&b_st, y_data, Td_tmp, Td);
        f_size[0] = loop_ub;
        f_size[1] = 6;
        for (i = 0; i <= vectorUB_tmp; i += 2) {
          r1 = _mm_loadu_pd(&adjOmegap1_data[i]);
          _mm_storeu_pd(&f_data[i], _mm_mul_pd(_mm_set1_pd(t2), r1));
        }
        for (i = T_tmp; i < loop_ub_tmp; i++) {
          f_data[i] = t2 * adjOmegap1_data[i];
        }
        d_mtimes(f_data, f_size, Td, d_y_data, y_size);
        st.site = &o_emlrtRSI;
        e_mtimes(d_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
        if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
          emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &r_emlrtECI,
                                      (emlrtConstCTX)sp);
        }
        if ((dSdq_qdd_size[1] != b_y_size[1]) &&
            ((dSdq_qdd_size[1] != 1) && (b_y_size[1] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSdq_qdd_size[1], b_y_size[1],
                                      &s_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((b_y_size[0] == 6) && (dSdq_qdd_size[1] == b_y_size[1])) {
          b_loop_ub = 6 * dSdq_qdd_size[1];
          dSdq_qdd_size[0] = 6;
          scalarLB = (b_loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r1 = _mm_loadu_pd(&dSdq_qdd_data[i]);
            r2 = _mm_loadu_pd(&e_y_data[i]);
            _mm_storeu_pd(&dSdq_qdd_data[i], _mm_sub_pd(r1, r2));
          }
          for (i = scalarLB; i < b_loop_ub; i++) {
            dSdq_qdd_data[i] -= e_y_data[i];
          }
        } else {
          binary_expand_op(dSdq_qdd_data, dSdq_qdd_size, e_y_data, b_y_size);
        }
        st.site = &p_emlrtRSI;
        Td_tmp = c_mtimes(adjOmegap2_data, adjOmegap2_size, Omegad, y_data);
        st.site = &p_emlrtRSI;
        b_st.site = &p_emlrtRSI;
        dinamico_adj(&b_st, y_data, Td_tmp, Td);
        f_size[0] = loop_ub;
        f_size[1] = 6;
        for (i = 0; i <= vectorUB_tmp; i += 2) {
          r1 = _mm_loadu_pd(&adjOmegap1_data[i]);
          _mm_storeu_pd(&f_data[i], _mm_mul_pd(_mm_set1_pd(sintheta), r1));
        }
        for (i = T_tmp; i < loop_ub_tmp; i++) {
          f_data[i] = sintheta * adjOmegap1_data[i];
        }
        d_mtimes(f_data, f_size, Td, d_y_data, y_size);
        st.site = &p_emlrtRSI;
        e_mtimes(d_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
        if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
          emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &t_emlrtECI,
                                      (emlrtConstCTX)sp);
        }
        if ((dSddq_qd_size[1] != b_y_size[1]) &&
            ((dSddq_qd_size[1] != 1) && (b_y_size[1] != 1))) {
          emlrtDimSizeImpxCheckR2021b(dSddq_qd_size[1], b_y_size[1],
                                      &u_emlrtECI, (emlrtConstCTX)sp);
        }
        if ((b_y_size[0] == 6) && (dSddq_qd_size[1] == b_y_size[1])) {
          b_loop_ub = 6 * dSddq_qd_size[1];
          dSddq_qd_size[0] = 6;
          scalarLB = (b_loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r1 = _mm_loadu_pd(&dSddq_qd_data[i]);
            r2 = _mm_loadu_pd(&e_y_data[i]);
            _mm_storeu_pd(&dSddq_qd_data[i], _mm_sub_pd(r1, r2));
          }
          for (i = scalarLB; i < b_loop_ub; i++) {
            dSddq_qd_data[i] -= e_y_data[i];
          }
        } else {
          binary_expand_op(dSddq_qd_data, dSddq_qd_size, e_y_data, b_y_size);
        }
        if (u - 1 >= 0) {
          for (i = 0; i < 6; i++) {
            for (i1 = 0; i1 < 6; i1++) {
              b_adjOmegapd[i1 + 6 * i] = adjOmegapd[i1 + 24 * i];
            }
          }
        }
        for (p = 0; p < u; p++) {
          /* 3rd for loop */
          if (p + 1 == 1) {
            eye(Td);
            adjOmegap3_size[0] = 6;
            memcpy(&adjOmegap3_data[0], &Td[0], 36U * sizeof(real_T));
          } else {
            i = (p - 1) * 6;
            i1 = 6 * p;
            if (i + 1 > i1) {
              i = 0;
              i1 = 0;
            }
            b_loop_ub = i1 - i;
            adjOmegap3_size[0] = b_loop_ub;
            for (i1 = 0; i1 < 6; i1++) {
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                adjOmegap3_data[i2 + b_loop_ub * i1] =
                    adjOmegap[(i + i2) + 24 * i1];
              }
            }
            /* adjOmega^(p-1) */
          }
          if (p + 1 == u) {
            eye(Td);
            adjOmegap4_size[0] = 6;
            adjOmegap4_size[1] = 6;
            memcpy(&adjOmegap4_data[0], &Td[0], 36U * sizeof(real_T));
          } else {
            i = u - p;
            i1 = (i - 2) * 6 + 1;
            i = 6 * (i - 1);
            if (i1 > i) {
              i1 = 0;
              i = 0;
            } else {
              if (i1 < 1) {
                emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &i_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              i1--;
              if (i < 1) {
                emlrtDynamicBoundsCheckR2012b(i, 1, 24, &j_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
            }
            b_loop_ub = i - i1;
            adjOmegap4_size[0] = b_loop_ub;
            adjOmegap4_size[1] = 6;
            for (i = 0; i < 6; i++) {
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                adjOmegap4_data[i2 + b_loop_ub * i] =
                    adjOmegap[(i1 + i2) + 24 * i];
              }
            }
            /* adjOmega^(u-p-1) */
          }
          st.site = &q_emlrtRSI;
          d_mtimes(adjOmegap4_data, adjOmegap4_size, b_adjOmegapd, d_y_data,
                   y_size);
          st.site = &q_emlrtRSI;
          b_st.site = &t_emlrtRSI;
          if (adjOmegap2_size[0] != 6) {
            emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                          "MATLAB:innerdim", 0);
          }
          f_mtimes(d_y_data, y_size, adjOmegap2_data, adjOmegap2_size, f_data,
                   f_size);
          st.site = &q_emlrtRSI;
          Td_tmp = c_mtimes(f_data, f_size, Omegad, y_data);
          st.site = &q_emlrtRSI;
          b_st.site = &q_emlrtRSI;
          dinamico_adj(&b_st, y_data, Td_tmp, Td);
          t1 = f[b_r];
          f_size[0] = adjOmegap3_size[0];
          f_size[1] = 6;
          b_loop_ub = adjOmegap3_size[0] * 6;
          scalarLB = (b_loop_ub / 2) << 1;
          vectorUB = scalarLB - 2;
          for (i = 0; i <= vectorUB; i += 2) {
            r1 = _mm_loadu_pd(&adjOmegap3_data[i]);
            _mm_storeu_pd(&f_data[i], _mm_mul_pd(_mm_set1_pd(t1), r1));
          }
          for (i = scalarLB; i < b_loop_ub; i++) {
            f_data[i] = t1 * adjOmegap3_data[i];
          }
          d_mtimes(f_data, f_size, Td, d_y_data, y_size);
          st.site = &q_emlrtRSI;
          e_mtimes(d_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
          if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
            emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &v_emlrtECI,
                                        (emlrtConstCTX)sp);
          }
          if ((dSddq_qd_size[1] != b_y_size[1]) &&
              ((dSddq_qd_size[1] != 1) && (b_y_size[1] != 1))) {
            emlrtDimSizeImpxCheckR2021b(dSddq_qd_size[1], b_y_size[1],
                                        &w_emlrtECI, (emlrtConstCTX)sp);
          }
          if ((b_y_size[0] == 6) && (dSddq_qd_size[1] == b_y_size[1])) {
            b_loop_ub = 6 * dSddq_qd_size[1];
            dSddq_qd_size[0] = 6;
            scalarLB = (b_loop_ub / 2) << 1;
            vectorUB = scalarLB - 2;
            for (i = 0; i <= vectorUB; i += 2) {
              r1 = _mm_loadu_pd(&dSddq_qd_data[i]);
              r2 = _mm_loadu_pd(&e_y_data[i]);
              _mm_storeu_pd(&dSddq_qd_data[i], _mm_sub_pd(r1, r2));
            }
            for (i = scalarLB; i < b_loop_ub; i++) {
              dSddq_qd_data[i] -= e_y_data[i];
            }
          } else {
            binary_expand_op(dSddq_qd_data, dSddq_qd_size, e_y_data, b_y_size);
          }
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b((emlrtConstCTX)sp);
          }
        }
        i = b_r - u;
        if (i - 1 >= 0) {
          for (i1 = 0; i1 < 6; i1++) {
            for (i2 = 0; i2 < 6; i2++) {
              b_adjOmegapd[i2 + 6 * i1] = adjOmegapd[i2 + 24 * i1];
            }
          }
        }
        for (p = 0; p < i; p++) {
          /* 3rd for loop */
          if (p + 1 == 1) {
            eye(Td);
            adjOmegap3_size[0] = 6;
            adjOmegap3_size[1] = 6;
            memcpy(&adjOmegap3_data[0], &Td[0], 36U * sizeof(real_T));
          } else {
            i1 = (p - 1) * 6;
            i2 = 6 * p;
            if (i1 + 1 > i2) {
              i1 = 0;
              i2 = 0;
            }
            b_loop_ub = i2 - i1;
            adjOmegap3_size[0] = b_loop_ub;
            adjOmegap3_size[1] = 6;
            for (i2 = 0; i2 < 6; i2++) {
              for (i3 = 0; i3 < b_loop_ub; i3++) {
                adjOmegap3_data[i3 + b_loop_ub * i2] =
                    adjOmegap[(i1 + i3) + 24 * i2];
              }
            }
            /* adjOmega^(p-1) */
          }
          if (p + 1 == i) {
            eye(Td);
            adjOmegap4_size[0] = 6;
            adjOmegap4_size[1] = 6;
            memcpy(&adjOmegap4_data[0], &Td[0], 36U * sizeof(real_T));
          } else {
            i1 = i - p;
            i2 = (i1 - 2) * 6 + 1;
            i1 = 6 * (i1 - 1);
            if (i2 > i1) {
              i2 = 0;
              i1 = 0;
            } else {
              if (i2 < 1) {
                emlrtDynamicBoundsCheckR2012b(i2, 1, 24, &k_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
              i2--;
              if (i1 < 1) {
                emlrtDynamicBoundsCheckR2012b(i1, 1, 24, &l_emlrtBCI,
                                              (emlrtConstCTX)sp);
              }
            }
            b_loop_ub = i1 - i2;
            adjOmegap4_size[0] = b_loop_ub;
            adjOmegap4_size[1] = 6;
            for (i1 = 0; i1 < 6; i1++) {
              for (i3 = 0; i3 < b_loop_ub; i3++) {
                adjOmegap4_data[i3 + b_loop_ub * i1] =
                    adjOmegap[(i2 + i3) + 24 * i1];
              }
            }
            /* adjOmega^(r-u-p) */
          }
          st.site = &r_emlrtRSI;
          t1 = f[b_r];
          f_size[0] = loop_ub;
          f_size[1] = 6;
          for (i1 = 0; i1 <= vectorUB_tmp; i1 += 2) {
            r1 = _mm_loadu_pd(&adjOmegap1_data[i1]);
            _mm_storeu_pd(&f_data[i1], _mm_mul_pd(_mm_set1_pd(t1), r1));
          }
          for (i1 = T_tmp; i1 < loop_ub_tmp; i1++) {
            f_data[i1] = t1 * adjOmegap1_data[i1];
          }
          d_mtimes(f_data, f_size, b_adjOmegapd, d_y_data, y_size);
          st.site = &r_emlrtRSI;
          b_st.site = &t_emlrtRSI;
          if (adjOmegap3_size[0] != 6) {
            emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI, "MATLAB:innerdim",
                                          "MATLAB:innerdim", 0);
          }
          f_mtimes(d_y_data, y_size, adjOmegap3_data, adjOmegap3_size, f_data,
                   f_size);
          st.site = &r_emlrtRSI;
          Td_tmp = c_mtimes(adjOmegap4_data, adjOmegap4_size, Omegad, y_data);
          st.site = &r_emlrtRSI;
          b_st.site = &r_emlrtRSI;
          dinamico_adj(&b_st, y_data, Td_tmp, Td);
          d_mtimes(f_data, f_size, Td, d_y_data, y_size);
          st.site = &r_emlrtRSI;
          e_mtimes(d_y_data, y_size, Phi_data, Phi_size, e_y_data, b_y_size);
          if ((b_y_size[0] != 6) && (b_y_size[0] != 1)) {
            emlrtDimSizeImpxCheckR2021b(6, b_y_size[0], &x_emlrtECI,
                                        (emlrtConstCTX)sp);
          }
          if ((dSddq_qd_size[1] != b_y_size[1]) &&
              ((dSddq_qd_size[1] != 1) && (b_y_size[1] != 1))) {
            emlrtDimSizeImpxCheckR2021b(dSddq_qd_size[1], b_y_size[1],
                                        &y_emlrtECI, (emlrtConstCTX)sp);
          }
          if ((b_y_size[0] == 6) && (dSddq_qd_size[1] == b_y_size[1])) {
            b_loop_ub = 6 * dSddq_qd_size[1];
            dSddq_qd_size[0] = 6;
            scalarLB = (b_loop_ub / 2) << 1;
            vectorUB = scalarLB - 2;
            for (i1 = 0; i1 <= vectorUB; i1 += 2) {
              r1 = _mm_loadu_pd(&dSddq_qd_data[i1]);
              r2 = _mm_loadu_pd(&e_y_data[i1]);
              _mm_storeu_pd(&dSddq_qd_data[i1], _mm_sub_pd(r1, r2));
            }
            for (i1 = scalarLB; i1 < b_loop_ub; i1++) {
              dSddq_qd_data[i1] -= e_y_data[i1];
            }
          } else {
            binary_expand_op(dSddq_qd_data, dSddq_qd_size, e_y_data, b_y_size);
          }
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b((emlrtConstCTX)sp);
          }
        }
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b((emlrtConstCTX)sp);
        }
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtConstCTX)sp);
      }
    }
  }
}

/* End of code generation (RigidJointDifferentialKinematics.c) */
