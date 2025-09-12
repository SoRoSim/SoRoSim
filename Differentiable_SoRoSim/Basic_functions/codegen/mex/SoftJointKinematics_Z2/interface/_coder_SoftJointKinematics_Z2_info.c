/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_SoftJointKinematics_Z2_info.c
 *
 * Code generation for function 'SoftJointKinematics_Z2'
 *
 */

/* Include files */
#include "_coder_SoftJointKinematics_Z2_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789ced54cb4ac340149d48151755bb72dd856b295a2ab8d33eac15a53405ad56da493221"
      "d36466623295143f42fc0357eefc1fc14d7f44b0af340f0cad54224a"
      "677373e6ccbd67ee9d7080707a2e000036c1783d27c77163825393b802822bcc0ba17342"
      "f038580589409ecb3f4ea2cc28470e1f030a099a662a8c600a29aff7",
      "4c042c6433e31e292346c506aa6382443fb8182252f2515330a486df790dc9bad825c0d2"
      "6cef86861f4ce7d18fe83731e73c1e22e6910af137c5dbc261b3866c"
      "042d596b8a4ce5e91a9318c7b29dae336648cc19ecd6988849fa04f372577261b3805515"
      "5988720c2503b5dced636863b9a576a9cc31a3f6a8668561cacf3045",
      "040e2bb7aef77649a05f73c17e9333fa757965f0a604cbaca541eed76f2fa8bf16a93f66"
      "14d61dccc8d37b5b50ef2e522fc8c7f0befe914e5e75f65cb7e6ec33"
      "1cbdf3eba3f8f2f42ec4a997d96954e3d473d76fe93911f5e6fd4fb723f45221be9cebe5"
      "346eda44218adac8654e7a451be6bd7b5467e8ccba0788c071d5ef47",
      "e42ffdfceb7ebfede750e9f8f5db0bea2ffd7c34d27fefe7af1f574b3f073fefe7fa7ed6"
      "b9cc167b052aeba674a477ca9503bdf4f7fdfc136e1115d8",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 3256U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9] = {"Version",
                                    "ResolvedFunctions",
                                    "Checksum",
                                    "EntryPoints",
                                    "CoverageInfo",
                                    "IsPolymorphic",
                                    "PropertyList",
                                    "UUID",
                                    "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8] = {
      "Name",     "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "FullPath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 4);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("SoftJointKinematics_Z2"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(8.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\SoftJointKinematics_Z2.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739686.59887731483));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.1.0.2653294 (R2024a) Update 5"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("lnram8IhKh5njPPdEMcYbE"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_SoftJointKinematics_Z2_info.c) */
