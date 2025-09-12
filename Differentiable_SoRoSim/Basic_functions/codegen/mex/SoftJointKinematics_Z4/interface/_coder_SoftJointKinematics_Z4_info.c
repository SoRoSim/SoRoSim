/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_SoftJointKinematics_Z4_info.c
 *
 * Code generation for function 'SoftJointKinematics_Z4'
 *
 */

/* Include files */
#include "_coder_SoftJointKinematics_Z4_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789ced54cb4ec240149d1a342e5059b966e1da90483071a73cc5680890288a81693b4d87"
      "7666b09d1a1a3fc2f807aedcf93f266ef8111329b4f4111b30188c86"
      "d9dc9e3973ef997ba73940383d170000db60b29e9393b8e5e2941bd74078457921724e08"
      "1f07eb2011caf3f847374a8c7234e013402141d34c99114c21e54dbb",
      "8f80814ca6df2379cc2858474d4c5023082e1c444a016a0a1ccaf9ceab48d21a1601866a"
      "fa37d483603a8f614cbf8939e7f110338f5484bf29de168eda756422"
      "68486abbc1149eae3391712c99e92663bac806a3dd3a6b60922e635eb1440fb60b585190"
      "8128c750d451c7db3e8126963a8a45258e1935c735ab0c537e862922",
      "d0a9dcb9ceee9350bffd05fb4dcee8d7e3e5d19b122cb10e947b41fdee82fa1bb1fa1346"
      "66d66846bededb827a77b17a617e09ef1b1ca9fbaab3e7ba33679fd1"
      "e89fdf1cc797a77761997aaf1f57b565ea79ebb7f40631f5e6fd4f7763f452115e3bc80e"
      "2eb345bb4025ad2f1e6bbd4af5502bf9f7a8cdd099750f108397557f",
      "1893bff2f3affbfdb69fab9007f5bb0beaaffc7c3cd27fefe799bdd6cacfc1cffb792567"
      "e754de37894c64a595cb94eda209f37fdfcf3f019dcc15dc",
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
  xInputs = emlrtCreateLogicalMatrix(1, 6);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("SoftJointKinematics_Z4"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(6.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(8.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\SoftJointKinematics_Z4.m"));
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

/* End of code generation (_coder_SoftJointKinematics_Z4_info.c) */
