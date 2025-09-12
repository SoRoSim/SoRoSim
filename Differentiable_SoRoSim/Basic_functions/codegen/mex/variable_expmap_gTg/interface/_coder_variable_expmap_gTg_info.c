/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_variable_expmap_gTg_info.c
 *
 * Code generation for function 'variable_expmap_gTg'
 *
 */

/* Include files */
#include "_coder_variable_expmap_gTg_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789ced54cb4ec240149d1a342e5059b966e19a90483071a73c44130da1244ac4e0b49dd2"
      "a19d99da4e49fb15ea4fb8f37f4cfc161381d2429b346020351a6673"
      "7be6dc9933f7dee600e1f25a0000ec037fbd64fdb837c5b969dc02d115e785589e104d07"
      "db20133917f0cfd32833ca91cb7d402141e14985114c21e56dcf44c0",
      "423633864899302a36501b1324ce839b3122f5392a04636afc5dd190ac8b0e019666cf5e"
      "68cc83b01f1f09f56696ec879bd08f5c8cbfaf3d544fbb2d642368c9"
      "5a57642acfb798c43896ed7c9b314362ee68b7c5444cf21798371c2980dd2a56556421ca"
      "31940cd40bb6cfa18de59eea50996346edee105a7e02724d02cd5ebf",
      "dd2f90a04e73c53ab30bea0c7865344b8265d6d3e064d881fee38afa3b89fa3ea3306754"
      "fafae6fa94a817e55398eb7c4b0b64aabba8af074bd6198fb3fcdd49"
      "7c7bfd14d2d42b1e759a69ea05ebb7f4dc908fdeb7ec7f7a98a0978bf18db257d6b86913"
      "85286aa75cbcf06a36acccded15ca0b3e81d2001a775ffc6c797abf3",
      "c73e0e95c1186f7c7c7d3e3e6ae9bff7f1f7afbb8d8f83f5fbb87e5c726f4b35af4a65dd"
      "94cef441e3ea44afff7d1fff06dd7c13e8",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 3240U, &nameCaptureInfo);
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
  xInputs = emlrtCreateLogicalMatrix(1, 1);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("variable_expmap_gTg"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\variable_expmap_gTg.m"));
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

/* End of code generation (_coder_variable_expmap_gTg_info.c) */
