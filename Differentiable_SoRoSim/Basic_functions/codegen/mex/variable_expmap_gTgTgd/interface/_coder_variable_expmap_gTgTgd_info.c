/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_variable_expmap_gTgTgd_info.c
 *
 * Code generation for function 'variable_expmap_gTgTgd'
 *
 */

/* Include files */
#include "_coder_variable_expmap_gTgTgd_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789ced544d4fc23018ee0c1a0fa89c3c73f06c48249878533e44130d01122562a0db3a57"
      "b6ae732b64c41f61fc079ebcf97f4cbcf0474c04b6c1b6d80c8399d1"
      "d01dde3d7ddef6e9fbb679807076210000b6813b9ed36edcf270c68b6b203ca2bc10c913"
      "c2e9601da442eb7cfed18b12351872980b0c48d06ca54c0936a0c19a",
      "4313010bd9541f2079ca2858474d4c5023082e27885402d40c4ca8c97f514592d6e81360"
      "a9f6fc847a10ccfa31e2d49b5ab01f0f9c7e6422fc4df9b674d4ae23"
      "1b414b52db0daab06c9d8a9461c9ce3629d545ea8c67ebb48149f614b36a5ff461bb8415"
      "0559c860188a3aeaf8d327d0c65247e91b12c3d4b0db0368b909c831",
      "09343b77cdf127ef9350bde692f5a663eaf579797ca7044bb4a34216d4ef2ea9bfc1d577"
      "1999f6c72d98ebbd2da977cfd50bf309dc6fb0a5deadc6f77567c13a"
      "a3719ebf398d2f4fef42927ab9bd562d493d7ffc969ec3d96fd177bacbd1cb44f86a6158"
      "5099691399c84aab903b1d966d589c9fa316a313770ec0c149ed3fe2",
      "ac5ff9f9d7f57edbcfa1dc0bea7797d45ff9f9b4a5ffdecf5f3fae577e0e7edecfb583bc"
      "73952f0f4b86a499e2b1d6ab9e1f6a95bfefe79f2c831652",
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
  xInputs = emlrtCreateLogicalMatrix(1, 2);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("variable_expmap_gTgTgd"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(3.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\variable_expmap_gTgTgd.m"));
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

/* End of code generation (_coder_variable_expmap_gTgTgd_info.c) */
