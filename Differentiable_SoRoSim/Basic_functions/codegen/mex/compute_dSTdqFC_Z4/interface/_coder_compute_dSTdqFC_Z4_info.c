/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_compute_dSTdqFC_Z4_info.c
 *
 * Code generation for function 'compute_dSTdqFC_Z4'
 *
 */

/* Include files */
#include "_coder_compute_dSTdqFC_Z4_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[4] = {
      "789cc553cb4ac340149d4a2d6eaa75e3ba0bd72245105cda8715b194269bd6484c26133a"
      "9a995b93498dfe84f8137e8f4bc19f316d32e9438614aaf56e6ece1c"
      "26e7dc731954b8bc2e20847651521fa5a497535c49fb165aac65bea0e8b2b65171e19ee4"
      "5fd38e810b128904708b91eca6038c728b0bfd7944904f02f0c6c499",
      "322ef5884e19d1e6416782586b8ecac0849a7cd787043f682143fe309839f4e64196c7a7"
      "62dee28a793c29f2a82cf137cddbc699d12301b17c3c34347045b507"
      "36088a83aa0ee0d910c5a73dd028ab5e50d10e6d098d06755de2132ea8657bc494c7e756"
      "40b1e9861c0b0a3c3030b0512888e968baf3d8aa9b83932396cd19ad",
      "39e77ece9c9277e25d328ac1c46039f7b6e5a7e7d2c7dd9a3e4a4a1f09e3401867f47bfb"
      "1d2bf516f90decf747b4f17a7373dd5b714ed5bb2ea39d697f7ffb9a"
      "529bd23b3eec7737a927ebbff4d67d9f070abdca123fd03bd14bdfe776d8ea87f5d35a70"
      "556b5f36673eba393a793e9002fff5ffbf0173a49f42",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 1768U, &nameCaptureInfo);
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
  xInputs = emlrtCreateLogicalMatrix(1, 10);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("compute_dSTdqFC_Z4"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(10.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\compute_dSTdqFC_Z4.m"));
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
                emlrtMxCreateString("7n7pDLHPmLLVNxUmQsclXG"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_compute_dSTdqFC_Z4_info.c) */
