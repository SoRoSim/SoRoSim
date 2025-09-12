/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_SoftJointDifferentialKinematics_Z2_info.c
 *
 * Code generation for function 'SoftJointDifferentialKinematics_Z2'
 *
 */

/* Include files */
#include "_coder_SoftJointDifferentialKinematics_Z2_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789ced54cb4ac340149d48151755bb72dd856b295a2ab8d33ead28a52d68b55227c9844c"
      "9399a9c954da2f7029fe843bc1cf11fc017f4230699ae601a1d59688"
      "d2d9dc9c3977e6ccbd371c209c9c0900804de0acd7a41337c638358e2b20b8c2bc10ca13"
      "82e960152402e75cfe711c2546391a70075048d0e4a4cc08a690f2e6",
      "b08780814ca6df2379c42858474d4c50c30fce6d444a3e6a026ccafeceab48d21a7d020c"
      "d5f45ea8fbc1a41f1f11f52666ecc743443f5221feba7853386cd791"
      "89a021a9ed065378bace44c6b164a69b8ce9221b58bb75d6c0245dc6bcd2175dd82e6045"
      "4106a21c4351471d77fb189a58ea287d2a71cca839bab3ca30e5be7c",
      "fd145344a0add2b9dadb2593ba7b73d69d9c52b7cbcbd66c0996584785a3e1bbfab773ea"
      "af45ea3b8cccfa56af3cbdb739f5ee22f5827c0c73f6b7d49aa8b3a6"
      "f5756bc63ac3d1cb5f1fc5e7a777214ebdcc4eab16a79ebb7e4b6f1071dfacffe976845e"
      "2ac45772c39cca7b269189acb47299f2b068c2bcf78eda149d69ef00",
      "1138aefb97befeb3babfedeb50eeda78e9eb8bf375aba5ffded75f3e2f97be0e16efebda"
      "7e7670912d0e0b54d27ae291d6ad540fb4d2dff7f52fb43a1fc2",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 3272U, &nameCaptureInfo);
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
                emlrtMxCreateString("SoftJointDifferentialKinematics_Z2"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(6.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(12.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\SoftJointDifferentialKine"
                          "matics_Z2.m"));
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

/* End of code generation (_coder_SoftJointDifferentialKinematics_Z2_info.c) */
