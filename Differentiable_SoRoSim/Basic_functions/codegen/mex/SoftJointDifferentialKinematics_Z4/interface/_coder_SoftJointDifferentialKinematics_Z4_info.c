/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_SoftJointDifferentialKinematics_Z4_info.c
 *
 * Code generation for function 'SoftJointDifferentialKinematics_Z4'
 *
 */

/* Include files */
#include "_coder_SoftJointDifferentialKinematics_Z4_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789ced55cb4ac340149d48151755bb72dd856b29582ab8d33eac15a5b405ad56ea249990"
      "693233319948fa052ec59f7027f839823fe04f08264dd3a681906a4b"
      "45e9dddc9c9c3b73e6de09274038391300009bc08fd7b49f37863833cc2b6032a2bc10a9"
      "1326cbc12a484dac0bf8c7619618e5c8e13ea090a0d14a99114c21e5",
      "adbe8180892ca6df2379c02858472d4c50330cce3d442a216a043cca7b2eaa48d29a3601"
      "a66a8d4fa887c1681e1f31fda6a69cc743cc3c3211feba7c533ae834"
      "9085a029a99d265378b6c144c6b164655b8ce92273dcb70dd6c4247b8c79d51603d82961"
      "454126a21c435147dde0f511b4b0d4556c2a71cca835d8b3c630e5a1",
      "7afd145344a0a7d2bdcaef9251dfc68c7da713fa0e78d9bd5b8225d68572cfc381feed8c"
      "fa6bb1fa3e2333db9dd558ef6d46bdbb58bd497e01f71c1ea97ba37e"
      "24cd756bca3ea3795cbf3ec8cf4fefc222f55e3e2feb8bd40be2b7f49c98fda6fd4eb763"
      "f432115edbcb3b17f972bf4425cd100fb55eb5b6af55c6e7a827e824",
      "9d03c4e045edbff4f59ff5fd6d5f57e1e0a7bef4f5f9f9ba3bd27fefebb99df6d2d7c1fc"
      "7dbd5ae817546e584426b2d22ee48efb650b16ffbeaf7f01eaed1fc6",
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
  xInputs = emlrtCreateLogicalMatrix(1, 8);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("SoftJointDifferentialKinematics_Z4"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(8.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(12.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\SoftJointDifferentialKine"
                          "matics_Z4.m"));
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

/* End of code generation (_coder_SoftJointDifferentialKinematics_Z4_info.c) */
