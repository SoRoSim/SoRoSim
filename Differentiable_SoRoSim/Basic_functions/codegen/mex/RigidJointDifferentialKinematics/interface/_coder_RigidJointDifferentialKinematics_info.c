/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_RigidJointDifferentialKinematics_info.c
 *
 * Code generation for function 'RigidJointDifferentialKinematics'
 *
 */

/* Include files */
#include "_coder_RigidJointDifferentialKinematics_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789ced54cb4ac340149d48151755bb72dd856b29582ab8d33eac15a524052d56ea249998"
      "693233b5994af201aec59f7027f839823fe04f08264dd32681d06a4b"
      "45e9dddc9c393373e6de1b0e104ece0400c026f0e335ede78d21ce0cf30a88469c1762fb"
      "84e876b00a52917301ff38cc0aa31cd9dc071412343aa9328229a4bc",
      "e17411e8218b99f7481d301a365103132485c1b9874825448d804779df451d2986d427a0"
      "a75be3179a6130eac74742bda929fbf190d08f4c8cbf2a5f970e5a22"
      "b210ec297a4b621acf8a4c661c2b56b6c1982933db5d15998449f618f36a5f0e60ab8435"
      "0df510e518ca266a07cb47d0c24a5beb53856346ad96886fb15a6398",
      "f2d001f3145344a027b34b42757767ac3b3da1ee8057ddd912acb0b60e07c30ff46f66d4"
      "5f4bd4f71995f5dd5e8df5de66d4bb4bd48bf20b9873b8a583997a31"
      "a9af5b53d619cfe3fdeb83fcfcf42e2c522fb7d3ac2f522f88dfd2b313ee9bf63fdd4ed0"
      "cbc4f86ac129e8bc6b1195a85ab3903b76ca162c8edf519fa033e91d",
      "20012feafea5afffacee6ffb3a543b002c7d7d9ebeeeb6f4dffbfacbe7e5d2d7c1fc7ddd"
      "d8cbdb17f9b253a28ad1950f8d4eb5b66f54febeaf7f01266d1e8a",
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
  xInputs = emlrtCreateLogicalMatrix(1, 5);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("RigidJointDifferentialKinematics"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(5.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(12.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\RigidJointDifferentialKin"
                          "ematics.m"));
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

/* End of code generation (_coder_RigidJointDifferentialKinematics_info.c) */
