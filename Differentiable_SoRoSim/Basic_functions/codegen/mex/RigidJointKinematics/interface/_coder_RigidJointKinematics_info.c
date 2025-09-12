/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_RigidJointKinematics_info.c
 *
 * Code generation for function 'RigidJointKinematics'
 *
 */

/* Include files */
#include "_coder_RigidJointKinematics_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789ced544f4fc2301ced0c1a0fa89c3c73f06c4824987853fe8ad19041a2c419e8b6ce95"
      "ad2db262b64f61fc069ebcf97d4cbcf0454c04c6604c1b30188c865e"
      "7e7b7d6d5fdfafcb03d2e9b90400d806fe788afb756b8413a3ba06a647949722eba4e9e5"
      "601dc4a6f605fcc3a86a8c72e4721f5048d078a7ce08a690f29ad746",
      "a0831c66df237dc818d846354c50350c2e06881442d4180ca8c177d6449a55ed12d0319d"
      "c90ded3018f7a327f01b9bb31f9ea01f89087f9dbfc91d29327210ec"
      "68a65265064fca4c651c6b4eb2c698ad32b73f2bb32a26c922e6a5ae1a4025870d037510"
      "e518aa366a04d327d0c15ac3e8528d63461d45c6b7582f334cf919a6",
      "88c0c1d1fb24e2b7bda0dff80cbf01aff7df94608d354cc8c3facd05f53784fa3ea3b36e"
      "bf4713bdd705f5ee847ad3fc12de37dcd24fefda14f8d899d367b44e"
      "d66f0eebf3e39bb44cbdd45ebdb24cbd60fc969e2b386fdeff7457a09788f0a58c973179"
      "db213ad18d7a2655f4f20ecc4eee5199a133eb1e408097757e4fb07f",
      "95e75ffbfd769e43bd15d66f2ea8bfcaf3614bff7d9ebfbc5fadf21cfc7c9e5b0769f732"
      "9df77254b3daeab1d52a950fadc2dfcff30fe26714a0",
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
  xInputs = emlrtCreateLogicalMatrix(1, 3);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("RigidJointKinematics"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(3.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(8.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\RigidJointKinematics.m"));
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

/* End of code generation (_coder_RigidJointKinematics_info.c) */
