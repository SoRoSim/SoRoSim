/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_variable_expmap_g_info.c
 *
 * Code generation for function 'variable_expmap_g'
 *
 */

/* Include files */
#include "_coder_variable_expmap_g_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[4] = {
      "789cc553b14ec330107550a918287462eec08c3a7560a42d2d0308355d2a8282e35c8845"
      "1c87d8a9d2af40fc04dfc22720f1333449dda695ac542a945b2ecf2f"
      "f6bb776723e3e6d640089da0223eeb456e2c7073910fd07a6cf28626ab3844b5b57d8a7f"
      "5b64c24309a92c4088192c77ba9cd11087723c8b00c5207830053767",
      "3c1ac0983230cbe02e43ecba442d414665df5d1fc88b993014fb6255615006cb7e7c69fc"
      "d6b6ecc754d38fe606ffd07fec5d5a23108063e25b26f7646bc41d2e"
      "2911ad31e781c3d3f9ea889b94b506540e134741ab473d0f620825c54e00b65abec28212"
      "db4b4222290f8535c571f103a411c391fd7cc1563ea31d7d1e57f854",
      "bc3b9f25a384db3ece87adf49f76d4af6bf50bc6e5c9dcfaefcdf555abb7ceef61aee596"
      "6623cda3aaafa75bfad4bde7063acaf3c7fbb7b14fbdf6f9e47e9f7a"
      "2afe4b2fd59cb7ed3d3dd3e83537f86167d6f1652498cb5c6fd2690f667d81bbab3aee2b"
      "74aaea401afcd7e7ff00aeb59bb9",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 1760U, &nameCaptureInfo);
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
                emlrtMxCreateString("variable_expmap_g"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\Research\\Soft Robotics Toolbox\\SoRoSim "
                          "GitHub\\SoRoSim\\Differentiable_SoRoSim\\Basic_"
                          "functions\\variable_expmap_g.m"));
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
                emlrtMxCreateString("rLq5rriT0S8a9G1JMriz8D"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_variable_expmap_g_info.c) */
