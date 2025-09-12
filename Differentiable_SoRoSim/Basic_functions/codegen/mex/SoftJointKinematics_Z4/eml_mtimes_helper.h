/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_mtimes_helper.h
 *
 * Code generation for function 'eml_mtimes_helper'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void binary_expand_op(real_T in1_data[], int32_T in1_size[2], real_T in2,
                      const real_T in3_data[], const int32_T in3_size[2]);

void binary_expand_op_1(real_T in1_data[], int32_T in1_size[2], real_T in2,
                        const real_T in3_data[], const int32_T in3_size[2],
                        const real_T in4_data[], const int32_T in4_size[2]);

/* End of code generation (eml_mtimes_helper.h) */
