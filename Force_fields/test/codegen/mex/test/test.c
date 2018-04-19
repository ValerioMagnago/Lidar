/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * test.c
 *
 * Code generation for function 'test'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "test.h"
#include "test_emxutil.h"
#include "test_data.h"

/* Variable Definitions */
static emlrtRTEInfo emlrtRTEI = { 1, 19, "test",
  "/media/marco/Data/Cloud/Mega/Dottorato/Materiale Ricerca/Tartufino/AutoTartu/Force_fields/test/test.m"
};

static emlrtRTEInfo c_emlrtRTEI = { 4, 5, "test",
  "/media/marco/Data/Cloud/Mega/Dottorato/Materiale Ricerca/Tartufino/AutoTartu/Force_fields/test/test.m"
};

/* Function Definitions */
void test(const emlrtStack *sp, real_T n, emxArray_real_T *data)
{
  int32_T i0;
  int32_T i;
  emxArray_real_T *r0;
  int32_T loop_ub;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  i0 = data->size[0] * data->size[1];
  data->size[0] = 1;
  data->size[1] = 0;
  emxEnsureCapacity(sp, (emxArray__common *)data, i0, (int32_T)sizeof(real_T),
                    &emlrtRTEI);
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, n, mxDOUBLE_CLASS, (int32_T)n,
    &c_emlrtRTEI, sp);
  i = 0;
  emxInit_real_T(sp, &r0, 2, &emlrtRTEI, true);
  while (i <= (int32_T)n - 1) {
    i0 = r0->size[0] * r0->size[1];
    r0->size[0] = 1;
    r0->size[1] = 1 + data->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)r0, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    r0->data[0] = 1.0 + (real_T)i;
    loop_ub = data->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      r0->data[r0->size[0] * (i0 + 1)] = data->data[data->size[0] * i0];
    }

    i0 = data->size[0] * data->size[1];
    data->size[0] = 1;
    data->size[1] = r0->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)data, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    loop_ub = r0->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      data->data[data->size[0] * i0] = r0->data[r0->size[0] * i0];
    }

    i++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(&r0);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (test.c) */
