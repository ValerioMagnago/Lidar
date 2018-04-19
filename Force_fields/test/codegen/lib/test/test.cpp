//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: test.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 19-Apr-2018 12:26:38
//

// Include Files
#include "rt_nonfinite.h"
#include "test.h"
#include "test_emxutil.h"

// Function Definitions

//
// Arguments    : double n
//                emxArray_real_T *data
// Return Type  : void
//
void test(double n, emxArray_real_T *data)
{
  int i0;
  int i;
  emxArray_real_T *r0;
  int loop_ub;
  i0 = data->size[0] * data->size[1];
  data->size[0] = 1;
  data->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)data, i0, (int)sizeof(double));
  i = 0;
  emxInit_real_T(&r0, 2);
  while (i <= (int)n - 1) {
    i0 = r0->size[0] * r0->size[1];
    r0->size[0] = 1;
    r0->size[1] = 1 + data->size[1];
    emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(double));
    r0->data[0] = 1.0 + (double)i;
    loop_ub = data->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      r0->data[r0->size[0] * (i0 + 1)] = data->data[data->size[0] * i0];
    }

    i0 = data->size[0] * data->size[1];
    data->size[0] = 1;
    data->size[1] = r0->size[1];
    emxEnsureCapacity((emxArray__common *)data, i0, (int)sizeof(double));
    loop_ub = r0->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      data->data[data->size[0] * i0] = r0->data[r0->size[0] * i0];
    }

    i++;
  }

  emxFree_real_T(&r0);
}

//
// File trailer for test.cpp
//
// [EOF]
//
