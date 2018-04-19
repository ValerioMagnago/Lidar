//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: test_emxutil.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 19-Apr-2018 12:26:38
//
#ifndef __TEST_EMXUTIL_H__
#define __TEST_EMXUTIL_H__

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "test_types.h"

// Function Declarations
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for test_emxutil.h
//
// [EOF]
//
