#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void RF_FOOT_position_forward_zero(double const *const * in,
                                   double*const * out,
                                   struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[19];

   v[0] = cos(x[9]);
   v[1] = sin(x[10]);
   v[2] = v[0] * v[1];
   v[3] = sin(x[11]);
   v[4] = sin(x[9]);
   v[5] = cos(x[11]);
   v[6] = v[2] * v[3] - v[4] * v[5];
   v[7] = cos(x[10]);
   v[8] = v[0] * v[7];
   v[9] = cos(x[18]);
   v[2] = v[2] * v[5] + v[4] * v[3];
   v[10] = sin(x[18]);
   v[11] = v[6] * v[9] + v[2] * v[10];
   v[12] = sin(x[19]);
   v[13] = 0 - v[10];
   v[2] = v[6] * v[13] + v[2] * v[9];
   v[14] = cos(x[19]);
   v[15] = v[8] * v[12] + v[2] * v[14];
   v[16] = 0 - v[12];
   v[17] = sin(x[20]);
   v[18] = cos(x[20]);
   y[0] = -0.072 * v[6] + 0.259 * v[8] + x[6] + -0.0395 * v[11] + 0.089 * v[8] + -0.35 * v[15] + -0.1323 * v[11] + -0.35 * ((v[8] * v[14] + v[2] * v[16]) * v[17] + v[15] * v[18]);
   v[15] = v[4] * v[1];
   v[2] = v[15] * v[3] + v[0] * v[5];
   v[4] = v[4] * v[7];
   v[15] = v[15] * v[5] - v[0] * v[3];
   v[0] = v[2] * v[9] + v[15] * v[10];
   v[15] = v[2] * v[13] + v[15] * v[9];
   v[11] = v[4] * v[12] + v[15] * v[14];
   y[1] = -0.072 * v[2] + 0.259 * v[4] + x[7] + -0.0395 * v[0] + 0.089 * v[4] + -0.35 * v[11] + -0.1323 * v[0] + -0.35 * ((v[4] * v[14] + v[15] * v[16]) * v[17] + v[11] * v[18]);
   v[3] = v[7] * v[3];
   v[1] = 0 - v[1];
   v[7] = v[7] * v[5];
   v[10] = v[3] * v[9] + v[7] * v[10];
   v[7] = v[3] * v[13] + v[7] * v[9];
   v[12] = v[1] * v[12] + v[7] * v[14];
   y[2] = -0.072 * v[3] + 0.259 * v[1] + x[8] + -0.0395 * v[10] + 0.089 * v[1] + -0.35 * v[12] + -0.1323 * v[10] + -0.35 * ((v[1] * v[14] + v[7] * v[16]) * v[17] + v[12] * v[18]);
}

