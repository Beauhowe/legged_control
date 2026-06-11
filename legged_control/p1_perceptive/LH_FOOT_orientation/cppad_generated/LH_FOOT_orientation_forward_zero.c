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

void LH_FOOT_orientation_forward_zero(double const *const * in,
                                      double*const * out,
                                      struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[26];

   v[0] = sin(x[10]);
   v[1] = 0 - v[0];
   v[2] = cos(x[16]);
   v[3] = cos(x[10]);
   v[4] = sin(x[11]);
   v[5] = v[3] * v[4];
   v[6] = sin(x[15]);
   v[7] = 0 - v[6];
   v[8] = cos(x[11]);
   v[9] = v[3] * v[8];
   v[10] = cos(x[15]);
   v[11] = v[5] * v[7] + v[9] * v[10];
   v[12] = sin(x[16]);
   v[13] = 0 - v[12];
   v[14] = v[1] * v[2] + v[11] * v[13];
   v[15] = sin(x[17]);
   v[11] = v[1] * v[12] + v[11] * v[2];
   v[1] = cos(x[17]);
   v[16] = v[14] * v[15] + v[11] * v[1];
   v[17] = cos(x[9]);
   v[18] = v[17] * v[3];
   v[19] = v[17] * v[0];
   v[20] = sin(x[9]);
   v[21] = v[19] * v[4] - v[20] * v[8];
   v[19] = v[19] * v[8] + v[20] * v[4];
   v[22] = v[21] * v[7] + v[19] * v[10];
   v[23] = v[18] * v[2] + v[22] * v[13];
   v[22] = v[18] * v[12] + v[22] * v[2];
   v[18] = 0 - v[15];
   v[24] = v[23] * v[1] + v[22] * v[18];
   v[0] = v[20] * v[0];
   v[25] = v[0] * v[4] + v[17] * v[8];
   v[0] = v[0] * v[8] - v[17] * v[4];
   v[17] = v[25] * v[10] + v[0] * v[6];
   v[9] = v[5] * v[10] + v[9] * v[6];
   v[20] = v[20] * v[3];
   v[0] = v[25] * v[7] + v[0] * v[10];
   v[13] = v[20] * v[2] + v[0] * v[13];
   v[0] = v[20] * v[12] + v[0] * v[2];
   v[20] = v[13] * v[15] + v[0] * v[1];
   v[12] = v[9] - v[20];
   v[22] = v[23] * v[15] + v[22] * v[1];
   v[11] = v[14] * v[1] + v[11] * v[18];
   v[14] = v[22] - v[11];
   if( v[24] > v[17] ) {
      v[23] = v[12];
   } else {
      v[23] = v[14];
   }
   v[15] = 0 - v[17];
   v[0] = v[13] * v[1] + v[0] * v[18];
   v[19] = v[21] * v[10] + v[19] * v[6];
   v[21] = v[0] - v[19];
   if( v[24] > v[17] ) {
      v[10] = 1 + v[24] - v[17] - v[16];
   } else {
      v[10] = 1 + v[17] - v[24] - v[16];
   }
   v[6] = 0 - v[17];
   if( v[24] < v[6] ) {
      v[13] = 1 + v[16] - v[24] - v[17];
   } else {
      v[13] = 1 + v[24] + v[17] + v[16];
   }
   if( v[16] < 0 ) {
      v[13] = v[10];
   } else {
      v[13] = v[13];
   }
   if( v[24] < v[15] ) {
      v[10] = v[21];
   } else {
      v[10] = v[13];
   }
   if( v[16] < 0 ) {
      v[10] = v[23];
   } else {
      v[10] = v[10];
   }
   v[23] = 0.5 / sqrt(v[13]);
   v[10] = v[10] * v[23];
   v[19] = v[0] + v[19];
   if( v[24] > v[17] ) {
      v[0] = v[19];
   } else {
      v[0] = v[13];
   }
   v[20] = v[9] + v[20];
   if( v[24] < v[6] ) {
      v[6] = v[20];
   } else {
      v[6] = v[14];
   }
   if( v[16] < 0 ) {
      v[6] = v[0];
   } else {
      v[6] = v[6];
   }
   v[6] = v[6] * v[23];
   if( v[24] > v[17] ) {
      v[19] = v[13];
   } else {
      v[19] = v[19];
   }
   v[11] = v[22] + v[11];
   if( v[24] < v[15] ) {
      v[12] = v[11];
   } else {
      v[12] = v[12];
   }
   if( v[16] < 0 ) {
      v[12] = v[19];
   } else {
      v[12] = v[12];
   }
   v[12] = v[12] * v[23];
   if( v[24] > v[17] ) {
      v[11] = v[11];
   } else {
      v[11] = v[20];
   }
   if( v[24] < v[15] ) {
      v[13] = v[13];
   } else {
      v[13] = v[21];
   }
   if( v[16] < 0 ) {
      v[13] = v[11];
   } else {
      v[13] = v[13];
   }
   v[13] = v[13] * v[23];
   y[0] = v[10] * x[24] + v[6] * x[26] - x[27] * v[12] - v[13] * x[25];
   y[1] = v[10] * x[25] + v[13] * x[24] - x[27] * v[6] - v[12] * x[26];
   y[2] = v[10] * x[26] + v[12] * x[25] - x[27] * v[13] - v[6] * x[24];
}

