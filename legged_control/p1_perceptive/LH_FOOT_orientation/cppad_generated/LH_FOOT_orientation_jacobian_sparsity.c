void LH_FOOT_orientation_jacobian_sparsity(unsigned long const** row,
                                           unsigned long const** col,
                                           unsigned long* nnz) {
   static unsigned long const rows[18] = {0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,2};
   static unsigned long const cols[18] = {9,10,11,15,16,17,9,10,11,15,16,17,9,10,11,15,16,17};
   *row = rows;
   *col = cols;
   *nnz = 18;
}
