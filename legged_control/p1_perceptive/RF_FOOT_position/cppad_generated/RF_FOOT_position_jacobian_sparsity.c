void RF_FOOT_position_jacobian_sparsity(unsigned long const** row,
                                        unsigned long const** col,
                                        unsigned long* nnz) {
   static unsigned long const rows[20] = {0,0,0,0,0,0,0,1,1,1,1,1,1,1,2,2,2,2,2,2};
   static unsigned long const cols[20] = {6,9,10,11,18,19,20,7,9,10,11,18,19,20,8,10,11,18,19,20};
   *row = rows;
   *col = cols;
   *nnz = 20;
}
