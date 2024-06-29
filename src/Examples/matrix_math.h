#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

void Matrix_mult(float *A, float *B, float *C, uint8_t dim1, uint8_t dim2, uint8_t dim3){
  // A and B are matrices to multiply, C is resultant matrix. dim 1 and 2 are the dimensions of A, dim3 is outer dimension of B
  for (uint8_t i=0;i<dim1;i++){
    for (uint8_t j=0;j<dim3;j++){
      for (uint8_t k=0;k<dim2;k++){
        *C += *(A++)**B;
        B += dim3;
      }
      ++C;
      A-=dim2;
      B-=(dim3*dim2);
      ++B;
    }
    B -= dim3;
    A += dim2;
  }
}

void Matrix_add(float *A, float *B, float *C, uint8_t dim1, uint8_t dim2){
  // A and B are matrices to add, C is the resultant. dim1 and 2 are the dimensions of A and B
  for (uint8_t i=0;i<(dim1*dim2);i++){
    *C++ = *(A++)+*(B++);
  }
}

void Matrix_sub(float *A, float *B, float *C, uint8_t dim1, uint8_t dim2){
  // A and B are matrices to subtract, C is the resultant. dim1 and 2 are the dimensions of A and B
  for (uint8_t i=0;i<(dim1*dim2);i++){
    *C++ = *(A++)-*(B++);
  }
}

void Matrix_trans(float *A, float *B, uint8_t dim1, uint8_t dim2){
  // A is the matrix to be transposed and B is the resultant, dim1 and 2 are the dimensions of A
  for (uint8_t i=0;i<dim2;i++){
    for (uint8_t j=0;j<dim1;j++){
      *(B++) = *A;
      A += dim2;
    }
    A -= (dim1*dim2);
    ++A;
  }
}

void Matrix_det2(float *A, float *a){
  float *temp = A+3;
  *a = (*A**temp) - (*(++A)**(++A));
}

void Matrix_det3(float A[9], float *a){
  float temp1[4] = {A[4],A[5],A[7],A[8]};
  float temp2[4] = {A[3],A[5],A[6],A[8]};
  float temp3[4] = {A[3],A[4],A[6],A[7]};
  float temp4[3];
  Matrix_det2(temp1,&temp4[0]);
  Matrix_det2(temp2,&temp4[1]);
  Matrix_det2(temp3,&temp4[2]);
  *a = A[0]*temp4[0] - A[1]*temp4[1] + A[2]*temp4[2];
}

void Matrix_adj(float A[9], float *B){
  // A iA the matrix to take the adjoint of, B is a pointer to the start of the resultant matrix
  *B++ = A[4]*A[8] - A[5]*A[7]; 
  *B++ = A[2]*A[7] - A[1]*A[8]; 
  *B++ = A[1]*A[5] - A[2]*A[4];
  *B++ = A[5]*A[6] - A[3]*A[8]; 
  *B++ = A[0]*A[8] - A[2]*A[6]; 
  *B++ = A[2]*A[3] - A[0]*A[5];
  *B++ = A[3]*A[7] - A[4]*A[6]; 
  *B++ = A[1]*A[6] - A[0]*A[7]; 
  *B = A[0]*A[4] - A[1]*A[3];
}

void Matrix_inv3(float A[9], float B[9]){
  float temp;
  Matrix_det3(A,&temp);
  float temp2 = 1/temp;
  float C[9];
  Matrix_adj(A,C);
  for (uint8_t i=0;i<9;i++){
    B[i] = temp2*C[i];
  }
}

#endif