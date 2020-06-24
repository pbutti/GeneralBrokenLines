#include "GblPoint.h"

const int M = 5;
const int N = 5;

extern "C" gbl::GblPoint *GblPointCtor(double matrix[M][N]); 
gbl::GblPoint* GblPointCtor(double matrix[M][N]) { 
    gbl::Matrix5d jacobian; 
    for (int i{0}; i < 5; i++) {
        for (int j{0}; j < 5; j++) { 
            jacobian(i, j) = matrix[i][j]; 
        }
    }

    return new gbl::GblPoint(jacobian); 
}
