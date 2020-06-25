#include "GblPoint.h"


const int NROW = 5;
const int NCOL = 5;

using namespace gbl;

extern "C" gbl::GblPoint *GblPointCtor(double matrixArray[NROW*NCOL]);
gbl::GblPoint* GblPointCtor(double matrixArray[NROW*NCOL]) {
    gbl::Matrix5d jacobian;
    std::cout<<"Constructor!"<<NROW<<" " << NCOL<<std::endl;
    for (unsigned int i=0; i < NROW; ++i){
        for (unsigned int j=0; j<NCOL; ++j) {
            jacobian(i,j) = matrixArray[i*NCOL+j];
        }
    }
    std::cout<<"Constructed"<<std::endl;
    std::cout<<jacobian<<std::endl;
    return new gbl::GblPoint(jacobian);
    
}

extern "C" unsigned int GblPoint_hasMeasurement(const GblPoint* self) {
    return self->hasMeasurement();
}


extern "C" double GblPoint_getMeasPrecMin(const GblPoint* self) {
    return self->getMeasPrecMin();
}










