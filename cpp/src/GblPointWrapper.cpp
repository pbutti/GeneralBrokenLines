#include "GblPoint.h"
#include "Eigen/Core"


const int NROW = 5;
const int NCOL = 5;

using namespace gbl;
using namespace Eigen;

extern "C" { 
    gbl::GblPoint* GblPointCtor(double matrixArray[NROW*NCOL]) {

        Map<Matrix5d> jacobian(matrixArray,5,5);
        std::cout<<"Constructor"<<std::endl;
        std::cout<<jacobian<<std::endl;
        return new gbl::GblPoint(jacobian);
        
    }
    
    unsigned int GblPoint_hasMeasurement(const GblPoint* self) {
        return self->hasMeasurement();
    }
    
    
    double GblPoint_getMeasPrecMin(const GblPoint* self) {
        return self->getMeasPrecMin();
    }
    
    //Just for test
    void GblPoint_addJacobian(const GblPoint* self, double *array, int rows, int cols) {
        Map<Matrix5d> jacobian(array,rows,cols);
        std::cout<<jacobian<<std::endl;
    }
    
    //Only supporting:
    //2D position residual
    //2x2 projection matrix
    
    void GblPoint_addMeasurement2D(GblPoint* self, 
                                   double *projArray,
                                   double *resArray,
                                   double *precArray, 
                                   double minPrecision) { 
        
        Map<Matrix2d> aProjection(projArray,2,2);
        Map<Vector2d> aResiduals(resArray, 2);
        Map<Vector2d> aPrecision(precArray,2);
        
        std::cout<<"addMeasurement2D::"<<std::endl;
        std::cout<<aProjection<<std::endl;
        std::cout<<aResiduals<<std::endl;
        std::cout<<aPrecision<<std::endl;
        
        self->addMeasurement(aProjection, aResiduals, aPrecision, minPrecision);
    }

    
    //Only support vector precision
    void GblPoint_addScatterer(GblPoint* self, double *resArray, double *precArray) {
        
        Map<Vector2d> aResiduals(resArray,2);
        Map<Vector2d> aPrecision(precArray,2);
        
        std::cout<<"addScatterer"<<std::endl;
        std::cout<<"scatterer\n"<<aResiduals<<std::endl;
        std::cout<<"precision\n"<<aPrecision<<std::endl;
        
        self->addScatterer(aResiduals,aPrecision);
    }
        

}


/*
extern "C" void GblPoint_addMeasurement(const GblPoint* self, double projArray[], double resArray[], double precArray[], double minPrecision = 0.) {
    //form the Eigen matrices
}
*/

/*
extern "C" void GblPoint_addMeasurement(const GblPoint* self, double projArray[], double resArray[], double precArray[]) {
    return GblPoint_addMeasurement(self,projArray,resArray,precArray,0.);
}
*/  





