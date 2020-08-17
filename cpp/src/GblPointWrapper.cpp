#include "GblPoint.h"
#include "Eigen/Core"


const int NROW = 5;
const int NCOL = 5;

using namespace gbl;
using namespace Eigen;

extern "C" { 
    GblPoint* GblPointCtor(double matrixArray[NROW*NCOL]) {
        
        Map<Matrix5d> jacobian(matrixArray,5,5);
        return new GblPoint(jacobian);
        
    }
    
    void GblPoint_printPoint(const GblPoint* self, unsigned int level) {
        self->printPoint(level);
    }
    
    unsigned int GblPoint_hasMeasurement(const GblPoint* self) {
        return self->hasMeasurement();
    }
    
    
    double GblPoint_getMeasPrecMin(const GblPoint* self) {
        return self->getMeasPrecMin();
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
        
        self->addMeasurement(aProjection, aResiduals, aPrecision, minPrecision);
    }

    
    //Only support vector precision
    void GblPoint_addScatterer(GblPoint* self, double *resArray, double *precArray) {
        
        Map<Vector2d> aResiduals(resArray,2);
        Map<Vector2d> aPrecision(precArray,2);
        
        self->addScatterer(aResiduals,aPrecision);
    }
    
    //Add global derivatives (6x6 only!) - FIX FIX FIX 
    void GblPoint_addGlobals(GblPoint* self, int *labels, int nlabels, double* derArray) {
        std::vector<int> aLabels;
        for (int i=0; i<6; i++) {
            aLabels.push_back(labels[i]);
        }
        Map<Eigen::Matrix<double,1,6> > derivatives(derArray,1,6);
        self->addGlobals(aLabels, derivatives);
    }

    int GblPoint_getNumGlobals(GblPoint* self) {
        return self->getNumGlobals();
    }
    
    //TODO revisit these!

    //Should I add the number of nlabels?
    void GblPoint_getGlobalLabels(GblPoint* self, int* labels) {
        
        std::vector<int> glabels;
        self->getGlobalLabels(glabels);
        
        for (unsigned int il = 0 ; il < glabels.size(); il++) {
            labels[il] = glabels.at(il);
        }
    }  

    //Should I add the number of derivatives?
    void GblPoint_getGlobalDerivatives(GblPoint* self, double* gders) {
        
        int ngders = self->getNumGlobals();
        Eigen::MatrixXd e_gders(ngders,ngders);
        self->getGlobalDerivatives(e_gders);
        
        Map<MatrixXd>(gders,ngders,ngders) = e_gders;
                
    }

}

