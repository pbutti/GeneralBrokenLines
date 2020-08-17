#include "GblTrajectory.h"

using namespace gbl;
using namespace Eigen;

extern "C" {
    
    GblTrajectory* GblTrajectoryCtor(int flagCurv, int flagU1dir, int flagU2dir) {
        
        return new GblTrajectory(flagCurv, flagU1dir, flagU2dir);
        
    }
        
    //Simple trajectory constructor wrapper
    GblTrajectory* GblTrajectoryCtorPtrArray(GblPoint* points[], int npoints, 
                                             int flagCurv, int flagU1dir, int flagU2dir) {
        
        
        std::vector<GblPoint> aPointList;
        
        for (int i=0; i<npoints; i++) {
            //get the point pointer
            GblPoint* gblpoint = points[i];
            
            //add it to the vector
            aPointList.push_back(*(gblpoint));
        }
        
        return new GblTrajectory(aPointList, flagCurv, flagU1dir, flagU2dir);
    }

    //Simple trajectory constructor with seed wrapper
    
    GblTrajectory* GblTrajectoryCtorPtrArraySeed(GblPoint* points[], int npoints,
                                                 int aLabel, double seedArray[],
                                                 int flagCurv, int flagU1dir, int flagU2dir) {
        
        std::vector<GblPoint> aPointList;
        
        for (int i=0; i<npoints; i++) {
            //get the point pointer
            GblPoint* gblpoint = points[i];
            
            //add it to the vector
            aPointList.push_back(*(gblpoint));
        }
        
        Map<Matrix5d> seed(seedArray,5,5);
        
        return new GblTrajectory(aPointList, aLabel, seed, flagCurv, flagU1dir, flagU2dir);
        
    }
    
    void GblTrajectory_fit(GblTrajectory* self, double* Chi2, int* Ndf, double* lostWeight, char* c_optionList, unsigned int aLabel) {
        
        std::string optionList(c_optionList);
        self->fit(*Chi2, *Ndf, *lostWeight, optionList,aLabel);
    }
    
    int GblTrajectory_isValid(GblTrajectory* self) {
        
        return (int) self->isValid();
    }

    void GblTrajectory_printTrajectory(GblTrajectory* self, int level) {
        return self->printTrajectory();
    }

    void GblTrajectory_printData(GblTrajectory* self) {
        return self->printData();
    }
    
    void GblTrajectory_printPoints(GblTrajectory* self, int level) {
        return self->printPoints(level);
    }

    //Only 5-vector and 5x5 cov matrix.
    void GblTrajectory_getResults(GblTrajectory* self, int aSignedLabel, double* localPar, int* nLocalPar,
                                  double * localCov, int* sizeLocalCov) {

        Eigen::VectorXd e_localPar(5);
        Eigen::MatrixXd e_localCov(5,5);
        
        self->getResults(aSignedLabel, e_localPar, e_localCov);

        //std::cout<<"gblTrajectoryWrapper::getResults"<<std::endl;
        //std::cout<<e_localPar<<std::endl;
        //std::cout<<e_localCov<<std::endl;
        
        Map<Vector5d>(localPar, 5) = e_localPar;
        Map<Matrix5d>(localCov, 5, 5) = e_localCov; 
        *nLocalPar = 5;
        *sizeLocalCov = 5;
        
    }

    void GblTrajectory_milleOut(GblTrajectory* self, MilleBinary* millebinary) {
        self->milleOut(*millebinary);
    }

    
    

}



