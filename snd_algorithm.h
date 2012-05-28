#ifndef SND_ALGORITHM_H
#define SND_ALGORITHM_H

#include <iostream>
#include <assert.h>
#include <vector>
#include <pthread.h>

#include "spaces.h"
#include "snd_data.h"

using namespace std;

typedef unsigned int uint;

class Gap {
    public:
        Angle bearing;
        double distance;
        int iDir;
        
        Gap(){ bearing = 0.0; distance = 0.0; iDir = 0; };
        Gap( Angle ang, double d, int iD ){ bearing = ang; distance = d; iDir = iD; };
        Gap( const Gap &g ){ bearing = g.bearing; distance = g.distance; iDir = g.iDir; };
};

class Valley {
    public:
        Gap risingGap;
        Gap otherGap;
        
        Valley(){};
        Valley( Gap rG, Gap oG ){ risingGap = rG; otherGap = oG; };
};


// Use gDebug to set verbosity of output, -1 for silent, 0 for hardly any output, 5 for normal debug
static int gDebug=9;

class SND_algorithm 
{
    private:
        SND_data * robot;

        double robotRadius;
        double minGapWidth;
        double obstacleAvoidDist;
        double maxSpeed;
        double maxTurnRate;
        double goalPositionTol;
        double goalAngleTol;

        double fMaxRange;
        double fScanRes;
        uint iNumLPs;

        Pose robotPose;
        Pose goal;
		  
		  bool scanValid;
		  bool goalValid;
		  bool poseValid;

        vector<pair<double,Angle> > laserScan;
        vector<Gap> gapVec;
        Valley* pBestValley;
        Angle safeRisingGapAngle;
        Angle midValleyAngle;
        double obsAvoidDelta;

        Angle driveAngle;
        
        bool isFilterClear( const Angle &testBearing, const double &width, const double &forwardLength, bool bDoRearCheck );
        bool isRisingGapSafe( Gap &risingGap );
        void buildGapVector( );
        void removeDuplicateGaps( );
        void findBestValley( Position distToGoal );
        void setObsAvoidDelta( double safetyDist );
        
    public :
        SND_algorithm( SND_data * r );
        bool isReady() { return (scanValid && goalValid && poseValid); };
        void step( );
};

#endif //SND_ALGORITHM_H