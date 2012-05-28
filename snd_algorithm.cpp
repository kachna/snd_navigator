/*  Smooth ND Alghoritm based on Player's snd_driver by:
 *  
 *                Joey Durham (algorithm) , 
 *                Luca Invernizzi (driver implementation)
 *
 *  Implemented on top of Player - One Hell of a Robot Server
 *  Copyright (C) 2003  (Brian Gerkey, Andrew Howard)
 *  
 *  Modificated for ROS by Petr Martinec
 * 
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <iostream>
#include <assert.h>
#include <vector>
//#include <boost/timer.hpp>

#include "snd_algorithm.h"




SND_algorithm::SND_algorithm( SND_data* r )
//SND_algorithm::SND_algorithm()
{
/*
	robotRadius = 0.2;
	obstacleAvoidDist = 2*robotRadius;
	maxSpeed = 0.6;
	maxTurnRate = 60;
	minGapWidth = 2.2*robotRadius;	
	goalPositionTol = 0.5*robotRadius;
	goalAngleTol = 15;

   fMaxRange = 0; //robot->GetMaxRange();
   fScanRes = 0; //robot->GetScanRes();
   iNumLPs = 0; //robot->GetCount();
*/

	
	robot = r;
	
	robotRadius = robot->robot_radius;
	minGapWidth = robot->min_gap_width;
	obstacleAvoidDist = robot->obstacle_avoid_dist;
	maxSpeed = robot->max_speed;
	maxTurnRate = robot->max_turn_rate;
	goalPositionTol = robot->goal_position_tol;
	goalAngleTol = robot->goal_angle_tol;

	fMaxRange = robot->GetMaxRange();
	fScanRes = robot->GetScanRes();
	iNumLPs = robot->GetCount();
	
	pBestValley = NULL;


/*
    if( gDebug >= 0 ) std::cout << "Starting SND driver 3.0 ..." << std::endl;
    if( gDebug >= 0 ) std::cout << "Robot radius: " << robotRadius << "; obstacle_avoid_dist " << obstacleAvoidDist << std::endl;
    if( gDebug >= 0 ) std::cout << "Pos tol: " << goalPositionTol << "; angle tol " << goalAngleTol << std::endl;
    
    while( iNumLPs <= 0 || iNumLPs > 100000 || fScanRes <= M_PI/1000 || fScanRes > 1.0 )
    {
        if (gDebug > 0) std::cout << "Waiting for real data" << std::endl;
/*
		  robot->Read();
        iNumLPs = robot->GetCount();
        fMaxRange = robot->GetMaxRange();
        fScanRes = robot->GetScanRes();

		  
    }
    
    if( gDebug >= 0 ) std::cout << "SND driver ready" << std::endl;
    
    if( gDebug > 0 )
    {
        std::cerr<< std::endl;    
        std::cerr<< "Robot at " << robot->GetXPos() << ", " << robot->GetYPos() << std::endl;
        std::cerr<< std::endl;
    }
*/
}

bool SND_algorithm::isFilterClear( const Angle &testBearing, const double &width, const double &forwardLength, bool bDoRearCheck )
{
    for( uint i = 0; i < laserScan.size(); i++ )
    {
    double deltaAngle = laserScan[i].second.alDiff(testBearing);
        
        if( fabs(deltaAngle) > M_PI/2  )
        {
            // Semi-circle behind sensor
            if( bDoRearCheck && (laserScan[i].first < width/2.0) )
            {
                if( gDebug > 7 ) std::cout<< "  Filter:  obstacle at " << laserScan[i].second.print() << " in rear semi-circle" << std::endl;
                return false;
            }
        }
        else
        {
            // Rectangle in front of robot
            double d1 = /*std::*/fabs((width/2.0)/(sin(deltaAngle)));
            double d2 = /*std::*/fabs((forwardLength)/(cos(deltaAngle)));
            
            if( laserScan[i].first < std::min(d1,d2) )
            {
                if( gDebug > 7 ) std::cout<< "  Filter: obstacle at " << laserScan[i].second.print() << " in front rectangle" << std::endl;
                return false;
            }
        }
    }
    
    return true;
}

bool SND_algorithm::isRisingGapSafe( Gap &risingGap )
{
    // TODO: only checks if point creating gap is to close to obstacle on other side ...
    // does not guarantee safe passage through gap
    
    Position posGapCorner = Position( risingGap.distance, risingGap.bearing );

    for( uint i = 0; i < laserScan.size(); i++ )
    {
        double deltaAngle = risingGap.iDir * laserScan[i].second.alDiff(risingGap.bearing);
        
        if( deltaAngle > 0 && deltaAngle < M_PI/2 )
        {
            if( laserScan[i].first < fMaxRange - 0.01 )
            {
                if( posGapCorner.dist(Position(laserScan[i].first, laserScan[i].second)) < minGapWidth )
                {
                    if( gDebug > 4 ) cout<< "    Gap at " << risingGap.bearing.print() << " ruled unsafe by obstacle at " << laserScan[i].second.print() << endl;
                    return false;
                }
            }
        }
    }
    
    return true;
}

void SND_algorithm::buildGapVector( )
{
    pair<double,Angle> rayR;
    pair<double,Angle> rayL;
    
    gapVec.clear();
    
    iNumLPs = robot->GetCount();
    
    if( iNumLPs <= 0 )
    {
        return;
    }
    
    // Right edge of scan is right gap
    rayR = laserScan[iNumLPs - 1];
    rayL = laserScan[0];
    
    double dist = (rayR.first * rayR.first) + (rayL.first * rayL.first) - 2 * rayR.first * rayL.first * cos( rayR.second.ccwDiff(rayL.second) );
    
    if( dist >= minGapWidth )
    {
        Gap newGap( rayL.second, -robotRadius, -1 );
        gapVec.push_back(newGap);
    }
    
    for( uint i = 1; i < iNumLPs - 1; i++ )
    {
        rayR = rayL;
        rayL = laserScan[i];
        
        dist = rayL.first - rayR.first;
        
        if( (dist >= minGapWidth) || (rayL.first == fMaxRange && rayR.first < fMaxRange) )
        {
            Gap newGap( rayR.second, rayR.first, -1 );
            gapVec.push_back(newGap);
        }
        else if( (dist <= -minGapWidth) || (rayR.first == fMaxRange && rayL.first < fMaxRange))
        {
            Gap newGap( rayL.second, rayL.first, 1 );
            gapVec.push_back(newGap);
        }
    }
    
    // Left edge of scan is a left gap
    rayR = laserScan[iNumLPs - 1];
    rayL = laserScan[0];
    
    dist = (rayR.first * rayR.first) + (rayL.first * rayL.first) - 2 * rayR.first * rayL.first * cos( rayR.second.ccwDiff(rayL.second) );
    
    if( dist >= minGapWidth )
    {
        Gap newGap( rayR.second, -robotRadius, 1 );
        gapVec.push_back(newGap);
    }
}

void SND_algorithm::removeDuplicateGaps( )
{
    std::vector<Gap>::iterator iterR, iterL;
    iterL = gapVec.begin();
    iterR = iterL;
    iterL++;
    int iCount = 0;
    while( iterL != gapVec.end() && iterR != gapVec.end() && gapVec.size() > 1)
    {
        if( ((*iterR).bearing.ccwDiff((*iterL).bearing) < 2*fScanRes) && ((*iterL).iDir == 1) && ((*iterR).iDir == 1) )
        {
            if( gDebug > 4 ) cout<< "    Removed duplicate gap at " << (*iterR).bearing.print() << endl;
            iterL = gapVec.erase(iterR);
        }
    
        iterR = iterL;
        iterL++;
        iCount++;
    }
    
    if( iterR != gapVec.end() && gapVec.size() > 1 )
    {
        iterL = gapVec.begin();
        
        if( ((*iterR).bearing.ccwDiff((*iterL).bearing) < 2*fScanRes) && ((*iterL).iDir == 1) && ((*iterR).iDir == 1) )
        {
            if( gDebug > 4 ) cout<< "    Removed duplicate gap at " << (*iterR).bearing.print() << endl;
            gapVec.erase(iterR);
        }
        iCount++;
    }
    
    std::vector<Gap>::reverse_iterator riterR, riterL;
    riterR = gapVec.rbegin();
    riterL = riterR;
    riterR++;
    iCount = 0;
    while( riterL != gapVec.rend() && riterR != gapVec.rend() && gapVec.size() > 1 )
    {
        if( ((*riterR).bearing.ccwDiff((*riterL).bearing) < 2*fScanRes) && ((*riterL).iDir == -1) && ((*riterR).iDir == -1) )
        {
            if( gDebug > 4 ) cout<< "    Removed duplicate gap at " << (*(riterL + 1).base()).bearing.print() << endl;
            gapVec.erase((riterL+1).base());
        }
    
        riterL = riterR;
        riterR++;
        iCount++;
    }
    
    if( riterL != gapVec.rend() && gapVec.size() > 1 )
    {
        riterR = gapVec.rbegin();
        
        if( ((*riterR).bearing.ccwDiff((*riterL).bearing) < 2*fScanRes) && ((*riterL).iDir == -1) && ((*riterR).iDir == -1) )
        {
            if( gDebug > 4 ) cout<< "    Removed duplicate gap at " << (*(riterL+1).base()).bearing.print() << endl;
            gapVec.erase((riterL+1).base());
        }
        iCount++;
    }
}

void SND_algorithm::findBestValley( Position distToGoal )
{
    uint iR, iL;
    int iBestValleyRising = -1;
    int iBestValleyOther  = -1;
    
    for( iR = 0; iR < gapVec.size(); iR++ )
    {
        iL = (iR + 1)%(gapVec.size());
        
        int iRisingGap = -1;
        int iOtherGap = -1;
        
        if( gapVec[iR].iDir == -1 )
        {
            if( isRisingGapSafe(gapVec[iR]) )
            {
                iRisingGap = iR;
                iOtherGap = iL;
            }
            else
            {
                if( gDebug > 2 ) cout<< " Potential rising gap at " << gapVec[iR].bearing.print() << " is not safe" << endl;
            }
        }
        
        if( gapVec[iL].iDir == 1 )
        {
            if( isRisingGapSafe(gapVec[iL]))
            {
                if( iRisingGap >= 0 )
                {
                    // Both gaps are rising, pick one closer to goal
                    
                    // Angular proximity
                    if( fabs(gapVec[iL].bearing.alDiff(distToGoal.bearing())) < fabs(gapVec[iRisingGap].bearing.alDiff(distToGoal.bearing())) )
                    {
                        iRisingGap = iL;
                        iOtherGap = iR;
                    }
                    
                    // Physical proximity
                    /*if( distToGoal.dist( Position(gapVec[iL].distance, gapVec[iL].bearing) ) <
                        distToGoal.dist( Position(gapVec[iR].distance, gapVec[iR].bearing) ) )
                    {
                        iRisingGap = iL;
                        iOtherGap = iR;
                    }*/
                }
                else
                {
                    iRisingGap = iL;
                    iOtherGap = iR;
                }
            }
            else
            {
                if( gDebug > 2 ) cout<< " Potential rising gap at " << gapVec[iL].bearing.print() << " is not safe" << endl;
            }
        }
        
        if( iRisingGap >= 0 )
        {
            if( iBestValleyRising >= 0 )
            {
                if( gDebug > 4 )
                {
                    cout<< "      Checking valley with rising " << iRisingGap << endl;
                    cout<< "        Goal at " << distToGoal.bearing().print() << endl;
                    cout<< "        diff from rising at " << gapVec[iRisingGap].bearing.print() << " is " << fabs(gapVec[iRisingGap].bearing.alDiff(distToGoal.bearing())) << endl;
                    cout<< "        diff from best at   " << gapVec[iBestValleyRising].bearing.print() << " is " << fabs(gapVec[iBestValleyRising].bearing.alDiff(distToGoal.bearing())) << endl;
                }
                
                // Angular proximity
                if( fabs(gapVec[iRisingGap].bearing.alDiff(distToGoal.bearing())) < fabs(gapVec[iBestValleyRising].bearing.alDiff(distToGoal.bearing())) )
                {
                    if( gDebug > 3 ) cout<< "    New best valley with rising " << iRisingGap << endl;
                    iBestValleyRising = iRisingGap;
                    iBestValleyOther = iOtherGap;
                }
                
                // Physical proximity
                /*if( distToGoal.dist( Position(gapVec[iRisingGap].distance, gapVec[iRisingGap].bearing) ) <
                    distToGoal.dist( Position(gapVec[iBestValleyRising].distance, gapVec[iBestValleyRising].bearing) ) )
                {
                    iBestValleyRising = iRisingGap;
                    iBestValleyOther = iOtherGap;
                }*/
            }
            else
            {
                if( gDebug > 3 ) cout<< "    New best valley with rising " << iRisingGap << endl;
                iBestValleyRising = iRisingGap;
                iBestValleyOther = iOtherGap;
            }
        }
    }
    
    if( iBestValleyRising >= 0 )
    {
        if( gDebug > 1 ) cout<< "  Best valley has rising gap at " << gapVec[iBestValleyRising].bearing.print() << endl;
        pBestValley = new Valley( gapVec[iBestValleyRising], gapVec[iBestValleyOther] );
    }
}

void SND_algorithm::setObsAvoidDelta( double safetyDist )
{
    obsAvoidDelta = 0.0;
    double deltaMag = 0.0;
    double deltaAngle = 0.0;
    double deltaAreaSum = 0.0;
    
    
    for( uint i = 0; i < laserScan.size(); i++ )
    {
        if( laserScan[i].first <= safetyDist + robotRadius )
        {
            deltaMag = limit((safetyDist + robotRadius - laserScan[i].first)/(safetyDist),0.0,1.0);
            deltaAngle = driveAngle.alDiff(laserScan[i].second + M_PI);
            
            deltaAreaSum += deltaMag*deltaMag;
            
            obsAvoidDelta += deltaMag*deltaMag*deltaMag*deltaAngle;
        }
    }
    
    if( deltaAreaSum > 0 )
    {
        obsAvoidDelta /= deltaAreaSum;
    }
    else
    {
        obsAvoidDelta = 0.0;
    }
}


void SND_algorithm::step( )
{

	fMaxRange = robot->GetMaxRange();
	fScanRes = robot->GetScanRes();
	iNumLPs = robot->GetCount();

    
    double driveSpeed = 0.0;
    double driveTurnRate = 0.0;
    
    // update _var values
    robotPose = Pose( robot->GetXPos(), robot->GetYPos(), Angle(robot->GetYaw()) );
    if( gDebug > 2 ) cout << "pose " << robotPose.print() << endl;

    //pthread_mutex_lock(&(robot->goal_mutex));
    goal = Pose( robot->getGoalX(), robot->getGoalY(), Angle(robot->getGoalA()));
    //pthread_mutex_unlock(&(robot->goal_mutex));
    if( gDebug > 2 ) cout << "goal " << goal.print() << endl;

    gapVec.clear();
    if( pBestValley != NULL )
    {
        delete pBestValley;
    }
    pBestValley = NULL;
    driveAngle = 0.0;
    safeRisingGapAngle = 0.0;
    midValleyAngle = 0.0;
    obsAvoidDelta = 0.0;
    
	 
    iNumLPs = robot->GetCount();
    
    if( iNumLPs <= 0 || iNumLPs > 100000 )
    {
        robot->publishSpeed(0.0, 0.0);
        return;
    }
	 
    if( laserScan.size() != iNumLPs )
    {
        laserScan.resize( iNumLPs );
    }
    
    for( uint i = 0; i < iNumLPs; i++ )
    {
        laserScan[i].first = robot->range(i);
        laserScan[i].second = Angle( fScanRes * (i - iNumLPs/2.0) );
    }
    
    Pose relGoal = (goal - robotPose);
    Position distToGoal = Position(relGoal.pos().norm(), relGoal.pos().bearing() - robotPose.ori());
    
    if( gDebug > 4 )
    {
        cout<< "  Rel goal pose " << relGoal.print() << endl;
        cout<< "  Dist to goal " << distToGoal.print() << endl;
    }
    
    // Goal position fulfilled, no need to continue
    if( distToGoal.norm() <= goalPositionTol )
    {
        if( robotPose.ori().almostEqual( goal.ori(), goalAngleTol ) )
        {
            if( gDebug > 0 ) std::cout<< "Reached goal location " << goal.print() << std::endl;
            
            robot->WaitForNextGoal();

				if (! robot->hasNextGoal()) 
					robot->publishSpeed(0.0, 0.0);
				

            return;
        }
        else
        {
            driveSpeed = 0.0;
            
            if( robotPose.ori().ccwDiff(goal.ori()) < M_PI )
            {
                driveTurnRate = std::min( maxTurnRate/2, robotPose.ori().ccwDiff(goal.ori())/3 );
            }
            else
            {
                driveTurnRate = std::max( -maxTurnRate/2, robotPose.ori().cwDiff(goal.ori())/3 );
            }
            
            robot->publishSpeed(driveSpeed, driveTurnRate); //SetSpeed( driveSpeed, driveTurnRate );

            return;
        }
    }
    
    // Check closest obstacle
    double distToClosestObstacle = fMaxRange;
    for( uint i = 0; i < iNumLPs; i++ )
    {
        if( laserScan[i].first <= robotRadius )
        {
            if( gDebug > 0 ) cout<< " Obstacle inside of robot's boundary!  Stopping" << endl;
            robot->publishSpeed(0.0, 0.0); //SetSpeed( 0.0, 0.0 );
            return;
        }
        
        if( laserScan[i].first <= obstacleAvoidDist + robotRadius && laserScan[i].first < distToClosestObstacle )
        {
            distToClosestObstacle = laserScan[i].first;
        }
    }

    double safetyDist = limit( 3.0*(distToClosestObstacle - robotRadius), robotRadius/10.0, obstacleAvoidDist);
    
    // Create list of gap/discontinuity angles
    buildGapVector( );
    
    if( gDebug > 2 ) 
    {
        for( uint i = 0; i < gapVec.size(); i++ )
        {
            cout<< "  Gap at " << gapVec[i].bearing.dCastPi() << ", dir " << gapVec[i].iDir;
            cout<< ", " << gapVec[i].distance << endl;
        }
    }
    
    // Clean up gap list, combining neighboring gaps.  Keep right most right gap, left most left gap
    removeDuplicateGaps( );
    
    
    // Find best valley
    findBestValley( distToGoal );
    
    Angle safeRisingGapAngle;
    Angle midValleyAngle;
    
	 

	 // Drive scenarios
    if( isFilterClear( distToGoal.bearing(), minGapWidth, std::min(fMaxRange - robotRadius, (double)(distToGoal.norm())), false) )
    {
        if( gDebug > 0 ) cout<< "  Heading straight for goal, path is clear!" << endl;
        if( gDebug > 0 ) cout<< "   Dist to goal " << (double)distToGoal.norm() << " angle " << distToGoal.bearing().print() << endl;
        driveAngle = distToGoal.bearing().dCast();
    }
    else if( pBestValley == NULL )
    {
        if( gDebug > 0 ) cout<< "  No where to go, turning in place" << endl;
        driveAngle = M_PI/2.0;
        driveSpeed = 0;
        driveTurnRate = maxTurnRate/2.0;
        robot->publishSpeed(driveSpeed, driveTurnRate); //ed( driveSpeed, driveTurnRate );
        return;
    }
    else
    {
        // arctangent seems to work better in tight situations, arcsin sometimes wanted to send robot far from valley
        // in tight situations
        Angle safetyDeltaAngle = atan2( obstacleAvoidDist + robotRadius, std::max(robotRadius,pBestValley->risingGap.distance) );
        /*
        Angle safetyDeltaAngle = M_PI/2;
        if( pBestValley->risingGap.distance > obstacleAvoidDist + robotRadius )
        {
            safetyDeltaAngle = asin( limit((obstacleAvoidDist + robotRadius)/(pBestValley->risingGap.distance), 0.0, 1.0) );
        }
        */
        
        if( gDebug > 1 )
        {
            cout<< "    Best valley has rising at " << pBestValley->risingGap.bearing.print() << endl;
            cout<< "      safety delta = " << safetyDeltaAngle.print() << endl;
        }
        
        safeRisingGapAngle = pBestValley->risingGap.bearing - (pBestValley->risingGap.iDir * safetyDeltaAngle.dCast());
        
        if( pBestValley->risingGap.iDir > 0 )
        {
            midValleyAngle = pBestValley->risingGap.bearing.cwMean(pBestValley->otherGap.bearing);
        }
        else
        {
            midValleyAngle = pBestValley->risingGap.bearing.ccwMean(pBestValley->otherGap.bearing);
        }
        
        if( fabs(safeRisingGapAngle.alDiff(pBestValley->risingGap.bearing)) < fabs(midValleyAngle.alDiff(pBestValley->risingGap.bearing)) )
        {
            driveAngle = safeRisingGapAngle;
        }
        else
        {
            driveAngle = midValleyAngle;
        }
        
        //if( gDebug > 1 )
        //{
            //cout<< "    Best valley has rising at " << pBestValley->risingGap.bearing.print() << endl;
            //cout<< "      safety delta = " << safetyDeltaAngle.print() << endl;
            //cout<< "      safe rising  = " << safeRisingGapAngle.print() << endl;
            //cout<< "      mid valley   = " << midValleyAngle.print() << endl;
        //}
    }
        
    // Consider nearby obstacles
    setObsAvoidDelta( safetyDist );
    
    if( gDebug > 2 ) cout<< " Starting drive angle " << driveAngle.print() << "   " << driveAngle.dCastPi() << endl;
    
    // Don't allow obstacles to change sign of sharp turns
    if( driveAngle.dCastPi() > M_PI/2.0 )
    {
        driveAngle += obsAvoidDelta;
        if( driveAngle.dCastPi() < 0.0 )
        {
            driveAngle = M_PI/2.0;
        }
    }
    else if( driveAngle.dCastPi() < -M_PI/2.0 )
    {
        driveAngle += obsAvoidDelta;
        if( driveAngle.dCast() > 0.0 )
        {
            driveAngle = -M_PI/2.0;
        }
    }
    else
    {
        driveAngle += obsAvoidDelta;
    }

    double theta = driveAngle.dCast() - (driveAngle.dCast() > M_PI ? 2*M_PI : 0);
    
    if( gDebug > 2 ) 
    {
        cout<< " Drive angle : " << driveAngle.print();
        cout<< " from mid " << midValleyAngle.print() << ", safe rising " << safeRisingGapAngle.print() << ", and " << (double)obsAvoidDelta << " obs delta" << endl;
    }

    if( gDebug > 2 ) cout<< " Theta " << theta << " , " << driveAngle.dCastPi() << endl;

    theta = limit( theta, -M_PI/2.0, M_PI/2.0 );


	 driveTurnRate = maxTurnRate*(2.0*theta/M_PI);
    driveTurnRate *= limit(pow((double)distToGoal.norm(), 0.5), 0.2, 1.0);
    driveTurnRate *= limit(pow((double)(distToClosestObstacle - robotRadius)/obstacleAvoidDist, 0.5), 0.5, 1.0);

	 theta = limit( theta, -M_PI/4.0, M_PI/4.0 );

    driveSpeed = maxSpeed;
    driveSpeed *= limit(pow((double)distToGoal.norm(), 0.5), 0.0, 1.0);
    driveSpeed *= limit(pow((double)(distToClosestObstacle - robotRadius)/obstacleAvoidDist, 0.5), 0.0, 1.0);
    driveSpeed *= limit((M_PI/6.0 - fabs(theta))/(M_PI/6.0),0.0,1.0);
    
    robot->publishSpeed( driveSpeed, driveTurnRate );

	 if( gDebug > 0 ) cout<< " Drive commands: " << driveSpeed << ", " << driveTurnRate << endl;
    
    return;
    
    
}