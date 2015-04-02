/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 #include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
  
        static	float rot = 0.1f;			// rads/sec
        static float adv = 100.f;			// mm/sec
        static float turnSwitch = 1;
        const float advIncLow = 0.8;		// mm/sec
        const float advIncHigh = 2.f;		// mm/sec
        const float rotInc = 0.25;			// rads/sec
        const float rotMax = 0.4;			// rads/sec
        const float advMax = 200;			// milimetres/sec
        const float distThreshold = 500; 	// milimetres
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
        if( ldata.front().dist < distThreshold) 
        {
            adv = adv * advIncLow; 
            rot = rot + turnSwitch * rotInc;
            if( rot < -rotMax) rot = -rotMax;
            if( rot > rotMax) rot = rotMax;
            differentialrobot_proxy->setSpeedBase(adv, rot);
        }
        else
        {
            adv = adv * advIncHigh; 
            if( adv > advMax) adv = advMax;
            rot = 0.f;
            differentialrobot_proxy->setSpeedBase(adv, 0.f);		
            turnSwitch = -turnSwitch;
        }	
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};