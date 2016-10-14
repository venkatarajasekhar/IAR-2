/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */

#include "Iar_Boids/include/Iar_BoidsAgentObserver.h"



Iar_BoidsAgentObserver::Iar_BoidsAgentObserver( )
{
}

Iar_BoidsAgentObserver::Iar_BoidsAgentObserver( RobotWorldModel *__wm )
{
	_wm = __wm;
}

Iar_BoidsAgentObserver::~Iar_BoidsAgentObserver()
{
	// nothing to do.
}

void Iar_BoidsAgentObserver::reset()
{
	// nothing to do.
}

void Iar_BoidsAgentObserver::step()
{
    // * send callback messages to objects touched or walked upon.
    
    // through distance sensors
    for( int i = 0 ; i < _wm->_cameraSensorsNb; i++)
    {
        int targetIndex = _wm->getObjectIdFromCameraSensor(i);
        
        if ( targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gRobotIndexStartOffset )   // sensor ray bumped into a physical object
        {
            targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
            //std::cout << "[DEBUG] Robot #" << _wm->getId() << " touched " << targetIndex << "\n";
            gPhysicalObjects[targetIndex]->isTouched(_wm->getId());
        }
    }
    
    // through floor sensor
    int targetIndex = _wm->getGroundSensorValue();
    if ( targetIndex >= gPhysicalObjectIndexStartOffset && targetIndex < gPhysicalObjectIndexStartOffset + (int)gPhysicalObjects.size() )   // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        //std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
        gPhysicalObjects[targetIndex]->isWalked(_wm->getId());
    }
}

