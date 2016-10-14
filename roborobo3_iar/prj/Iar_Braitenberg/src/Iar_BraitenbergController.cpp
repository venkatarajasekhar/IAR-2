/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include "Iar_Braitenberg/include/Iar_BraitenbergController.h"

Iar_BraitenbergController::Iar_BraitenbergController( RobotWorldModel *__wm ) : Controller ( __wm )
{
	// nothing to do
}

Iar_BraitenbergController::~Iar_BraitenbergController()
{
	// nothing to do.
}

void Iar_BraitenbergController::reset()
{
	// nothing to do.
}


void Iar_BraitenbergController::step()
{
    //_wm->_desiredTranslationalValue = 0.2; // dans [-2,+2]
    //_wm->_desiredRotationalVelocity = 1.0; // dans [-30,+30]

    /* Cas difficiles:
        - objets à gauche et à droite, on devrait reculer ou tourner tant que devant il y a un obstacle
    */
    bool detect = false;
    for (int i  = 0; i < _wm->_cameraSensorsNb; i++)
        {
            double distance = _wm->getDistanceValueFromCameraSensor(i);

            // Si on repère un obstacle, on essaie de l'éviter
            if (distance < gSensorRange)
            {
                detect = true;
                if (i <= 8) // Si il est devant
                {
                    _wm->_desiredTranslationalValue = 1;
                    _wm->_desiredRotationalVelocity = 2.0;
                }
            }
        }

    // Si on n'a détecté aucun obstacle, on fonce à toute allure devant
    if (!detect) 
    {
        _wm->_desiredTranslationalValue = 10;
        _wm->_desiredRotationalVelocity = 0;
    }

    monitorSensoryInformation();
}

void Iar_BraitenbergController::monitorSensoryInformation()
{
    if ( gVerbose && gDisplayMode == 0 )
    {      
        std::cout << "=-= Robot #" << _wm->getId() << " : STARTING monitoring sensory information at iteration #" << gWorld->getIterations() << ".\n";
        
        std::cout << "Agent actual translational speed: " << _wm->_actualTranslationalValue << std::endl;
        std::cout << "Agent actual rotational speed: " << std::setw(4) << _wm->_actualRotationalVelocity << std::endl;

        for(int i  = 0; i < _wm->_cameraSensorsNb; i++)
        {
            double distance = _wm->getDistanceValueFromCameraSensor(i);
            std::cout << "Sensor #" << i << " returns distance " << distance << "/" << gSensorRange << "\n";

        }
        std::cout << "=-= Robot #" << _wm->getId() << " : STOPPING monitoring sensory information\n";
    }
    
}
