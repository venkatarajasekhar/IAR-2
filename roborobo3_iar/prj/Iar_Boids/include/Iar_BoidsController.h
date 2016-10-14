/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */



#ifndef IAR_BOIDSCONTROLLER_H
#define IAR_BOIDSCONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"

#include "Controllers/Controller.h"

#include "WorldModels/RobotWorldModel.h"

#include "Agents/Agent.h"
#include "Agents/Robot.h"

class Iar_BoidsController : public Controller
{
	public:
		//Initializes the variables
		Iar_BoidsController( RobotWorldModel *__wm );
		~Iar_BoidsController();
    
		void reset();
		void step();
		void orientationBehavior();
		void avoidObstacle();
		void attractionBehavior();
		void repulsionBehavior();
    
        void monitorSensoryInformation();
};


#endif

