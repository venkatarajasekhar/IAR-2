/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */



#ifndef IAR_BRAITENBERGCONTROLLER_H
#define IAR_BRAITENBERGCONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"

#include "Controllers/Controller.h"

#include "WorldModels/RobotWorldModel.h"

#include "Agents/Agent.h"
#include "Agents/Robot.h"


class Iar_BraitenbergController : public Controller
{
	public:
		//Initializes the variables
		Iar_BraitenbergController( RobotWorldModel *__wm );
		~Iar_BraitenbergController();
		
		void reset();
		void step();
    
        	void monitorSensoryInformation();
};


#endif

