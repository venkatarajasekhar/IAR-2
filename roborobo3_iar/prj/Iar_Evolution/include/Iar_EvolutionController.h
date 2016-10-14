/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */



#ifndef IAR_EVOLUTIONCONTROLLER_H
#define IAR_EVOLUTIONCONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"

#include "Controllers/Controller.h"

#include "WorldModels/RobotWorldModel.h"

#include "Agents/Agent.h"
#include "Agents/Robot.h"

class Iar_EvolutionController : public Controller
{
	public:
		//Initializes the variables
		Iar_EvolutionController( RobotWorldModel *__wm );
		~Iar_EvolutionController();
    
        std::vector<double> _params;
		
		void reset();
		void step();
    
        void monitorSensoryInformation();
};


#endif

