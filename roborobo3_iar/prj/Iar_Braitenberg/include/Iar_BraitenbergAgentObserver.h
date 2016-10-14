/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */
 
#ifndef IAR_BRAITENBERGAGENTOBSERVER_H
#define IAR_BRAITENBERGAGENTOBSERVER_H 

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "WorldModels/RobotWorldModel.h"
#include "Observers/AgentObserver.h"

class Iar_BraitenbergAgentObserver : public AgentObserver
{
	public:
		Iar_BraitenbergAgentObserver( );
		Iar_BraitenbergAgentObserver( RobotWorldModel *__wm );
		~Iar_BraitenbergAgentObserver();
				
		void reset();
		void step();
		
};


#endif

