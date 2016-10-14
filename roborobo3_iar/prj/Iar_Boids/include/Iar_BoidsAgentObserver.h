/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */
 
#ifndef IAR_BOIDSAGENTOBSERVER_H
#define IAR_BOIDSAGENTOBSERVER_H 

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "WorldModels/RobotWorldModel.h"
#include "Observers/AgentObserver.h"

class Iar_BoidsAgentObserver : public AgentObserver
{
	public:
		Iar_BoidsAgentObserver( );
		Iar_BoidsAgentObserver( RobotWorldModel *__wm );
		~Iar_BoidsAgentObserver();
				
		void reset();
		void step();
		
};


#endif

