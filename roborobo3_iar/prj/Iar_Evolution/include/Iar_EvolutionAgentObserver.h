/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */
 
#ifndef IAR_EVOLUTIONAGENTOBSERVER_H
#define IAR_EVOLUTIONAGENTOBSERVER_H 

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "WorldModels/RobotWorldModel.h"
#include "Observers/AgentObserver.h"

class Iar_EvolutionAgentObserver : public AgentObserver
{
	public:
		Iar_EvolutionAgentObserver( );
		Iar_EvolutionAgentObserver( RobotWorldModel *__wm );
		~Iar_EvolutionAgentObserver();
				
		void reset();
		void step();
		
};


#endif

