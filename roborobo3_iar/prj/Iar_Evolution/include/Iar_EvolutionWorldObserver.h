/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#ifndef IAR_EVOLUTIONWORLDOBSERVER_H
#define IAR_EVOLUTIONWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"

#include "Observers/WorldObserver.h"

class World;

class Iar_EvolutionWorldObserver : public WorldObserver
{
	protected:
		int genIt;
		
	public:
		Iar_EvolutionWorldObserver( World *__world );
		~Iar_EvolutionWorldObserver();
				
		void reset();
		void step();
		
};

#endif

