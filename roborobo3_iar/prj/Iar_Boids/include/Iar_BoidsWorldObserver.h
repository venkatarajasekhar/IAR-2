/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#ifndef IAR_BOIDSWORLDOBSERVER_H
#define IAR_BOIDSWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"

#include "Observers/WorldObserver.h"

class World;

class Iar_BoidsWorldObserver : public WorldObserver
{
	protected:
		
	public:
		Iar_BoidsWorldObserver( World *__world );
		~Iar_BoidsWorldObserver();
				
		void reset();
		void step();
		
};

#endif

