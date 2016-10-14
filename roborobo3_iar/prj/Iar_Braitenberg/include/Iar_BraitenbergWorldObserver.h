/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#ifndef IAR_BRAITENBERGWORLDOBSERVER_H
#define IAR_BRAITENBERGWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"

#include "Observers/WorldObserver.h"

class World;

class Iar_BraitenbergWorldObserver : public WorldObserver
{
	protected:
		
	public:
		Iar_BraitenbergWorldObserver( World *__world );
		~Iar_BraitenbergWorldObserver();
				
		void reset();
		void step();
		
};

#endif

