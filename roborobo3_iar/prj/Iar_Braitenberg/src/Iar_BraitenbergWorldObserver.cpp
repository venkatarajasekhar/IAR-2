/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include "Iar_Braitenberg/include/Iar_BraitenbergWorldObserver.h"

#include "World/World.h"


Iar_BraitenbergWorldObserver::Iar_BraitenbergWorldObserver( World *__world ) : WorldObserver( __world )
{
	_world = __world;
}

Iar_BraitenbergWorldObserver::~Iar_BraitenbergWorldObserver()
{
	// nothing to do.
}

void Iar_BraitenbergWorldObserver::reset()
{
	// nothing to do.
}

void Iar_BraitenbergWorldObserver::step()
{
	// nothing to do.
}
