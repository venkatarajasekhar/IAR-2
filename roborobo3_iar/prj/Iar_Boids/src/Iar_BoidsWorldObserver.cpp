/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include "Iar_Boids/include/Iar_BoidsWorldObserver.h"

#include "World/World.h"

#include "Iar_Boids/include/Iar_BoidsController.h"

Iar_BoidsWorldObserver::Iar_BoidsWorldObserver( World *__world ) : WorldObserver( __world )
{
	_world = __world;
}

Iar_BoidsWorldObserver::~Iar_BoidsWorldObserver()
{
	// nothing to do.
}

void Iar_BoidsWorldObserver::reset()
{
	// nothing to do.
}

void Iar_BoidsWorldObserver::step()
{
    if ( gWorld->getIterations() == 0 ) // Premiere itération, positionnement des boids.
    {
        for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
        {
            Robot *robot = (gWorld->getRobot(i));
            (*robot).setCoordReal( 100 + (i*50)%800 , 100 + ( i*50/800 * 50 )  );
        }
    }
    
}
