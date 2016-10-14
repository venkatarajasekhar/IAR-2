/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include "Iar_Evolution/include/Iar_EvolutionWorldObserver.h"

#include "World/World.h"

#include "Iar_Evolution/include/Iar_EvolutionController.h"

Iar_EvolutionWorldObserver::Iar_EvolutionWorldObserver( World *__world ) : WorldObserver( __world )
{
	_world = __world;
	genIt = 0;
}

Iar_EvolutionWorldObserver::~Iar_EvolutionWorldObserver()
{
	// nothing to do.
}

void Iar_EvolutionWorldObserver::reset()
{
	// nothing to do.
}

void Iar_EvolutionWorldObserver::step()
{
    
    // The following code shows an example where every N iterations, robots are re-located to their initial positions, and parameters are randomly changed.
    //
    // REMOVE OR COMMENT THE FOLLOWING TO AVOID RESETTING POSITIONS EVERY 100 ITERATIONS
    //
    
    if ( gWorld->getIterations() % 1000 == 0 )
    {
	std::cout << "Generation " << genIt << std::endl;
	genIt++;

        if ( gVerbose && gDisplayMode == 0 )
            std::cout << "Randomizing parameters\n";

        for ( int i = 0 ; i != gWorld->getNbOfRobots() ; i++ )
        {
            Robot *robot = (gWorld->getRobot(i));
            
            (*robot).setCoordReal( 100 + (i*50)%800 , 100 + ( i*50/800 * 50 )  );
            
            Iar_EvolutionController *controller = ((Iar_EvolutionController*)(gWorld->getRobot(i)->getController()));
            
            for ( size_t j = 0 ; j != (*controller)._params.size() ; j++ )
            {
                (*controller)._params[(int)j] = ranf()*2.0-1.0;
            }
        }
    }
    
}
