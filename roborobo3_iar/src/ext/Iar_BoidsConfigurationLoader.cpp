#if defined PRJ_IAR_BOIDS || !defined MODULAR

#include "Config/Iar_BoidsConfigurationLoader.h"

#include "Iar_Boids/include/Iar_BoidsWorldObserver.h"
#include "Iar_Boids/include/Iar_BoidsAgentObserver.h"
#include "Iar_Boids/include/Iar_BoidsController.h"

#include "WorldModels/RobotWorldModel.h"


Iar_BoidsConfigurationLoader::Iar_BoidsConfigurationLoader()
{
	// create the single instance of Agent-World Interface.
}

Iar_BoidsConfigurationLoader::~Iar_BoidsConfigurationLoader()
{
	//nothing to do
}

WorldObserver* Iar_BoidsConfigurationLoader::make_WorldObserver(World* wm)
{
	return new Iar_BoidsWorldObserver(wm);
}

RobotWorldModel* Iar_BoidsConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* Iar_BoidsConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new Iar_BoidsAgentObserver(wm);
}

Controller* Iar_BoidsConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new Iar_BoidsController(wm);
}


#endif
