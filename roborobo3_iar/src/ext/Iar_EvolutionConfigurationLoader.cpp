#if defined PRJ_IAR_EVOLUTION || !defined MODULAR

#include "Config/Iar_EvolutionConfigurationLoader.h"

#include "Iar_Evolution/include/Iar_EvolutionWorldObserver.h"
#include "Iar_Evolution/include/Iar_EvolutionAgentObserver.h"
#include "Iar_Evolution/include/Iar_EvolutionController.h"

#include "WorldModels/RobotWorldModel.h"


Iar_EvolutionConfigurationLoader::Iar_EvolutionConfigurationLoader()
{
	// create the single instance of Agent-World Interface.
}

Iar_EvolutionConfigurationLoader::~Iar_EvolutionConfigurationLoader()
{
	//nothing to do
}

WorldObserver* Iar_EvolutionConfigurationLoader::make_WorldObserver(World* wm)
{
	return new Iar_EvolutionWorldObserver(wm);
}

RobotWorldModel* Iar_EvolutionConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* Iar_EvolutionConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new Iar_EvolutionAgentObserver(wm);
}

Controller* Iar_EvolutionConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new Iar_EvolutionController(wm);
}


#endif
