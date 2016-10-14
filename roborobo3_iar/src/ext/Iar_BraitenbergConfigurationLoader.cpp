#if defined PRJ_IAR_BRAITENBERG || !defined MODULAR

#include "Config/Iar_BraitenbergConfigurationLoader.h"

#include "Iar_Braitenberg/include/Iar_BraitenbergWorldObserver.h"
#include "Iar_Braitenberg/include/Iar_BraitenbergAgentObserver.h"
#include "Iar_Braitenberg/include/Iar_BraitenbergController.h"

#include "WorldModels/RobotWorldModel.h"


Iar_BraitenbergConfigurationLoader::Iar_BraitenbergConfigurationLoader()
{
	// create the single instance of Agent-World Interface.
}

Iar_BraitenbergConfigurationLoader::~Iar_BraitenbergConfigurationLoader()
{
	//nothing to do
}

WorldObserver* Iar_BraitenbergConfigurationLoader::make_WorldObserver(World* wm)
{
	return new Iar_BraitenbergWorldObserver(wm);
}

RobotWorldModel* Iar_BraitenbergConfigurationLoader::make_RobotWorldModel()
{
	return new RobotWorldModel();
}

AgentObserver* Iar_BraitenbergConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new Iar_BraitenbergAgentObserver(wm);
}

Controller* Iar_BraitenbergConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new Iar_BraitenbergController(wm);
}


#endif
