/*
 * Iar_EvolutionConfigurationLoader.h
 */

#ifndef IAR_EVOLUTIONCONFIGURATIONLOADER_H
#define	IAR_EVOLUTIONCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class Iar_EvolutionConfigurationLoader : public ConfigurationLoader
{
	public:
		Iar_EvolutionConfigurationLoader();
		~Iar_EvolutionConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};

#endif
