/*
 * Iar_BoidsConfigurationLoader.h
 */

#ifndef IAR_BOIDSCONFIGURATIONLOADER_H
#define	IAR_BOIDSCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class Iar_BoidsConfigurationLoader : public ConfigurationLoader
{
	public:
		Iar_BoidsConfigurationLoader();
		~Iar_BoidsConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};

#endif
