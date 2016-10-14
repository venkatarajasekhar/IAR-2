/*
 * Iar_BraitenbergConfigurationLoader.h
 */

#ifndef IAR_BRAITENBERGCONFIGURATIONLOADER_H
#define	IAR_BRAITENBERGCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class Iar_BraitenbergConfigurationLoader : public ConfigurationLoader
{
	public:
		Iar_BraitenbergConfigurationLoader();
		~Iar_BraitenbergConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};

#endif
