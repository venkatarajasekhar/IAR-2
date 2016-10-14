#include "Config/ConfigurationLoader.h"
#include <string.h>

#include "Config/TemplateWanderConfigurationLoader.h"
#include "Config/TemplateBoidsConfigurationLoader.h"
#include "Config/TemplateMedeaConfigurationLoader.h"
#include "Config/TemplateRandomwalkConfigurationLoader.h"
#include "Config/Iar_BoidsConfigurationLoader.h"
#include "Config/Iar_BraitenbergConfigurationLoader.h"
#include "Config/Iar_EvolutionConfigurationLoader.h"
//###DO-NOT-DELETE-THIS-LINE###TAG:INCLUDE###//


ConfigurationLoader::ConfigurationLoader()
{
	//nothing to do
}

ConfigurationLoader::~ConfigurationLoader()
{
	//nothing to do
}

ConfigurationLoader* ConfigurationLoader::make_ConfigurationLoader (std::string configurationLoaderObjectName)
{
	if (0)
	{
		// >>> Never reached
	}
#if defined PRJ_TEMPLATEWANDER || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateWanderConfigurationLoader" )
	{
		return new TemplateWanderConfigurationLoader();
	}
#endif
#if defined PRJ_TEMPLATEBOIDS || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateBoidsConfigurationLoader" )
	{
		return new TemplateBoidsConfigurationLoader();
	}
#endif
#if defined PRJ_TEMPLATEMEDEA || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateMedeaConfigurationLoader" )
	{
		return new TemplateMedeaConfigurationLoader();
	}
#endif
#if defined PRJ_TEMPLATERANDOMWALK || !defined MODULAR
	else if (configurationLoaderObjectName == "TemplateRandomwalkConfigurationLoader" )
	{
		return new TemplateRandomwalkConfigurationLoader();
	}
#endif
#if defined PRJ_IAR_BOIDS || !defined MODULAR
	else if (configurationLoaderObjectName == "Iar_BoidsConfigurationLoader" )
	{
		return new Iar_BoidsConfigurationLoader();
	}
#endif
#if defined PRJ_IAR_BRAITENBERG || !defined MODULAR
	else if (configurationLoaderObjectName == "Iar_BraitenbergConfigurationLoader" )
	{
		return new Iar_BraitenbergConfigurationLoader();
	}
#endif
#if defined PRJ_IAR_EVOLUTION || !defined MODULAR
	else if (configurationLoaderObjectName == "Iar_EvolutionConfigurationLoader" )
	{
		return new Iar_EvolutionConfigurationLoader();
	}
#endif
    //###DO-NOT-DELETE-THIS-LINE###TAG:SWITCH###//
	else
	{
		return NULL;
	}

}
