/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 */


#include "Iar_Boids/include/Iar_BoidsController.h"

// Sensor Id
// (L)eft, (R)ight, (B)ack, (F)ront
// FFFL is a front sensor, pointing very slightly to the left (15°).
#define SENSOR_L 0
#define SENSOR_FL 1
#define SENSOR_FFL 2
#define SENSOR_FFFL 3
#define SENSOR_F 4
#define SENSOR_FFFR 5
#define SENSOR_FFR 6
#define SENSOR_FR 7
#define SENSOR_R 8
#define SENSOR_BR 9
#define SENSOR_B 10
#define SENSOR_BL 11

#define attractionRadius gSensorRange
#define repulsionRadius gSensorRange*0.8


Iar_BoidsController::Iar_BoidsController( RobotWorldModel *__wm ) : Controller ( __wm )
{
    if ( _wm->_cameraSensorsNb != 12 )
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with 12 sensors (current specs: " << _wm->_cameraSensorsNb << "). STOP.\n";
        exit(-1);
    }
}

Iar_BoidsController::~Iar_BoidsController()
{
	// nothing to do.
}

void Iar_BoidsController::reset()
{
	// nothing to do.
}

void Iar_BoidsController::step()
{
    //_wm->_desiredTranslationalValue = 0.2; // dans [-2,+2]
    //_wm->_desiredRotationalVelocity = 1.0; // dans [-30,+30]

    attractionBehavior();
    repulsionBehavior();
    orientationBehavior();
    avoidObstacle();
    monitorSensoryInformation();
}

void Iar_BoidsController::orientationBehavior()
{
    int nbNeighbors = 0;
    double averageOrientation = 0;
    double tempOrientation;

    for (int i = 0; i < _wm->_cameraSensorsNb; i++)
    {
        int rawIndex = _wm->getObjectIdFromCameraSensor(i);
        if (rawIndex >= gRobotIndexStartOffset)
        {
            int robotIndex = rawIndex - gRobotIndexStartOffset;
            tempOrientation = gWorld->getRobot(robotIndex)->getWorldModel()->_agentAbsoluteOrientation;

            tempOrientation = _wm->_agentAbsoluteOrientation - tempOrientation;

            if (tempOrientation < -180)
                tempOrientation = 360 + tempOrientation;
            if (tempOrientation > 180)
                tempOrientation = -360 + tempOrientation;

            averageOrientation += tempOrientation;
            nbNeighbors++;
        }
    }

    averageOrientation /= nbNeighbors;
    if (averageOrientation < 0)
        _wm->_desiredRotationalVelocity = 10;
    if (averageOrientation > 0)
        _wm->_desiredRotationalVelocity = -10;
    if (averageOrientation == 0)
        _wm->_desiredRotationalVelocity = 0;
}

void Iar_BoidsController::attractionBehavior()
{
    int nbNeighbors = 0;
    double centreDeMasseX = 0;
    double centreDeMasseY = 0;
    double distance;

    for (int i = 0; i < _wm->_cameraSensorsNb; i++)
    {
        int rawIndex = _wm->getObjectIdFromCameraSensor(i);
        if (rawIndex >= gRobotIndexStartOffset)
        {
            distance = _wm->getDistanceValueFromCameraSensor(i);
            if (distance < attractionRadius)
            {
                int robotIndex = rawIndex - gRobotIndexStartOffset;
                centreDeMasseX += gWorld->getRobot(robotIndex)->getWorldModel()->_xReal;
                centreDeMasseY += gWorld->getRobot(robotIndex)->getWorldModel()->_yReal;
                nbNeighbors++;
            }

        }
    }

    centreDeMasseX /= nbNeighbors;
    centreDeMasseY /= nbNeighbors;

    double angleRotation = getAngleToTarget(_wm->_xReal, _wm->_yReal, _wm->_agentAbsoluteOrientation, centreDeMasseX, centreDeMasseY);
    if (angleRotation < 0)
        _wm->_desiredRotationalVelocity = -10;
    if (angleRotation > 0)
        _wm->_desiredRotationalVelocity = 10;


}

void Iar_BoidsController::repulsionBehavior()
{
    double distance;
    double minDistance = gSensorRange + 1;
    int closestRobot = -1;

    for (int i = 0; i < _wm->_cameraSensorsNb; i++)
    {
        int rawIndex = _wm->getObjectIdFromCameraSensor(i);
        if (rawIndex >= gRobotIndexStartOffset)
        {
            distance = _wm->getDistanceValueFromCameraSensor(i);
            if (distance < repulsionRadius)
            {
                if (distance < minDistance)
                {
                    closestRobot = rawIndex - gRobotIndexStartOffset;
                    minDistance = distance;
                }
            }

        }
    }

    if (closestRobot != -1)
    {
        double posX = gWorld->getRobot(closestRobot)->getWorldModel()->_xReal;
        double posY = gWorld->getRobot(closestRobot)->getWorldModel()->_yReal;
        double angleRotation = getAngleToTarget(_wm->_xReal, _wm->_yReal, _wm->_agentAbsoluteOrientation, posX, posY);
        if (angleRotation < 0)
            _wm->_desiredRotationalVelocity = 10;
        if (angleRotation > 0)
            _wm->_desiredRotationalVelocity = -10;
    }
}

void Iar_BoidsController::avoidObstacle()
{
    bool detect = false;
    for (int i  = 0; i < _wm->_cameraSensorsNb; i++)
        {
            double distance = _wm->getDistanceValueFromCameraSensor(i);

            // Si on repère un obstacle (pas un voisin), on essaie de l'éviter
            if (distance < gSensorRange && _wm->getObjectIdFromCameraSensor(i) < gRobotIndexStartOffset)
            {
                detect = true;
                if (i <= 8) // Si il est devant
                {
                    _wm->_desiredTranslationalValue = 5;
                    _wm->_desiredRotationalVelocity = 1.0;
                }
            }
        }

    // Si on n'a détecté aucun obstacle, on fonce à toute allure devant
    if (!detect) 
    {
        _wm->_desiredTranslationalValue = 10;
        //_wm->_desiredRotationalVelocity = 0;
    }
}

void Iar_BoidsController::monitorSensoryInformation()
{
    // Note that this code is executed only for the agent which is "on focus". By default, this means agent #0.
    // When window mode:
    //      To show which agent has the focus, press 'F'
    //      To cycle through agents, press <tab> (or shift+<tab>)
        
    if ( gVerbose && gDisplayMode == 0 && gRobotIndexFocus == _wm->getId() )
    {
        
        std::cout << "=-= Robot #" << _wm->getId() << " : STARTING monitoring sensory information at iteration #" << gWorld->getIterations() << ".\n";
        
        // Rotational and translational speed, agent orientation wrt. upwards
        //      - *actual* and *desired* translational/rotational values are very different
        //          - actual values is what the robot is actually doing (this is measured)
        //          - desired values are set by the controller (this is set and the hardware controller tries to match it)
        //          - rational: you may ask for something (e.g. max speed) but physics and electronics may be limited
        //          - typical example: when going for max speed, the robot cannot instantaneously go at max speed.
        //      - agent orientation acts as a compass with respect the y-axis, similar to a magnetic compass where north is upward
        
        double srcOrientation = _wm->_agentAbsoluteOrientation;
        
        std::cout << "Agent orientation: " << std::setw(4) << srcOrientation << "° wrt North (ie. upwards).\n";
        
        std::cout << "Agent desired translational speed: " << _wm->_desiredTranslationalValue << std::endl;
        std::cout << "Agent desired rotational speed: " << std::setw(4) << _wm->_desiredRotationalVelocity << std::endl;
        
        std::cout << "Agent actual translational speed: " << _wm->_actualTranslationalValue << std::endl;
        std::cout << "Agent actual rotational speed: " << std::setw(4) << _wm->_actualRotationalVelocity << std::endl;
        
        // Camera/distance sensors -- provide: distance to obstacle, obstacle type, orientation (if robot)
        // REMARKS:
        //      - distance sensors are actually camera rays, and provides extended information:
        //          - distance to contact
        //          - type of contact (walls, objects, or robots)
        //          - if robot: group number, LED values, absolute orientation (from which one can compute relative orientation)
        //      - Objects and walls are different.
        //          - Walls are fixed, and loaded from gEnvironmentImage (see gEnvironmentImageFilename file)
        //          - There are several types of objects. Check children of object PhysicalObject.
        //          - Note that PhysicalObjects can have a tangible part (see gEnvironmentImage) and an intangible part (see gGroundSensorImage. The intangible part can be spotted with the floorSensor.
        //          - Some PhysicalObject are active and react to the robot actions (e.g. disappear, give energy, ...)
        //          - examples of use of a PhysicalObject:
        //              - a tangible object onto which the robot can crash. It is seen through distance sensors.
        //              - an intangible object onto which the robot can walk upon. It is seen through the floor sensor.
        //              - a mix of the two.
        
        for(int i  = 0; i < _wm->_cameraSensorsNb; i++)
        {
            double distance = _wm->getDistanceValueFromCameraSensor(i);
            // double distanceNormalized = _wm->getDistanceValueFromCameraSensor(i) / _wm->getCameraSensorMaximumDistanceValue(i); // Similar to _wm->getNormalizedDistanceValueFromCameraSensor(i); -- unused here

            int objectId = _wm->getObjectIdFromCameraSensor(i);
            
            std::cout << "Sensor #" << i << ":";
            
            if ( PhysicalObject::isInstanceOf(objectId) ) // sensor touched an object. What type? (could be GateObject, SwitchObject, RoundObject, ... -- check descendants of PhysicalObject class)
            {
                int nbOfTypes = PhysicalObjectFactory::getNbOfTypes();
                for ( int i = 0 ; i != nbOfTypes ; i++ )
                {
                    if ( i == gPhysicalObjects[objectId - gPhysicalObjectIndexStartOffset]->getType() )
                    {
                        std::cout << "object of type " << i << " detected\n";
                        break;
                    }
                }
            }
            else
            {
                if ( Agent::isInstanceOf(objectId) )
                {
                    int targetRobotId = objectId-gRobotIndexStartOffset;

                    std::cout << " touched robot #" << gWorld->getRobot(targetRobotId) << ", at distance " << std::setw(4) << distance << ".\n";
                    
                    // Distance to target , orientation wrt target, target absolute orientation, target LED values
                    // Distance to target is approximated through sensor ray length before contact.
                    
                    double tgtOrientation = gWorld->getRobot(targetRobotId)->getWorldModel()->_agentAbsoluteOrientation;
                    double delta_orientation = - ( srcOrientation - tgtOrientation );
                    if ( delta_orientation >= 180.0 )
                        delta_orientation = - ( 360.0 - delta_orientation );
                    else
                        if ( delta_orientation <= -180.0 )
                            delta_orientation = - ( - 360.0 - delta_orientation );
                    
                    std::cout << "\trelative orientation wrt target robot is " <<std::setw(4) << delta_orientation/180.0 << "\n";
                    std::cout << "\tabsolute orientation of target robot is  " <<std::setw(4) << tgtOrientation << "\n";
                    
                    // same group? -- unusued as of Oct. 2015
                    
                    if ( gWorld->getRobot(targetRobotId)->getWorldModel()->getGroupId() == _wm->getGroupId() )
                        std::cout << "\trobots are from the same group.\n";
                    else
                        std::cout << "\trobots are from different group.\n";
                    
                    // LED values of other robot (can be used for communication)
                    
                    double tgt_LED_redValue = (double)_wm->getRobotLED_redValue()/255.0;
                    double tgt_LED_greenValue = (double)_wm->getRobotLED_greenValue()/255.0;
                    double tgt_LED_blueValue = (double)_wm->getRobotLED_blueValue()/255.0;
                    
                    std::cout << "\tLED values: R=" << tgt_LED_redValue << ", G=" << tgt_LED_greenValue << ", B=" << tgt_LED_blueValue << "\n";
                    
                }
                else
                {
                    // input: wall or empty?
                    if ( objectId >= 0 && objectId < gPhysicalObjectIndexStartOffset ) // not empty, but cannot be identified: this is a wall.
                    {
                        std::cout << " touched an unindentified obstacle (probably a wall, id=" << objectId << "), at distance " << std::setw(4) << distance << ".\n";
                    }
                    else
                        std::cout << " nothing (id="<< objectId << "). Returns maximum value (" << std::setw(4) << distance << ")\n";
                }
            }
        }
        std::cout << "=-= Robot #" << _wm->getId() << " : STOPPING monitoring sensory information\n";
    }
    
}
