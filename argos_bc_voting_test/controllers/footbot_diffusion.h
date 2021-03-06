/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
///* Definition of the differential steering actuator */
//#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
///* Definition of the foot-bot proximity sensor */
//#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ground_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <vector>
#include <set>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>

#include "geth_wrapper.h" /* Use geth from C++ */

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using namespace std;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CBlockchainVotingController : public CCI_Controller {
public:
	  // Random walk
	  struct Movement {
	    SInt32 walkTime;         // Movement time counter;
	    UInt32 actualDirection;  // 0, straight; 1, turn CW; 2, turn CCW (TOCHECK: if 1 is counterclockwise or vice versa; fix comment)
	    Movement();
	  };

	  struct SimulationState {
	    //UInt32 decision_rule;
	    //Real percentRed, percentBlue;
	    //Real g;
	    //Real sigma;
	    bool exitFlag;
	    bool profiling;
	    bool useMultipleNodes;
	    bool useBackgroundGethCalls;
	    //std::string radix;
	    std::string baseDir; /* Basedir of the controller folder */
	    std::string baseDirRaw; /* Basedir of the controller folder */
	    std::string interfacePath;
	    std::string mappingPath;
	    std::string regenerateFile;
	    //	    std::string mappingByzantinePath;
	    std::string blockchainPath;
	    std::string datadirBase;
	    int basePort;
	    //	    int numByzantine;
	    //UInt32 numPackSaved;
	    UInt32 status;
	    UInt32 LAMDA, turn;
	    //bool useClassicalApproach;
	    UInt32 numRobots; /* total amount of robots in the experiment */
	    void Init(TConfigurationNode& t_node);
	  };
	  
public:
   /* Class constructor. */
   CBlockchainVotingController();

   /* Class destructor. */
   virtual ~CBlockchainVotingController() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   virtual void RandomWalk();
   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   void fromLoopFunctionResPrepare();
   void fromLoopFunctionResStart();
   //void killGethAndRemoveFolders(std::string bcPath, std::string regenFile);
   void Explore();
   void Move();
   void TurnLeds();
   
   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy();

   inline Movement & GetMovement() {
      return movement;
   }
   
   inline std::string & GetAddress() {
      return address;
   }

   inline std::string & GetMinerAddress() {
      return minerAddress;
   }
   
   inline bool isMining() {
     return mining;
   }
   
   inline std::string getEnode()  {
     return enode;
   }

   inline int getNodeInt() {
     return nodeInt;
   }

   inline void setContractAddress(std::string contractAddr) {
     contractAddress = contractAddr;
   }
   
   inline CColor& GetColor() {
      return m_cColor;
   }

   inline long long GetStepCnt() {
      return m_lStepCnt;
   }
   
   //inline void setColor(CColor color) {
   //  m_cColor = color;
   //}
   void setColor(CColor color);

   inline void setSquareRadius(Real r) {
     m_fFoodSquareRadius = r;
   }

   inline void setFoodPos(std::vector<CVector2> cFoodPos) {
     m_cFoodPos = cFoodPos;
   }

   inline void setCurrentPos(CVector2 cPos) {
     m_cPos = cPos;
   }
   
   void UpdateNeighbors(std::set<int> newNeighbors);
   void registerRobot(); // Tell the smart contract the robot's public key
   void updateRegistration(); // Wait for the first event of the smart contract 
   
private:
   void InitGeth(int robotId);
   void readNodeMapping();
   //   void readByzantineMapping();
   void DistributeID();
   
    
   /*
    * Updates the state information.
    * In pratice, it sets the SStateData::InNest flag.
    * Future, more complex implementations should add their
    * state update code here.
    */
   void UpdateState();
 
   /*
    * Calculates the vector to the light. Used to perform
    * phototaxis and antiphototaxis.
    */
   CVector2 CalculateVectorToLight();
   
   Real m_fFoodSquareRadius;
   std::vector<CVector2> m_cFoodPos;
   CVector2 m_cPos;
   CColor m_cColor;
   
   Real m_fWheelVelocity;
   CCI_EPuckWheelsActuator* m_pcWheels;
   CCI_LEDsActuator* m_pcLEDs;
   CCI_EPuckRangeAndBearingActuator*  m_pcRABA;
   
   CCI_EPuckRangeAndBearingSensor* m_pcRABS;
   CDegrees m_cAlpha;                         // OBST. AVOID.
   Real m_fDelta;                             // OBST. AVOID.
   CCI_EPuckProximitySensor* m_pcProximity;   // OBST. AVOID.
   ///* Pointer to the foot-bot light sensor */
   //CCI_FootBotLightSensor* m_pcLight;
   ///* Pointer to the foot-bot motor ground sensor */
   //CCI_EPuckGroundSensor* m_pcGround;
   CRange<CRadians> m_cGoStraightAngleRange;  // OBST. AVOID.
      
   CRandom::CRNG* m_pcRNG;
   
   /* Files */
   std::ofstream epuckFile;
   /* All others used variables */
   //SStateData m_sStateData;
   SimulationState simulationParams;
   
   Movement movement;
   std::string address;
   std::string minerAddress;
   std::string contractAddress;
   std::string rawTx;
   std::set<int> neighbors;
   std::string enode;

   std::string blockchainPath;
   blockWithHash bwh;
   bool beginning;
   int nodeInt;
   std::map<int, int> robotIdToNode;  
   bool mining;
   int byzantineStyle;
   bool threadCurrentlyRunning;
   int eventTrials;

   long long m_lStepCnt;
};

#endif
