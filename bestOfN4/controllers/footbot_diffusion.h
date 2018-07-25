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

#include <argos3/core/utility/configuration/argos_configuration.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

//#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_actuator.h>
//#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_sensor.h>
//#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
//#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ground_sensor.h>
//#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>

/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

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
    /*
    * This structure holds data about food collecting by the robots
    */
    struct SFoodData {
      bool HasFoodItem;      // true when the robot is carrying a food item
      size_t FoodItemIdx;    // the index of the current food item in the array of available food items
      size_t TotalFoodItems; // the total number of food items carried by this robot during the experiment
 
      SFoodData();
      void Reset();
    };
    
    /*
    * The following variables are used as parameters for the
    * diffusion algorithm. You can set their value in the <parameters>
    * section of the XML configuration file, under the
    * <controllers><footbot_foraging_controller><parameters><diffusion>
    * section.
    */
    struct SDiffusionParams {
      /*
       * Maximum tolerance for the proximity reading between
       * the robot and the closest obstacle.
       * The proximity reading is 0 when nothing is detected
       * and grows exponentially to 1 when the obstacle is
       * touching the robot.
       */
      Real Delta;
      /* Angle tolerance range to go straight. */
      CRange<CRadians> GoStraightAngleRange;
 
      /* Constructor */
      SDiffusionParams();
 
      /* Parses the XML section for diffusion */
      void Init(TConfigurationNode& t_tree);
    };
 
    /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_foraging_controller><parameters><wheel_turning>
    * section.
    */
    struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;
 
      void Init(TConfigurationNode& t_tree);
    };
 
    /*
    * Contains all the state information about the controller.
    */
    struct SStateData {
      /* The three possible states in which the controller can be */
      enum EState {
         STATE_RESTING = 0,
         STATE_EXPLORING,
         STATE_RETURN_TO_NEST,
         STATE_FINISH
      } State;
 
      /* True when the robot is in the nest */
      bool InNest;
 
      /* Initial probability to switch from resting to exploring */
      Real InitialRestToExploreProb;
      /* Current probability to switch from resting to exploring */
      Real RestToExploreProb;
      /* Initial probability to switch from exploring to resting */
      Real InitialExploreToRestProb;
      /* Current probability to switch from exploring to resting */
      Real ExploreToRestProb;
      /* Used as a range for uniform number generation */
      CRange<Real> ProbRange;
      /* The increase of ExploreToRestProb due to the food rule */
      Real FoodRuleExploreToRestDeltaProb;
      /* The increase of RestToExploreProb due to the food rule */
      Real FoodRuleRestToExploreDeltaProb;
      /* The increase of ExploreToRestProb due to the collision rule */
      Real CollisionRuleExploreToRestDeltaProb;
      /* The increase of RestToExploreProb due to the social rule */
      Real SocialRuleRestToExploreDeltaProb;
      /* The increase of ExploreToRestProb due to the social rule */
      Real SocialRuleExploreToRestDeltaProb;
      /* The minimum number of steps in resting state before the robots
         starts thinking that it's time to move */
      size_t MinimumRestingTime;
      /* The number of steps in resting state */
      size_t TimeRested;
      /* The number of exploration steps without finding food after which
         a foot-bot starts thinking about going back to the nest */
      size_t MinimumUnsuccessfulExploreTime;
      /* The number of exploration steps without finding food */
      size_t TimeExploringUnsuccessfully;
      /* If the robots switched to resting as soon as it enters the nest,
         there would be overcrowding of robots in the border between the
         nest and the rest of the arena. To overcome this issue, the robot
         spends some time looking for a place in the nest before finally
         settling. The following variable contains the minimum time the
         robot must spend in state 'return to nest' looking for a place in
         the nest before switching to the resting state. */
      size_t MinimumSearchForPlaceInNestTime;
      /* The time spent searching for a place in the nest */
      size_t TimeSearchingForPlaceInNest;

      /*Count for color for influence*/
      int CountOfColor;

      /*Decision at nest to accept limited decisions*/
      int DecisionAtNest;
 
      /*Decision at explore area to accept limited decisions*/
      int DecisionAtExplore;
      int half;
      int cosnatantTime;
      float greenFractionTime;
      float blueFractionTime;
      float maxTimeRest;
      int MaxExperiemntTime;
      int TotalExecutionTime;
      int finishFlag;
      double duration;
      double start;
      
      SStateData();
      void Init(TConfigurationNode& t_node);
      void Reset();
    };
   
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
      //std::string mappingByzantinePath;
      std::string blockchainPath;
      std::string datadirBase;
      int basePort;
      //int numByzantine;
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
    
    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Reset();
   
   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy();
   
   /*
    * Returns true if the robot is currently exploring.
    */
   inline bool IsExploring() const {
      return m_sStateData.State == SStateData::STATE_EXPLORING;
   }
 
   /*
    * Returns true if the robot is currently resting.
    */
   inline bool IsResting() const {
      return m_sStateData.State == SStateData::STATE_RESTING;
   }
 
   /*
    * Returns true if the robot is currently returning to the nest.
    */
   inline bool IsReturningToNest() const {
      return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;
   }

   /*
    * Returns true if the robot is in FINISH state
    */
   inline bool IsFinished() const {
      return m_sStateData.State == SStateData::STATE_FINISH;
   }
 
   /*
    * Returns the food data
    */
   inline SFoodData& GetFoodData() {
      return m_sFoodData;
   }

   //virtual void RandomWalk();
   //void Explore();
   //void Move();
   //void TurnLeds();

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
   
   inline void setUseClassicalApproach(bool bUseClassicalApproach) {
      useClassicalApproach = bUseClassicalApproach;
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
   
   //void UpdateNeighbors(std::set<int> newNeighbors);
   //void registerRobot(); // Tell the smart contract the robot's public key
   //void updateRegistration(); // Wait for the first event of the smart contract 
   void fromLoopFunctionResPrepare();
   void fromLoopFunctionResStart();
   
private:
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
   
   /*
    * Calculates the diffusion vector. If there is a close obstacle,
    * it points away from it; it there is none, it points forwards.
    * The b_collision parameter is used to return true or false whether
    * a collision avoidance just happened or not. It is necessary for the
    * collision rule.
    */
   CVector2 DiffusionVector(bool& b_collision);
 
   /*
    * Gets a direction vector as input and transforms it into wheel
    * actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2& c_heading);
 
   /*
    * Executes the resting state.
    */
   void Rest();
 
   /*
    * Executes the exploring state.
    */
   void Explore();
 
   /*
    * Executes the return to nest state.
    */
   void ReturnToNest();

   /*
    * Reach consensus => STOP.
    */
   void Finish();
   
   //void InitGeth(int robotId);
   //void readNodeMapping();
   //void readByzantineMapping();
   //void DistributeID(); 
   //void killGethAndRemoveFolders(std::string bcPath, std::string regenFile);
   
   //Real m_fWheelVelocity;
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   CDegrees m_cAlpha;                         // OBST. AVOID.
   Real m_fDelta;                             // OBST. AVOID.
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the foot-bot motor ground sensor */
   CCI_FootBotMotorGroundSensor* m_pcGround;
   /* Pointer to the omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   CRange<CRadians> m_cGoStraightAngleRange;  // OBST. AVOID.
      
   CRandom::CRNG* m_pcRNG;
   
   /* Used in the social rule to communicate the result of the last
    * exploration attempt */
   enum ELastExplorationResult {
      LAST_EXPLORATION_NONE = 0,    // nothing to report
      LAST_EXPLORATION_SUCCESSFUL,  // the last exploration resulted in a food item found
      LAST_EXPLORATION_UNSUCCESSFUL // no food found in the last exploration
   } m_eLastExplorationResult;
 
   /* The controller state information */
   SStateData m_sStateData;
   /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;
   /* The diffusion parameters */
   SDiffusionParams m_sDiffusionParams;
   /* The food data */
   SFoodData m_sFoodData;

   /* All others used variables */
   bool useClassicalApproach;
   SimulationState simulationParams;
   
   Real m_fFoodSquareRadius;
   std::vector<CVector2> m_cFoodPos;
   CVector2 m_cPos;
   CColor m_cColor;
   
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
