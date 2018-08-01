/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <random>

using namespace argos;
using namespace std;

map<int, string> enodes;
map<int, string> coinbaseAddresses;
string interface; // Smart contract interface
SInt32 voteEveryXTicks = 5;

/****************************************/
/****************************************/
CBlockchainVotingController::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}
 
void CBlockchainVotingController::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/
void CBlockchainVotingController::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/
CBlockchainVotingController::SStateData::SStateData() :
   ProbRange(0.0f, 1.0f) {}
 
void CBlockchainVotingController::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "initial_rest_to_explore_prob", InitialRestToExploreProb);
      GetNodeAttribute(t_node, "initial_explore_to_rest_prob", InitialExploreToRestProb);
      GetNodeAttribute(t_node, "food_rule_explore_to_rest_delta_prob", FoodRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "food_rule_rest_to_explore_delta_prob", FoodRuleRestToExploreDeltaProb);
      GetNodeAttribute(t_node, "collision_rule_explore_to_rest_delta_prob", CollisionRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "social_rule_rest_to_explore_delta_prob", SocialRuleRestToExploreDeltaProb);
      GetNodeAttribute(t_node, "social_rule_explore_to_rest_delta_prob", SocialRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "minimum_resting_time", MinimumRestingTime);
      GetNodeAttribute(t_node, "minimum_unsuccessful_explore_time", MinimumUnsuccessfulExploreTime);
      GetNodeAttribute(t_node, "minimum_search_for_place_in_nest_time", MinimumSearchForPlaceInNestTime);
 
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}
 
void CBlockchainVotingController::SStateData::Reset() {
   State = STATE_RESTING;
   InNest = true;
   RestToExploreProb = InitialRestToExploreProb;
   ExploreToRestProb = InitialExploreToRestProb;
   TimeExploringUnsuccessfully = 0;
   /* Initially the robot is resting, and by setting RestingTime to
      MinimumRestingTime we force the robots to make a decision at the
      experiment start. If instead we set RestingTime to zero, we would
      have to wait till RestingTime reaches MinimumRestingTime before
      something happens, which is just a waste of time. */
   TimeRested = MinimumRestingTime;
   TimeSearchingForPlaceInNest = 0;
}

/****************************************/
/****************************************/
CBlockchainVotingController::Movement::Movement() :
  walkTime (3),
  actualDirection (0){}
  
/************************************************* INIT ********************************************************/
/***************************************************************************************************************/
void CBlockchainVotingController::SimulationState::Init(TConfigurationNode& t_node) {
  try{
    /* Getting sigma, G value and the decision rule to follow */
    //GetNodeAttribute(t_node, "g", g);
    //GetNodeAttribute(t_node, "sigma", sigma);
    GetNodeAttribute(t_node, "lamda", LAMDA);
    GetNodeAttribute(t_node, "turn", turn);
    //GetNodeAttribute(t_node, "decision_rule", decision_rule);
    GetNodeAttribute(t_node, "exitFlag", exitFlag);
    //GetNodeAttribute(t_node, "percent_white", percentRed);
    //GetNodeAttribute(t_node, "percent_black", percentBlue);
    //GetNodeAttribute(t_node, "num_pack_saved", numPackSaved);
    GetNodeAttribute(t_node, "base_dir", baseDir);
    GetNodeAttribute(t_node, "base_dir_raw", baseDirRaw);
    GetNodeAttribute(t_node, "interface_path", interfacePath);
    GetNodeAttribute(t_node, "mapping_path", mappingPath);
    GetNodeAttribute(t_node, "use_multiple_nodes", useMultipleNodes);
    GetNodeAttribute(t_node, "use_background_geth_calls", useBackgroundGethCalls);
    GetNodeAttribute(t_node, "blockchain_path", blockchainPath);
    GetNodeAttribute(t_node, "base_port", basePort);
    //GetNodeAttribute(t_node, "use_classical_approach", useClassicalApproach);
    GetNodeAttribute(t_node, "regenerate_file", regenerateFile);
    GetNodeAttribute(t_node, "max_unchanged_times", max_unchanged_times);
    GetNodeAttribute(t_node, "max_running_steps", max_running_steps);
    //GetNodeAttribute(t_node, "profiling", profiling);
  }
  catch(CARGoSException& ex) {
    THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
  }
}

CBlockchainVotingController::CBlockchainVotingController() :
    nodeInt(0),
    m_cColor(CColor::WHITE),
    m_cPrevColor(CColor::WHITE),
    m_lStepCnt(0),
    bMakeTurnOnce(false),
    nTurnStepsElapsed(0),
    bGenerateCheckConsensusScript(false),
    beginning(false),
    mining(false),
    m_pThreadPool(NULL),
    m_pcWheels (NULL),
    m_pcLEDs(NULL),
    //m_fWheelVelocity (10.0f),
    m_pcRABA (NULL),
    m_pcRABS (NULL),
    m_pcProximity(NULL),
    m_pcLight(NULL),
    m_pcGround(NULL),
    m_pcCamera(NULL),
    m_pcRNG(NULL) {}

/****************************************/
/****************************************/
void CBlockchainVotingController::Init(TConfigurationNode& t_node) {
    /*
    * Initialize sensors/actuators
    */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
    m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );

    m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
    m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
    m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );
    m_pcCamera    = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");

    /*
    * Parse XML parameters
    */
    /* Diffusion algorithm */
    m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
    /* Wheel turning */
    m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    /* Controller state */
    m_sStateData.Init(GetNode(t_node, "state"));

    m_pcRNG = CRandom::CreateRNG("argos");
    //GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    simulationParams.Init(GetNode(t_node, "simulation_parameters"));
  
    /*Enable the camera*/
    m_pcCamera->Enable();
    
    m_pThreadPool = new ThreadPool(1);
    
    Reset();
    
    /* Init REB actuators*/
    /*CCI_EPuckRangeAndBearingActuator::TData toSend;
    toSend[0]=5;
    toSend[1]=5;
    toSend[2]=5;
    toSend[3]=5;
    m_pcRABA->SetData(toSend);*/

    //readNodeMapping();
}

/****************************************/
/****************************************/
void CBlockchainVotingController::ControlStep() {
    m_lStepCnt++;
   
    /* switch state of STATE_RESTING, STATE_EXPLORING or STATE_RETURN_TO_NEST */
    /* Two different behaviours, depending on if they are diffusing or exploring */
    switch(m_sStateData.State) {
      case SStateData::STATE_EXPLORING: {
        Explore();
        break;
      }
      case SStateData::STATE_RESTING: {
        Rest();
        break;
        }
      case SStateData::STATE_RETURN_TO_NEST: {
        ReturnToNest();
        break;
      }    
      case SStateData::STATE_FINISH: {
        Finish();
        break;
      }
      default: {
        LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
    }
}

/****************************************/
/****************************************/ 
void CBlockchainVotingController::Reset() {
    /* Reset robot state */
    m_sStateData.Reset();
    /* Set LED color */
    m_pcLEDs->SetAllColors(CColor::RED);
    /* Clear up the last exploration result */
    m_eLastExplorationResult = LAST_EXPLORATION_NONE;
    m_pcRABA->ClearData();
    m_pcRABA->SetData(0, LAST_EXPLORATION_NONE);

    m_sStateData.DecisionAtExplore = 0;
    m_sStateData.DecisionAtNest = 0;
    //m_sStateData.half = 1;
    m_sStateData.cosnatantTime = 1000;
    m_sStateData.greenFractionTime = 0.3;
    m_sStateData.blueFractionTime = 0.6;
    m_sStateData.MaxExperiemntTime = 5000;
    m_sStateData.TotalExecutionTime = 0;
    m_sStateData.finishFlag = 0;
    m_sStateData.duration = 0;
    m_sStateData.start = 0;
    
    start = std::clock();
    
    m_cColor = CColor::GRAY50;
    m_cPrevColor = m_cColor;
    m_pcLEDs->SetAllColors(m_cColor);
    
    nUnchagedTimes = 0;
    bHasOpinion = false;
}

/****************************************/
/****************************************/
void CBlockchainVotingController::Destroy(){
  if(!useClassicalApproach) {
    int robotId = Geth_Wrapper::Id2Int(GetId());
    Geth_Wrapper::kill_geth_thread(robotId, simulationParams.basePort, nodeInt, simulationParams.blockchainPath);
  }
  if(m_pThreadPool != NULL)
      delete m_pThreadPool;
}

// Connect/disconnect Ethereum processes to each other
/*void CBlockchainVotingController::UpdateNeighbors(set<int> newNeighbors) {
  set<int> neighborsToAdd;
  set<int> neighborsToRemove;

  int robotId = Geth_Wrapper::Id2Int(GetId());
  
  //TODO: check code below because of error "set_difference is not a member of std"
  // // Old neighbors minus new neighbors = neighbors that should be removed
  // std::set_difference(neighbors.begin(),
                // neighbors.end(),
                // newNeighbors.begin(),
                // newNeighbors.end(),
                // std::inserter(neighborsToRemove, neighborsToRemove.end()));

  // // New neighbors minus old neighbors = neighbors that should be added
  // std::set_difference(newNeighbors.begin(),
                // newNeighbors.end(),
                // neighbors.begin(),
                // neighbors.end(),
                // std::inserter(neighborsToAdd, neighborsToAdd.end()));
  
  std::set<int>::iterator it;
  for (it = neighbors.begin(); it != neighbors.end(); ++it) {
    int i = *it;
  }

  for (it = newNeighbors.begin(); it != newNeighbors.end(); ++it) {
    int i = *it;
  }

  for (it = neighborsToRemove.begin(); it != neighborsToRemove.end(); ++it) {    
    int i = *it;
    string e = enodes[i];
    if (simulationParams.useBackgroundGethCalls)
      Geth_Wrapper::remove_peer_bg(robotId, e, nodeInt, simulationParams.blockchainPath);
    else
      Geth_Wrapper::remove_peer(robotId, e, nodeInt, simulationParams.blockchainPath);
  }
   
  for (it = neighborsToAdd.begin(); it != neighborsToAdd.end(); ++it) {
    int i = *it;
    string e = enodes[i];
    if (simulationParams.useBackgroundGethCalls)
      Geth_Wrapper::add_peer_bg(robotId, e, nodeInt, simulationParams.blockchainPath);
    else
      Geth_Wrapper::add_peer(robotId, e, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
  }
  
  // Update neighbor array
  set<int> neighborsTmp(newNeighbors);
  neighbors = neighborsTmp;  
}*/

/****************************************/
/****************************************/
void CBlockchainVotingController::UpdateState() {
    /* Reset state flags */
    m_sStateData.InNest = false;
    /* Read stuff from the ground sensor */
    const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
    /*
     * You can say whether you are in the nest by checking the ground sensor
     * placed close to the wheel motors. It returns a value between 0 and 1.
     * It is 1 when the robot is on a white area, it is 0 when the robot
     * is on a black area and it is around 0.5 when the robot is on a gray
     * area. 
     * The foot-bot has 4 sensors like this, two in the front
     * (corresponding to readings 0 and 1) and two in the back
     * (corresponding to reading 2 and 3).  Here we want the back sensors
     * (readings 2 and 3) to tell us whether we are on gray: if so, the
     * robot is completely in the nest, otherwise it's outside.
     */
    if(tGroundReads[2].Value < 0.01f && tGroundReads[3].Value < 0.01f) {
        m_sStateData.InNest = true;
    }
}
 
/****************************************/
/****************************************/ 
CVector2 CBlockchainVotingController::CalculateVectorToLight() {
    /* Get readings from light sensor */
    const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
    /* Sum them together */
    CVector2 cAccumulator;
    for(size_t i = 0; i < tLightReads.size(); ++i) {
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
    }
    /* If the light was perceived, return the vector */
    if(cAccumulator.Length() > 0.0f) {
      return CVector2(1.0f, cAccumulator.Angle());
    }
    /* Otherwise, return zero */
    else {
      return CVector2();
    }
}

/****************************************/
/****************************************/
CVector2 CBlockchainVotingController::DiffusionVector(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}

/****************************************/
/****************************************/ 
void CBlockchainVotingController::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/************************************************* EXPLORING STATE *********************************************/
/***************************************************************************************************************/
void CBlockchainVotingController::Rest() {
  /* If we have stayed here enough, probabilistically switch to
   * 'exploring' */
  int id = Geth_Wrapper::Id2Int(GetId());
  /*if( m_sStateData.TimeRested > m_sStateData.MinimumRestingTime &&
      m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.RestToExploreProb) {
    if(!bHasOpinion) {
      if(id > m_sStateData.half){
        m_cColor = CColor::CYAN;
      } else {
        m_cColor = CColor::YELLOW;
      }
    } else {
      if(m_sStateData.DecisionAtNest < 0){
        m_cColor = CColor::GREEN;
      } else {
        m_cColor = CColor::BLUE;
      }
    }
    m_pcLEDs->SetAllColors(m_cColor);
    m_sStateData.State = SStateData::STATE_EXPLORING;
    m_sStateData.TimeRested = 0;
    //std::cout << "======Robot: " << GetId() << " REST 1 => change to EXPLORE" << endl;
  } else*/ 
  if(m_sStateData.TimeRested <= m_sStateData.maxTimeRest){
    //++m_sStateData.TimeRested;
    /*const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
    for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
      if(m_sStateData.TimeRested % 10 == 0){
        std::string blue ("4294901760");
        std::string green ("4278255360");
        std::string tempColor = std::to_string(sReadings.BlobList[i]->Color);
        if(blue.compare(tempColor) == 0)
          m_sStateData.DecisionAtNest = m_sStateData.DecisionAtNest + 1;
          //m_sStateData.DecisionAtNest = 1;
        if(green.compare(tempColor) == 0)
          m_sStateData.DecisionAtNest = m_sStateData.DecisionAtNest - 1;
          //m_sStateData.DecisionAtNest = -1;
      }
    }*/

    /* Be sure not to send the last exploration result multiple times */
    //if(m_sStateData.TimeRested == 1) {
    //  m_pcRABA->SetData(0, LAST_EXPLORATION_NONE);
    //}
    if(m_sStateData.TimeRested == 0) {
      if((m_sStateData.DecisionAtExplore + m_sStateData.DecisionAtNest) > 0){
        m_cColor = CColor::BLUE;
        if(m_cPrevColor == m_cColor)
          nUnchagedTimes++;
        else
          nUnchagedTimes = 0;
        m_pcLEDs->SetAllColors(m_cColor);
        m_sStateData.DecisionAtExplore = 0;
        m_sStateData.maxTimeRest = m_sStateData.cosnatantTime * m_sStateData.blueFractionTime;
        std::cout << "======Robot: " << GetId() 
          << " REST 3a => RESTING and BLUE (decision>0)" << endl;
        if(!useClassicalApproach) {
          if(m_cPrevColor != m_cColor) {
            //VoteUsingGethCommands(id, "2");
            //// Create a thread using member function
            //std::thread th(&CBlockchainVotingController::VoteUsingGethCommands, this, id, "2");
            //th.join();
            // enqueue and store future
            auto result = m_pThreadPool->enqueue(
              [](int id, string option, int nodeInt, int basePort, 
                   string blockchainPath, string interface, string contractAddress) { 
                std::string strId = std::to_string(id);
                string args[2] = {strId, option};
                Geth_Wrapper::unlockAccount(id, "test", nodeInt, basePort, blockchainPath);
                long long nEther = Geth_Wrapper::check_ether(id, nodeInt, blockchainPath);
                Geth_Wrapper::start_mining(id, 1, nodeInt, blockchainPath);
                while(nEther <= 10) {
                    Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(1)", nodeInt, blockchainPath);
                    nEther = Geth_Wrapper::check_ether(id, nodeInt, blockchainPath);
                }
                Geth_Wrapper::smartContractInterfaceStringBg(id, interface, contractAddress, 
                    "voteForCandidate", args, 2, -1, nodeInt, blockchainPath);
                Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(2)", nodeInt, blockchainPath);
                //make sure the transaction is submitted and mined
                string args1[1] = {strId};
                //while(true) {
                string votedCand = Geth_Wrapper::smartContractInterfaceStringCall(id, interface, contractAddress, 
                    "getVotesFor", args1, 1, -1, nodeInt, blockchainPath);
                votedCand = Geth_Wrapper::replaceAll(votedCand, "\n", "");
                //std::cout << "Robot " << id << " - VoteUsingGethCommands: current=" 
                //    << votedCand << " <> " << option << endl;
                //}
                ostringstream ss;
                ss << "Robot " << id << " - VoteUsingGethCommands: current=" 
                   << votedCand << " <> " << option << endl;
                Geth_Wrapper::stop_mining(id, nodeInt, blockchainPath);
                return ss.str();
              }, 
              id, "2", nodeInt, simulationParams.basePort, simulationParams.blockchainPath, 
              interface, contractAddress
            );
          }
          //CheckConsensus(id);
          CheckConsensusUsingOneCommand(id);
        }
        m_cPrevColor = m_cColor;
      } else if ((m_sStateData.DecisionAtExplore + m_sStateData.DecisionAtNest) == 0){
        if(id > m_sStateData.half){
          m_cColor = CColor::BLUE;
          m_sStateData.maxTimeRest = m_sStateData.cosnatantTime * m_sStateData.blueFractionTime;
        } else {
          m_cColor = CColor::GREEN;
          m_sStateData.maxTimeRest = m_sStateData.cosnatantTime * m_sStateData.greenFractionTime;
        }
        m_pcLEDs->SetAllColors(m_cColor);
        std::cout << "======Robot: " << GetId() 
          << " REST 3b => RESTING (decision=0)" << endl;
      } else {
        m_cColor = CColor::GREEN;
        if(m_cPrevColor == m_cColor)
          nUnchagedTimes++;
        else
          nUnchagedTimes = 0;
        m_pcLEDs->SetAllColors(m_cColor);
        m_sStateData.DecisionAtExplore = 0;
        m_sStateData.maxTimeRest = m_sStateData.cosnatantTime * m_sStateData.greenFractionTime;
        std::cout << "======Robot: " << GetId() 
          << " REST 3c => RESTING and GREEN (decision<0)" << endl;
        if(!useClassicalApproach) {
          if(m_cPrevColor != m_cColor) {
            //VoteUsingGethCommands(id, "1");
            //std::thread th(&CBlockchainVotingController::VoteUsingGethCommands, this, id, "1");
            //th.join();
            auto result = m_pThreadPool->enqueue(
              [](int id, string option, int nodeInt, int basePort, 
                   string blockchainPath, string interface, string contractAddress) { 
                std::string strId = std::to_string(id);
                string args[2] = {strId, option};
                Geth_Wrapper::unlockAccount(id, "test", nodeInt, basePort, blockchainPath);
                long long nEther = Geth_Wrapper::check_ether(id, nodeInt, blockchainPath);
                Geth_Wrapper::start_mining(id, 1, nodeInt, blockchainPath);
                while(nEther <= 10) {
                    Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(1)", nodeInt, blockchainPath);
                    nEther = Geth_Wrapper::check_ether(id, nodeInt, blockchainPath);
                }
                Geth_Wrapper::smartContractInterfaceStringBg(id, interface, contractAddress, 
                    "voteForCandidate", args, 2, -1, nodeInt, blockchainPath);
                Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(2)", nodeInt, blockchainPath);
                //make sure the transaction is submitted and mined
                string args1[1] = {strId};
                //while(true) {
                string votedCand = Geth_Wrapper::smartContractInterfaceStringCall(id, interface, contractAddress, 
                    "getVotesFor", args1, 1, -1, nodeInt, blockchainPath);
                votedCand = Geth_Wrapper::replaceAll(votedCand, "\n", "");
                //std::cout << "Robot " << id << " - VoteUsingGethCommands: current=" 
                //    << votedCand << " <> " << option << endl;
                //}
                ostringstream ss;
                ss << "Robot " << id << " - VoteUsingGethCommands: current=" 
                   << votedCand << " <> " << option << endl;
                Geth_Wrapper::stop_mining(id, nodeInt, blockchainPath);
                return ss.str();
              }, 
              id, "1", nodeInt, simulationParams.basePort, simulationParams.blockchainPath, 
              interface, contractAddress
            );
          }
          //CheckConsensus(id);
          CheckConsensusUsingOneCommand(id);
        }
        m_cPrevColor = m_cColor;
      }
    }
    
    m_sStateData.TimeRested++;
    //m_pcLEDs->SetAllColors(m_cColor);
    //std::cout << "======Robot: " << GetId() 
    //    << " REST 2 m_sStateData.DecisionAtNest=" << m_sStateData.DecisionAtNest << endl;
    
    //if(m_sStateData.TimeRested % 10 ==0)
    //  CheckConsensusUsingOneCommand(id);
    
    /*
     * Social rule: listen to what other people have found and modify
     * probabilities accordingly
     */
    //const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
  } else {
    UpdateState();
    m_sStateData.State = SStateData::STATE_EXPLORING;
    m_sStateData.TimeRested = 0;
    /*if(m_sStateData.DecisionAtExplore + m_sStateData.DecisionAtNest > 0){
      m_cColor = CColor::BLUE;
      if(m_cPrevColor == m_cColor)
        nUnchagedTimes++;
      else
        nUnchagedTimes = 0;
      m_pcLEDs->SetAllColors(m_cColor);
      m_sStateData.DecisionAtExplore = 0;
      m_sStateData.maxTimeRest = m_sStateData.cosnatantTime * m_sStateData.blueFractionTime;
      std::cout << "======Robot: " << GetId() 
          << " REST 3a => EXPLORE and BLUE (decision>0)" << endl;
      if(!useClassicalApproach) {
        if(m_cPrevColor != m_cColor) {
          //VoteUsingGethCommands(id, "2");
          //// Create a thread using member function
          //std::thread th(&CBlockchainVotingController::VoteUsingGethCommands, this, id, "2");
          //th.join();
          // enqueue and store future
          auto result = m_pThreadPool->enqueue(
              [](int id, string option, int nodeInt, int basePort, 
                   string blockchainPath, string interface, string contractAddress) { 
                std::string strId = std::to_string(id);
                string args[2] = {strId, option};
                Geth_Wrapper::unlockAccount(id, "test", nodeInt, basePort, blockchainPath);
                long long nEther = Geth_Wrapper::check_ether(id, nodeInt, blockchainPath);
                Geth_Wrapper::start_mining(id, 1, nodeInt, blockchainPath);
                while(nEther <= 10) {
                    Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(1)", nodeInt, blockchainPath);
                    nEther = Geth_Wrapper::check_ether(id, nodeInt, blockchainPath);
                }
                Geth_Wrapper::smartContractInterfaceStringBg(id, interface, contractAddress, 
                    "voteForCandidate", args, 2, -1, nodeInt, blockchainPath);
                Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(2)", nodeInt, blockchainPath);
                //make sure the transaction is submitted and mined
                string args1[1] = {strId};
                //while(true) {
                string votedCand = Geth_Wrapper::smartContractInterfaceStringCall(id, interface, contractAddress, 
                    "getVotesFor", args1, 1, -1, nodeInt, blockchainPath);
                votedCand = Geth_Wrapper::replaceAll(votedCand, "\n", "");
                //std::cout << "Robot " << id << " - VoteUsingGethCommands: current=" 
                //    << votedCand << " <> " << option << endl;
                //}
                ostringstream ss;
                ss << "Robot " << id << " - VoteUsingGethCommands: current=" 
                   << votedCand << " <> " << option << endl;
                Geth_Wrapper::stop_mining(id, nodeInt, blockchainPath);
                return ss.str();
              }, 
              id, "2", nodeInt, simulationParams.basePort, simulationParams.blockchainPath, 
              interface, contractAddress
          );
        }
        //CheckConsensus(id);
        CheckConsensusUsingOneCommand(id);
      }
      
      m_cPrevColor = m_cColor;
    }
    else if(m_sStateData.DecisionAtExplore + m_sStateData.DecisionAtNest == 0){
      if(id > m_sStateData.half){
        m_cColor = CColor::BLUE;
        m_sStateData.maxTimeRest = m_sStateData.cosnatantTime * m_sStateData.blueFractionTime;
      } else {
        m_cColor = CColor::GREEN;
        m_sStateData.maxTimeRest = m_sStateData.cosnatantTime * m_sStateData.greenFractionTime;
      }
      m_pcLEDs->SetAllColors(m_cColor);
      std::cout << "======Robot: " << GetId() 
          << " REST 3b => EXPLORE (decision=0)" << endl;
    }
    else if(m_sStateData.DecisionAtExplore + m_sStateData.DecisionAtNest < 0){
      m_cColor = CColor::GREEN;
      if(m_cPrevColor == m_cColor)
        nUnchagedTimes++;
      else
        nUnchagedTimes = 0;
      m_pcLEDs->SetAllColors(m_cColor);
      m_sStateData.DecisionAtExplore = 0;
      m_sStateData.maxTimeRest = m_sStateData.cosnatantTime * m_sStateData.greenFractionTime;
      std::cout << "======Robot: " << GetId() 
          << " REST 3c => EXPLORE and GREEN (decision<0)" << endl;
      if(!useClassicalApproach) {
        if(m_cPrevColor != m_cColor) {
          //VoteUsingGethCommands(id, "1");
          //std::thread th(&CBlockchainVotingController::VoteUsingGethCommands, this, id, "1");
          //th.join();
          auto result = m_pThreadPool->enqueue(
              [](int id, string option, int nodeInt, int basePort, 
                   string blockchainPath, string interface, string contractAddress) { 
                std::string strId = std::to_string(id);
                string args[2] = {strId, option};
                Geth_Wrapper::unlockAccount(id, "test", nodeInt, basePort, blockchainPath);
                long long nEther = Geth_Wrapper::check_ether(id, nodeInt, blockchainPath);
                Geth_Wrapper::start_mining(id, 1, nodeInt, blockchainPath);
                while(nEther <= 10) {
                    Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(1)", nodeInt, blockchainPath);
                    nEther = Geth_Wrapper::check_ether(id, nodeInt, blockchainPath);
                }
                Geth_Wrapper::smartContractInterfaceStringBg(id, interface, contractAddress, 
                    "voteForCandidate", args, 2, -1, nodeInt, blockchainPath);
                Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(2)", nodeInt, blockchainPath);
                //make sure the transaction is submitted and mined
                string args1[1] = {strId};
                //while(true) {
                string votedCand = Geth_Wrapper::smartContractInterfaceStringCall(id, interface, contractAddress, 
                    "getVotesFor", args1, 1, -1, nodeInt, blockchainPath);
                votedCand = Geth_Wrapper::replaceAll(votedCand, "\n", "");
                //std::cout << "Robot " << id << " - VoteUsingGethCommands: current=" 
                //    << votedCand << " <> " << option << endl;
                //}
                ostringstream ss;
                ss << "Robot " << id << " - VoteUsingGethCommands: current=" 
                   << votedCand << " <> " << option << endl;
                Geth_Wrapper::stop_mining(id, nodeInt, blockchainPath);
                return ss.str();
              }, 
              id, "1", nodeInt, simulationParams.basePort, simulationParams.blockchainPath, 
              interface, contractAddress
          );
        }
        //CheckConsensus(id);
        CheckConsensusUsingOneCommand(id);
      }
      
      m_cPrevColor = m_cColor;
    }*/
  }
  
  /*const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
  if(((m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore) == 0) 
      && (id >= m_sStateData.half) 
      && (   (0.28 < tGroundReads[1].Value && tGroundReads[1].Value < 0.3 ) 
          || (0.28 < tGroundReads[2].Value && tGroundReads[2].Value < 0.3 ))) {
    m_pcWheels->SetLinearVelocity(70,0);
    std::cout << "======Robot: " << GetId() 
        << " REST 4 decision=0 && ground value betweem 0.28 && 0.3 "
        << " REST 4 => m_pcWheels->SetLinearVelocity(70,0)" << endl;
  }*/
  if(useClassicalApproach) {
    if(   nUnchagedTimes >= simulationParams.max_unchanged_times
       || m_lStepCnt >=  simulationParams.max_running_steps) {
      m_sStateData.State = SStateData::STATE_FINISH;
      m_pcWheels->SetLinearVelocity(0,0);
    }
  }
}

bool CBlockchainVotingController::CheckConsensus(int id) {
  string args1[] = {};
  string args2[] = {"2"};
  string args3[] = {"1"};
  string testConsensus = Geth_Wrapper::smartContractInterfaceStringCall(id, interface, contractAddress, 
    "consensus", args1, 0, -1, nodeInt, simulationParams.blockchainPath);
  string blueCount = Geth_Wrapper::smartContractInterfaceStringCall(id, interface, contractAddress, 
    "totalVotesFor", args2, 1, -1, nodeInt, simulationParams.blockchainPath);
  string greenCount = Geth_Wrapper::smartContractInterfaceStringCall(id, interface, contractAddress, 
    "totalVotesFor", args3, 1, -1, nodeInt, simulationParams.blockchainPath);
  //std::string trueFinish ("true");
  if(testConsensus.find("true") != string::npos){
    std::cout << "++++++++ Stop:+++++++: " << testConsensus << std::endl;
    m_sStateData.State = SStateData::STATE_FINISH;
    std::cout << "Vote for blue:  " << blueCount << std::endl;
    std::cout << "Vote for green: " << greenCount << std::endl;
    return true;
  } else {
    std::cout << "++++++++ Not Stop:+++++++: " << testConsensus << std::endl;
    std::cout << "Vote for blue:  " << blueCount << std::endl;
    std::cout << "Vote for green: " << greenCount << std::endl;
    return false;
  }
}

bool CBlockchainVotingController::CheckConsensusUsingScript(int id) {
  string tmpPath = simulationParams.baseDirRaw + "/checkConsensus.js";
  if(!bGenerateCheckConsensusScript) {
    ostringstream fullCommandStream;
    fullCommandStream << "var cC = web3.eth.contract(" << interface << ");" << endl;
    fullCommandStream << "var c = cC.at(" << contractAddress << ");" << endl;
    fullCommandStream << "console.log(\"Consensus=\" + c.consensus.call());" << endl;
    fullCommandStream << "console.log(\"Vote for blue=\" + c.totalVotesFor.call(\"2\"));" << endl;
    fullCommandStream << "console.log(\"Vote for green=\" + c.totalVotesFor.call(\"1\"));" << endl;
    string fullCommand = fullCommandStream.str();
    ofstream out(tmpPath.c_str());
    out << fullCommand;
    out.close();
    bGenerateCheckConsensusScript = true;
  }

  string testConsensus = Geth_Wrapper::exec_geth_cmd(id, "loadScript(\"" + tmpPath + "\")", 
    nodeInt, simulationParams.blockchainPath);
  //cout << "[CheckConsensusUsingScript] res: " << testConsensus << endl;
  if(testConsensus.find("Consensus=true") != string::npos){
    std::cout << "++++++++ Stop:+++++++: " << testConsensus << std::endl;
    m_sStateData.State = SStateData::STATE_FINISH;
    return true;
  } else {
    std::cout << "++++++++ Not Stop:+++++++: " << testConsensus << std::endl;
    return false;
  }
}

bool CBlockchainVotingController::CheckConsensusUsingOneCommand(int id) {
  ostringstream fullCommandStream;
  fullCommandStream << "var cC = web3.eth.contract(" << interface << ");";
  fullCommandStream << "var c = cC.at(" << contractAddress << ");";
  fullCommandStream << "console.log(\"Consensus=\" + c.consensus.call());";
  fullCommandStream << "console.log(\"Vote for blue=\" + c.totalVotesFor.call(\"2\"));";
  fullCommandStream << "console.log(\"Vote for green=\" + c.totalVotesFor.call(\"1\"));";
  string fullCommand = fullCommandStream.str();
  //std::cout << "CheckConsensus command: " << fullCommand << endl;
  string testConsensus = Geth_Wrapper::exec_geth_cmd(id, fullCommand, 
    nodeInt, simulationParams.blockchainPath);
  //cout << "[CheckConsensusUsingOneCommand] res: " << testConsensus << endl;
  if(testConsensus.find("Consensus=true") != string::npos){
    std::cout << "++++++++ Stop:+++++++: " << testConsensus << std::endl;
    m_sStateData.State = SStateData::STATE_FINISH;
    return true;
  } else {
    std::cout << "++++++++ Not Stop:+++++++: " << testConsensus << std::endl;
    return false;
  }
}

bool CBlockchainVotingController::VoteUsingGethCommands(int id, string option) {
  std::string strId = std::to_string(id);
  string args[2] = {strId, option};
  Geth_Wrapper::unlockAccount(id, "test", nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
  long long nEther = Geth_Wrapper::check_ether(id, nodeInt, simulationParams.blockchainPath);
  Geth_Wrapper::start_mining(id, 1, nodeInt, simulationParams.blockchainPath);
  while(nEther <= 10) {
    Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(1)", nodeInt, simulationParams.blockchainPath);
    nEther = Geth_Wrapper::check_ether(id, nodeInt, simulationParams.blockchainPath);
  }
  Geth_Wrapper::smartContractInterfaceStringBg(id, interface, contractAddress, 
    "voteForCandidate", args, 2, -1, nodeInt, simulationParams.blockchainPath);
  Geth_Wrapper::exec_geth_cmd_helper(id, "admin.sleepBlocks(2)", nodeInt, simulationParams.blockchainPath);
  //make sure the transaction is submitted and mined
  string args1[1] = {strId};
  //while(true) {
    string votedCand = Geth_Wrapper::smartContractInterfaceStringCall(id, interface, contractAddress, 
      "getVotesFor", args1, 1, -1, nodeInt, simulationParams.blockchainPath);
    votedCand = Geth_Wrapper::replaceAll(votedCand, "\n", "");
    std::cout << "Robot " << id << " - VoteUsingGethCommands: current=" 
        << votedCand << " <> " << option << endl; 
    //if(votedCand.compare(option) == 0) {
    //  break;
    //}
  //}
  Geth_Wrapper::stop_mining(id, nodeInt, simulationParams.blockchainPath);
  return true;
}

bool CBlockchainVotingController::VoteUsingScript(int id, string option) {
  std::string strId = std::to_string(id);
  string tmpPath = simulationParams.baseDirRaw + "/submitVote"+ strId + ".js";
  //std::ifstream inFile(tmpPath.c_str());
  //if(!inFile.good()) {
    ostringstream fullCommandStream;
    fullCommandStream << "web3.personal.unlockAccount(eth.coinbase, \"test\");" << endl;
    fullCommandStream << "nEther=web3.fromWei(eth.getBalance(eth.coinbase),\"ether\");" << endl;
    fullCommandStream << "web3.miner.start(1);" << endl;
    fullCommandStream << "while(nEther <= 10) {" << endl;
    fullCommandStream << "  sleep(5);" << endl;
    fullCommandStream << "  nEther=web3.fromWei(eth.getBalance(eth.coinbase),\"ether\");" << endl;
    fullCommandStream << "}" << endl;
    fullCommandStream << "var cC = web3.eth.contract(" << interface << ");"  << endl;
    fullCommandStream << "var c = cC.at(" << contractAddress << ");" << endl;
    fullCommandStream << "c.voteForCandidate(\""<< strId << "\",\"" << option << "\",";
    fullCommandStream << "  {from: eth.coinbase, gas: 3000000});" << endl;
    fullCommandStream << "web3.miner.stop();";
    string fullCommand = fullCommandStream.str();
    ofstream out(tmpPath.c_str());
    out << fullCommand;
    out.close();
  //} else {
  //  inFile.close();
  //}

  string res = Geth_Wrapper::exec_geth_cmd_helper(id, "loadScript(\"" + tmpPath + "\")", 
    nodeInt, simulationParams.blockchainPath);
  //cout << "[smartContractInterfaceFuncCallScript] res: " << res << endl;
  //return res;
  if(res.find("true") != string::npos){
    return true;
  } else {
    return false;
  }
}

void CBlockchainVotingController::Explore() {
  bool bReturnToNest(false);
  int id = Geth_Wrapper::Id2Int(GetId());
  
  if(!bHasOpinion) {
    m_cColor = CColor::GRAY50;
  }
  m_pcLEDs->SetAllColors(m_cColor);

  /* Test the second condition: we probabilistically switch to 'return to
   * nest' if we have been wandering for some time and found nothing */
  /*if(m_sStateData.TimeExploringUnsuccessfully > m_sStateData.MinimumUnsuccessfulExploreTime) {
    if (m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.ExploreToRestProb) {
      // Store the result of the expedition //
      m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
      // Switch to 'return to nest' //
      bReturnToNest = true;
      std::cout << "======Robot: " << GetId() 
        << " EXPLORE UNSUCCESSFUL => bReturnToNest=true " << endl;
    } else {
      // Apply the food rule, increasing ExploreToRestProb and
      // decreasing RestToExploreProb //
      m_sStateData.ExploreToRestProb += m_sStateData.FoodRuleExploreToRestDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
      m_sStateData.RestToExploreProb -= m_sStateData.FoodRuleRestToExploreDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
    }
  }*/

  //if(bReturnToNest){
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
    if(! sReadings.BlobList.empty()) {
      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
        if(m_sStateData.DecisionAtExplore==0){
          /*If the robot finds a cyan color in exploration area add 1 to the constant else subtract 1 to the constant*/
          std::string cyan ("4294967040");
          std::string yellow ("4278255615");
          std::string tempColor = std::to_string(sReadings.BlobList[i]->Color);
          if(cyan.compare(tempColor) == 0) {
            m_sStateData.DecisionAtExplore = 1;
            bReturnToNest = true;
            bHasOpinion = true;
          }
          if(yellow.compare(tempColor) == 0) {
            m_sStateData.DecisionAtExplore = -1;
            bReturnToNest = true;
            bHasOpinion = true;
          }
        }
      }
    }
  //}

  /* So, do we return to the nest now? */
  //if(bReturnToNest) {
  //  /* Yes, we do! */
  //  m_sStateData.TimeExploringUnsuccessfully = 0;
  //  m_sStateData.TimeSearchingForPlaceInNest = 0;
  //  //m_pcLEDs->SetAllColors(CColor::BLUE);
  //  m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
  //  std::cout << "======Robot: " << GetId() 
  //      << " EXPLORE SUCCESSFUL => STATE_RETURN_TO_NEST " << endl;
  //} else {
    /* No, perform the actual exploration */
    ++m_sStateData.TimeExploringUnsuccessfully;
    UpdateState();
    /* Get the diffusion vector to perform obstacle avoidance */
    bool bCollision;
    CVector2 cDiffusion = DiffusionVector(bCollision);
    /* Apply the collision rule, if a collision avoidance happened */
    if(bCollision) {
      /* Collision avoidance happened, increase ExploreToRestProb and
       * decrease RestToExploreProb */
      m_sStateData.ExploreToRestProb += m_sStateData.CollisionRuleExploreToRestDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
      m_sStateData.RestToExploreProb -= m_sStateData.CollisionRuleExploreToRestDeltaProb;
      m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
    }
    /*
     * If we are in the nest, we combine antiphototaxis with obstacle avoidance
     * Outside the nest, we just use the diffusion vector
     */
    if(m_sStateData.InNest) {
      /*
       * The vector returned by CalculateVectorToLight() points to
       * the light. Thus, the minus sign is because we want to go away
       * from the light.
       */
      const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
      if((tGroundReads[1].Value < 0.01) ||(tGroundReads[2].Value < 0.01))    {
        SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
        bMakeTurnOnce = false;
        nTurnStepsElapsed = 0;
      }
    } else {
      /* Use the diffusion vector only */
      const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
      if(    (tGroundReads[1].Value > 0.1 && tGroundReads[1].Value < 0.12 ) 
          || (tGroundReads[2].Value > 0.1 && tGroundReads[2].Value < 0.12 ) 
          || (tGroundReads[3].Value > 0.1 && tGroundReads[3].Value < 0.12 ) 
          || (tGroundReads[4].Value > 0.1 && tGroundReads[4].Value < 0.12 )) {
        if(   ((m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore) > 0) 
           || ( ((m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore) == 0) 
                && (id > m_sStateData.half)) ) {
          SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
        } else if(   ((m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore) < 0) 
                  || ( ((m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore) == 0)
                      && (id <= m_sStateData.half)) ){
          //std::cout << "======Robot: " << GetId() << " Change direction" << endl;
          //std::cout << "======Robot: " << GetId() 
          //  << " EXPLORING - IN BLUE need to turn back" 
          //  << " decision=" << m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore << endl;
          if(!bMakeTurnOnce) {
            m_pcWheels->SetLinearVelocity(90,0);
            bMakeTurnOnce = true;
          }
          nTurnStepsElapsed++;
          if(nTurnStepsElapsed > 50) { 
            // In stuck state => change return direction
            std::cout << "======Robot: " << GetId() 
              << " EXPLORING - IN BLUE need to turn back BUT STUCK => change method" << endl;
            m_pcWheels->SetLinearVelocity(0,-70);
            bMakeTurnOnce = true;
          }
        }
        if(m_sStateData.DecisionAtExplore != 0) {
          /* Yes, we do! */
          m_sStateData.TimeExploringUnsuccessfully = 0;
          m_sStateData.TimeSearchingForPlaceInNest = 0;
          m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
          std::cout << "======Robot: " << GetId() 
            << " EXPLORE SUCCESSFUL => STATE_RETURN_TO_NEST - DecisionAtExplore=" 
            << m_sStateData.DecisionAtExplore << endl;
        }
      } else if((tGroundReads[1].Value > 0.4 && tGroundReads[1].Value < 0.6 ) 
            ||  (tGroundReads[2].Value > 0.4 && tGroundReads[2].Value < 0.6 ) 
            ||  (tGroundReads[3].Value > 0.4 && tGroundReads[3].Value < 0.6 ) 
            ||  (tGroundReads[4].Value > 0.4 && tGroundReads[4].Value < 0.6 )) {
        if(   ((m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore) < 0) 
           || ( ((m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore) == 0) 
                && (id <= m_sStateData.half)) ) {
          SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
        } else if(   ((m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore) > 0) 
                  || ( ((m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore) == 0)
                      && (id > m_sStateData.half)) ){
          //std::cout << "======Robot: " << GetId() << " Change direction" << endl;
          //std::cout << "======Robot: " << GetId() 
          //  << " EXPLORING - IN GREEN need to turn back" 
          //  << " decision=" << m_sStateData.DecisionAtNest + m_sStateData.DecisionAtExplore << endl;
          if(!bMakeTurnOnce) {
            m_pcWheels->SetLinearVelocity(90,0);
            bMakeTurnOnce = true;
          }
          nTurnStepsElapsed++;
          if(nTurnStepsElapsed > 50) { 
            // In stuck state => change return direction
            std::cout << "======Robot: " << GetId() 
              << " EXPLORING - IN GREEN need to turn back BUT STUCK => change method" << endl;
            m_pcWheels->SetLinearVelocity(0,-70);
            bMakeTurnOnce = true;
          }
        }
        if(m_sStateData.DecisionAtExplore != 0) {
          /* Yes, we do! */
          m_sStateData.TimeExploringUnsuccessfully = 0;
          m_sStateData.TimeSearchingForPlaceInNest = 0;
          m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
          std::cout << "======Robot: " << GetId() 
            << " EXPLORE SUCCESSFUL => STATE_RETURN_TO_NEST - DecisionAtExplore=" 
            << m_sStateData.DecisionAtExplore << endl;
        }
      } else {
        SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
      }
    }
  //}
}

/****************************************/
/****************************************/
void CBlockchainVotingController::ReturnToNest() {
  /* As soon as you get to the nest, switch to 'resting' */
  UpdateState();

  const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();

  /* Are we in the nest? */
  const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
  if(m_sStateData.InNest) {
    /* Have we looked for a place long enough? */
    if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime) {
      /* Yes, stop the wheels... */
      m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
      /* Tell people about the last exploration attempt */
      //m_pcRABA->SetData(0, m_eLastExplorationResult);
      /* ... and switch to state 'resting' */
      //m_pcLEDs->SetSingleColor(12, CColor::RED);
      m_sStateData.State = SStateData::STATE_RESTING;
      m_sStateData.TimeSearchingForPlaceInNest = 0;
      m_sStateData.TimeRested = 0;
      m_eLastExplorationResult = LAST_EXPLORATION_NONE;
      std::cout << "======Robot: " << GetId() 
          << " ReturnToNest SUCCESSFUL => STATE_RESTING" << endl;
      return;
    } else {
      /* No, keep looking */
      ++m_sStateData.TimeSearchingForPlaceInNest;
      m_pcLEDs->SetSingleColor(12, CColor::RED);
      const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
        if(m_sStateData.TimeRested % 10 == 0){
          std::string blue ("4294901760");
          std::string green ("4278255360");
          std::string tempColor = std::to_string(sReadings.BlobList[i]->Color);
          if(blue.compare(tempColor) == 0)
            m_sStateData.DecisionAtNest = m_sStateData.DecisionAtNest + 1;
            //m_sStateData.DecisionAtNest = 1;
          if(green.compare(tempColor) == 0)
            m_sStateData.DecisionAtNest = m_sStateData.DecisionAtNest - 1;
            //m_sStateData.DecisionAtNest = -1;
        }
      }
    }
  } else {
    /* Still outside the nest */
    m_sStateData.TimeSearchingForPlaceInNest = 0;
    //  m_pcWheels->SetLinearVelocity(20.0f, 0.0f);
  }
  /* Keep going */
  bool bCollision;
  SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision));
}

/****************************************/
/****************************************/
void CBlockchainVotingController::Finish() {
  if(!bFinishedOutput) {
    end = clock();
    elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
    std::cout << "======Robot: " << GetId() << " Finish experiment in " 
              << elapsed_secs << " seconds ======" << endl;
    //std::cout << "======Robot: " << GetId() << " - Finish experiment======" << endl;
    bFinishedOutput = true;
  }
  m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

/****************************************/
/****************************************/
void CBlockchainVotingController::setColor(CColor color) {
  if(m_cColor == CColor::WHITE 
     && (color == CColor::RED || color == CColor::GREEN || color == CColor::BLUE)) {
    string strColor;
    if(color == CColor::RED) 
      strColor = "RED";
    else if(color == CColor::GREEN)
      strColor = "GREEN";
    else 
      strColor = "BLUE";
    string args[1] = {strColor};        
  
    int robotId = Geth_Wrapper::Id2Int(GetId());
    cout << "setColor starting...id=" << robotId << ", color=" << strColor << endl;
    Geth_Wrapper::unlockAccount(robotId, "test", nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
    long long nEther = Geth_Wrapper::check_ether(robotId, nodeInt, simulationParams.blockchainPath);
    Geth_Wrapper::start_mining(robotId, 1, nodeInt, simulationParams.blockchainPath);
    while(nEther <= 20) {
      Geth_Wrapper::exec_geth_cmd_helper(robotId, "admin.sleepBlocks(2)", nodeInt, simulationParams.blockchainPath);
      nEther = Geth_Wrapper::check_ether(robotId, nodeInt, simulationParams.blockchainPath);
    }
    //stop_mining(robotId, nodeInt, simulationParams.blockchainPath);
    cout << "Robot " << robotId << " current ether " << nEther << endl;
    Geth_Wrapper::smartContractInterfaceStringBg(robotId, interface, contractAddress, "voteForCandidate", args, 1, -1, nodeInt, simulationParams.blockchainPath);
    //Geth_Wrapper::smartContractInterfaceFuncScript(robotId, interface, contractAddress, "voteForCandidate", args, 1, -1, nodeInt, simulationParams.blockchainPath);
    //Geth_Wrapper::start_mining(robotId, 1, nodeInt, simulationParams.blockchainPath);
    Geth_Wrapper::exec_geth_cmd_helper(robotId, "admin.sleepBlocks(1)", nodeInt, simulationParams.blockchainPath);
    Geth_Wrapper::stop_mining(robotId, nodeInt, simulationParams.blockchainPath);
  }
  
  m_cColor = color;
}

/*********************** RANDOM WALK *******************************/
/******************************************************************/
/* void CBlockchainVotingController::RandomWalk() {
  // // walkTime represents the number of clock cycles (random number) of walk in a random direction
  // if ( movement.walkTime == 0 )      // Is the walkTime in that direction finished? ->
  // {                                  // -> YES: change direction//
    // if ( movement.actualDirection == 0 )  // If robot was going straight then turn standing in ->
    // // -> a position for an uniformly distribuited time //
    // {
      // CRange<Real> zeroOne(0.0,1.0);
      // Real p = m_pcRNG->Uniform(zeroOne);
      // p = p*simulationParams.turn;
      // Real dir = m_pcRNG->Uniform(CRange<Real>(-1.0,1.0));
      // if ( dir > 0 )
        // movement.actualDirection = 1;
      // else
        // movement.actualDirection = 2;
      // movement.walkTime = Floor(p);
    // }
    // else                         // The robot was turning, time to go straight for ->
    // // -> an exponential period of time //
    // {
      // movement.walkTime = Ceil(m_pcRNG->Exponential((Real)simulationParams.LAMDA)*4); // Exponential random generator. *50 is a scale factor for the time
      // movement.actualDirection = 0;
    // }
  // }
  // else {                            // NO: The period of time is not finished, decrement the ->
    // // -> walkTime and keep the direction //
    // movement.walkTime--;
  // }
// }*/

/*********************** MOVEMENT *********************************/
/******************************************************************/
/* // Implement the moviment leaded by the random walk (see loop_function)
// void CBlockchainVotingController::Move(){
  // if(movement.actualDirection == 0) // Go straight
    // m_pcWheels->SetLinearVelocity(m_fWheelVelocity,  m_fWheelVelocity);
  // else
    // if(movement.actualDirection == 1) // Turn right
      // m_pcWheels->SetLinearVelocity(m_fWheelVelocity,  -m_fWheelVelocity);
    // else
      // if(movement.actualDirection == 2) // Turn left
    // m_pcWheels->SetLinearVelocity(-m_fWheelVelocity,  m_fWheelVelocity);
// }*/

/****************************************/
/****************************************/
// Tell the smart contract the robot's public key
/*void CBlockchainVotingController::registerRobot() {
  int robotId = Geth_Wrapper::Id2Int(GetId());  
  //int args[1] = {(int) opinion.actualOpinion};
  int emptyArgs[0] = {};
  cout << "START now registrating robot " << robotId << endl; 
  // Modify state of the blockchain
  Geth_Wrapper::smartContractInterfaceBg(robotId, interface,
     //contractAddress, "registerRobot", args, 1, 0, nodeInt, simulationParams.blockchainPath);
     contractAddress, "registerRobot", emptyArgs, 1, 0, nodeInt, simulationParams.blockchainPath);
  cout << "END now registrating robot " << robotId << endl; 
}*/

// Wait for the first event of the smart contract
/*void CBlockchainVotingController::updateRegistration() {
  int robotId = Geth_Wrapper::Id2Int(GetId());
  string eventResult;
  do {
    eventResult = Geth_Wrapper::eventInterface(robotId, interface, contractAddress, nodeInt, simulationParams.blockchainPath);    
  } while (eventResult.find("Error") != string::npos);
  
  vector<string> splitResult = Geth_Wrapper::split(eventResult, ' ');    
  std::string sNewOpinion = splitResult[2];
  std::string sBlock = splitResult[1];
  std::string sBlockhash = splitResult[0];      
  cout << "sNewOpinion is " << sNewOpinion << endl;
  cout << "sBlock is " << sBlock << endl;
  cout << "sBlockhash is " << sBlockhash << endl;
  cout << "Registered robot" << endl;
  bwh.blockNumber = atoi(sBlock.c_str());
  bwh.hash = "\"" + sBlockhash + "\"";
  cout << "bwh.blockNumber: " << bwh.blockNumber << " bwh.hash:" << bwh.hash << endl;
}*/

/* void CBlockchainVotingController::killGethAndRemoveFolders(string bcPath, string regenFile){
  // Kill all geth processes  
  string bckiller = "bash " + bcPath + "/bckillerccall";
  Geth_Wrapper::exec(bckiller.c_str());
  // Remove blockchain folders
  string rmBlockchainData = "rm -rf " + bcPath + "*";
  Geth_Wrapper::exec(rmBlockchainData.c_str());  
  // Regenerate blockchain folders
  string regenerateFolders = "bash " + regenFile;
  Geth_Wrapper::exec(regenerateFolders.c_str());
} */

void CBlockchainVotingController::fromLoopFunctionResPrepare(){
  cout << "Called fromLoopFunctionRes" << endl;

  /*CCI_EPuckRangeAndBearingActuator::TData toSend;
  toSend[0]=5;
  toSend[1]=5;
  toSend[2]=5;
  toSend[3]=5;
  m_pcRABA->SetData(toSend);
  m_pcRABS->ClearPackets();
  //receivedOpinions.clear();*/

  cout << "ID Raw is: " << GetId() << endl;
  int robotId = Geth_Wrapper::Id2Int(GetId());

  beginning = true;
  
  // Ethereum
  //interface = Geth_Wrapper::readStringFromFile(simulationParams.baseDir + simulationParams.interfacePath);
  interface = Geth_Wrapper::readStringFromFile(simulationParams.baseDirRaw + "/Voting.abi");
  //cout << "Interface: " << interface << endl;

  /* Find out on which cluster node this robot's geth process should be executed */
  ostringstream genesisRawStream;
  genesisRawStream << simulationParams.baseDirRaw << "/genesis/genesis_template.json";
  string genesisRaw = genesisRawStream.str();
  ostringstream genesisPathStream;
  //genesisPathStream << simulationParams.baseDirRaw << "/genesis/genesis" << simulationParams.basePort  << ".json";
  genesisPathStream << simulationParams.baseDirRaw << "/genesis/genesis.json";
  string genesisPath = genesisPathStream.str();    
  
  // std::string newAcc = Geth_Wrapper::createAccountInit(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
  // string genesisTemplate = Geth_Wrapper::readAllFromFile(genesisRaw);
  // genesisTemplate = Geth_Wrapper::replaceAll(genesisTemplate, "ADDRESS", newAcc);
  // std::ofstream out(genesisPath.c_str());
  // out << genesisTemplate;
  // out.close();
  Geth_Wrapper::initGethNode(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath, genesisPath);
  coinbaseAddresses[robotId] = Geth_Wrapper::getCoinbase(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
  address = coinbaseAddresses[robotId];
}

void CBlockchainVotingController::fromLoopFunctionResStart(){
  int robotId = Geth_Wrapper::Id2Int(GetId());

  ostringstream genesisPathStream;
  //genesisPathStream << simulationParams.baseDirRaw << "/genesis/genesis" << simulationParams.basePort  << ".json";
  genesisPathStream << simulationParams.baseDirRaw << "/genesis/genesis.json";
  string genesisPath = genesisPathStream.str();
  
  enodes[robotId] = Geth_Wrapper::get_enode(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
  enode = enodes[robotId];
  Geth_Wrapper::unlockAccount(robotId, "test", nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
  //Geth_Wrapper::start_mining(robotId, 1, nodeInt, simulationParams.blockchainPath);
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CBlockchainVotingController, "blockchain_voting_controller")
