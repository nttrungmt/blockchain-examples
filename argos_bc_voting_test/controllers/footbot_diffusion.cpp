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
    //GetNodeAttribute(t_node, "profiling", profiling);
  }
  catch(CARGoSException& ex) {
    THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
  }
}

CBlockchainVotingController::CBlockchainVotingController() :
   nodeInt(0),
   m_cColor(CColor::WHITE),
   m_lStepCnt(0),
   beginning(false),
   mining(false),
   m_pcWheels (NULL),
   m_pcLEDs(NULL),
   m_fWheelVelocity (10.0f),
   m_pcRABA (NULL),
   m_pcRABS (NULL),
   m_cAlpha (10.0f),
   m_fDelta(0.5f),
   m_pcProximity(NULL),
   //m_pcLight(NULL),
   //m_pcGround(NULL),
   m_pcRNG(NULL),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
						  ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/
void CBlockchainVotingController::Init(TConfigurationNode& t_node) {
  eventTrials = 0;
  //receivedDecision = true;
  threadCurrentlyRunning = false;
  
  /* Initialize the actuators (and sensors) and the initial velocity as straight walking*/
  m_pcWheels = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
  m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
  m_pcRABA = GetActuator<CCI_EPuckRangeAndBearingActuator>("epuck_range_and_bearing");
  
  m_pcProximity = GetSensor <CCI_EPuckProximitySensor>("epuck_proximity");
  //m_pcLight     = GetSensor <CCI_FootBotLightSensor>("footbot_light");
  m_pcRABS      = GetSensor <CCI_EPuckRangeAndBearingSensor>("epuck_range_and_bearing");
  //m_pcGround    = GetSensor <CCI_FootBotMotorGroundSensor>("footbot_motor_ground" );
  //m_pcGround    = GetSensor <CCI_EPuckGroundSensor>("epuck_motor_ground" );
  m_pcRNG = CRandom::CreateRNG("argos");
  m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
  GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
  simulationParams.Init(GetNode(t_node, "simulation_parameters"));
  //simulationParams.g = simulationParams.g * 10;
  //simulationParams.sigma = simulationParams.sigma * 10;
  
  /* Init REB actuators*/
  CCI_EPuckRangeAndBearingActuator::TData toSend;
  toSend[0]=5;
  toSend[1]=5;
  toSend[2]=5;
  toSend[3]=5;
  m_pcRABA->SetData(toSend);

  //readNodeMapping();
}

/****************************************/
/****************************************/
 
void CBlockchainVotingController::UpdateState() {
   ///* Reset state flags */
   //m_sStateData.InNest = false;
   ///* Read stuff from the ground sensor */
   //const CCI_EPuckGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
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
   /*if(tGroundReads[2].Value > 0.25f &&
      tGroundReads[2].Value < 0.75f &&
      tGroundReads[3].Value > 0.25f &&
      tGroundReads[3].Value < 0.75f) {
      //m_sStateData.InNest = true;
   }*/
}
 
/****************************************/
/****************************************/
 
CVector2 CBlockchainVotingController::CalculateVectorToLight() {
   ///* Get readings from light sensor */
   //const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
   ///* Sum them together */
   //CVector2 cAccumulator;
   //for(size_t i = 0; i < tLightReads.size(); ++i) {
   //   cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   //}
   ///* If the light was perceived, return the vector */
   //if(cAccumulator.Length() > 0.0f) {
   //   return CVector2(1.0f, cAccumulator.Angle());
   //}
   ///* Otherwise, return zero */
   //else {
      return CVector2();
   //}
}

/****************************************/
/****************************************/
void CBlockchainVotingController::ControlStep() {
   m_lStepCnt++;
   
   int robotId = Geth_Wrapper::Id2Int(GetId());
   //if (!simulationParams.useClassicalApproach) {
     if (beginning) {
       // TODO: start_geth should be removed again; added it because of problem with kill geth
       //Geth_Wrapper::start_geth(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
       Geth_Wrapper::unlockAccount(robotId, "test", nodeInt, simulationParams.basePort, simulationParams.blockchainPath); // TODO: Also remove again
       //Geth_Wrapper::start_mining_bg(robotId, 1, nodeInt, simulationParams.blockchainPath);
       //registerRobot(); // TODO: remove this again, it's not in the original code but I added it due to problems with the registration
       //updateRegistration();
       //Geth_Wrapper::stop_mining_bg(robotId, nodeInt, simulationParams.blockchainPath);
       beginning = false;
     }
   //}
  
   /* Turn leds according with actualOpinion */
   TurnLeds();
   
   /* Move robots following randomWalk */
   Move();
   
   /* switch state of EXPLORING or DIFFUSING */
   /* Two different behaviours, depending on if they are diffusing or exploring */
   //switch(m_sStateData.State) {    
	//case SStateData::STATE_EXPLORING: {
		///* For a fully connected network (debugging) */
		//set<int> currentNeighbors;
        // Fully connected
		//for (UInt8 i = 0; i <= 19; i++) {
		//  currentNeighbors.insert(i);
		//}
		//if (!simulationParams.useClassicalApproach) {
			//UpdateNeighbors(currentNeighbors);
			//if (mining) {
			//	cout << " STOP MINING -- robot" << robotId << endl;
			//	mining = false;
			//	Geth_Wrapper::stop_mining_bg(robotId, nodeInt, simulationParams.blockchainPath);
			//}
		//}
		Explore();
		//break;
	//}
	//case SStateData::STATE_DIFFUSING: {  
	//	Diffusing();
	//	break;
	//}
  //}
   
   RandomWalk();
   
   /**** OBSTACLE AVOIDANCE ****/
   /* Get readings from proximity sensor and sum them together */
   const CCI_EPuckProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
     cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   if(tProxReads.size()>0)
     cAccumulator /= tProxReads.size();
	 /* If the angle of the vector is not small enough or the closest obstacle is not far enough curve a little */
	 CRadians cAngle = cAccumulator.Angle();
	 if(!(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) && cAccumulator.Length() < m_fDelta )) {
	   /* Turn, depending on the sign of the angle */
	   if(cAngle.GetValue() > 0.0f) {
		 m_pcWheels->SetLinearVelocity( m_fWheelVelocity, 0.0f);
	   }
	   else {
	  	 m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
	   }
   }
}

// Tell the smart contract the robot's public key
void CBlockchainVotingController::registerRobot() {
  int robotId = Geth_Wrapper::Id2Int(GetId());  
  //int args[1] = {(int) opinion.actualOpinion};
  int emptyArgs[0] = {};
  cout << "START now registrating robot " << robotId << endl; 
  // Modify state of the blockchain
  Geth_Wrapper::smartContractInterfaceBg(robotId, interface,
	 //contractAddress, "registerRobot", args, 1, 0, nodeInt, simulationParams.blockchainPath);
	 contractAddress, "registerRobot", emptyArgs, 1, 0, nodeInt, simulationParams.blockchainPath);
  cout << "END now registrating robot " << robotId << endl; 
}

// Wait for the first event of the smart contract
void CBlockchainVotingController::updateRegistration() {
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
}

/* Connect/disconnect Ethereum processes to each other */
void CBlockchainVotingController::UpdateNeighbors(set<int> newNeighbors) {
  set<int> neighborsToAdd;
  set<int> neighborsToRemove;

  int robotId = Geth_Wrapper::Id2Int(GetId());
  
  //TODO: check code below because of error "set_difference is not a member of std"
  // /* Old neighbors minus new neighbors = neighbors that should be removed */
  // std::set_difference(neighbors.begin(),
  		      // neighbors.end(),
  		      // newNeighbors.begin(),
  		      // newNeighbors.end(),
  		      // std::inserter(neighborsToRemove, neighborsToRemove.end()));

  // /* New neighbors minus old neighbors = neighbors that should be added */
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
}

/************************************************* EXPLORING STATE *********************************************/
/***************************************************************************************************************/
void CBlockchainVotingController::Explore() {
  m_pcRABS->ClearPackets();
  int robotId = Geth_Wrapper::Id2Int(GetId());
  //  int nodeInt = robotIdToNode[robotId];

	/*int num1 = std::rand() % 100 + 0;
	int num2 = std::rand() % 3 + 0;
	if(num1 >= 95) {
		std::cout << "RobotId: " << robotId << " num1:" << num1 << " num2:" << num2 << std::endl;
		//std::cout << "Begin submit voting transaction" << std::endl;
		string strColor;
		if(num2 == 0) 
			strColor = "RED";
		else if(num2 == 1)
			strColor = "GREEN";
		else 
			strColor = "BLUE";
		string args[1] = {strColor};
		Geth_Wrapper::unlockAccount(robotId, "test", nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
		//Geth_Wrapper::smartContractInterfaceStringBg(robotId, interface, contractAddress, "voteForCandidate", args, 1, -1, nodeInt, simulationParams.blockchainPath);
                Geth_Wrapper::smartContractInterfaceFuncScript(robotId, interface, contractAddress, "voteForCandidate", args, 1, -1, nodeInt, simulationParams.blockchainPath);
		Geth_Wrapper::start_mining(robotId, 1, nodeInt, simulationParams.blockchainPath);
		Geth_Wrapper::exec_geth_cmd_helper(robotId, "admin.sleepBlocks(5)", nodeInt, simulationParams.blockchainPath);
		Geth_Wrapper::stop_mining(robotId, nodeInt, simulationParams.blockchainPath);
	}*/

  bool bDone = false;
  CColor cColor = CColor::WHITE;
  for(size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
    if((m_cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
      // We are done 
      if(i==0)
        cColor = CColor::RED;
      else if(i==1)
        cColor = CColor::GREEN;
      else
        cColor = CColor::BLUE;
      bDone = true;
      break;
    }
  }
  setColor(cColor);  
}

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

/************************************************** RANDOM WALK ************************************************/
/***************************************************************************************************************/
void CBlockchainVotingController::RandomWalk() {
  /* walkTime represents the number of clock cycles (random number) of walk in a random direction*/
  if ( movement.walkTime == 0 )                            // Is the walkTime in that direction finished? ->
  { 				                       // -> YES: change direction//
    if ( movement.actualDirection == 0 )                  // If robot was going straight then turn standing in ->
	// -> a position for an uniformly distribuited time //
	{
	  CRange<Real> zeroOne(0.0,1.0);
	  Real p = m_pcRNG->Uniform(zeroOne);
	  p = p*simulationParams.turn;
	  Real dir = m_pcRNG->Uniform(CRange<Real>(-1.0,1.0));
	  if ( dir > 0 )
	    movement.actualDirection = 1;
	  else
	    movement.actualDirection = 2;
	  movement.walkTime = Floor(p);
	}
    else 						// The robot was turning, time to go straight for ->
	// -> an exponential period of time //
	{
	  movement.walkTime = Ceil(m_pcRNG->Exponential((Real)simulationParams.LAMDA)*4); // Exponential random generator. *50 is a scale factor for the time
	  movement.actualDirection = 0;
	}
  }
  else {							// NO: The period of time is not finished, decrement the ->
    // -> walkTime and keep the direction //
    movement.walkTime--;
  }
}

/************************************************* MOVEMENT ****************************************************/
/***************************************************************************************************************/
/* Implement the moviment leaded by the random walk (see loop_function) */
void CBlockchainVotingController::Move(){
  if(movement.actualDirection == 0) // Go straight
    m_pcWheels->SetLinearVelocity(m_fWheelVelocity,  m_fWheelVelocity);
  else
    if(movement.actualDirection == 1) // Turn right
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity,  -m_fWheelVelocity);
    else
      if(movement.actualDirection == 2) // Turn left
	m_pcWheels->SetLinearVelocity(-m_fWheelVelocity,  m_fWheelVelocity);
}

/************************************************* TURNING LEDS ON *********************************************/
/***************************************************************************************************************
0 = BLACK/EX-RED;
1 = GREEN; 
2 = WHITE/EX-BLUE
AGGIUNGERECOLORI 
*/
void CBlockchainVotingController::TurnLeds(){
   //cout << "Called TurnLeds..." << endl;
   /*switch(m_cColor) {
   case CColor::RED: {
     //opinion.actualOpCol = CColor::RED;
     m_pcLEDs->SetAllColors(CColor::RED);
     break;
   }
   case CColor::GREEN: {
     //opinion.actualOpCol = CColor::GREEN;
     m_pcLEDs->SetAllColors(CColor::GREEN);
     break;
   }
   case CColor::BLUE: {
     //opinion.actualOpCol = CColor::BLUE;
     m_pcLEDs->SetAllColors(CColor::BLUE);
     break;
   }
   default:
     m_pcLEDs->SetAllColors(CColor::WHITE);
     break;
   }*/
   m_pcLEDs->SetAllColors(m_cColor);
}

void CBlockchainVotingController::Destroy(){
  int robotId = Geth_Wrapper::Id2Int(GetId());
  //if(mining) {
  //  Geth_Wrapper::stop_mining(robotId, nodeInt, simulationParams.blockchainPath);
  //}
  Geth_Wrapper::kill_geth_thread(robotId, simulationParams.basePort, nodeInt, simulationParams.blockchainPath);
}

/****************************************/
/****************************************/
// void CBlockchainVotingController::killGethAndRemoveFolders(string bcPath, string regenFile){
  // // Kill all geth processes  
  // string bckiller = "bash " + bcPath + "/bckillerccall";
  // Geth_Wrapper::exec(bckiller.c_str());
  // // Remove blockchain folders
  // string rmBlockchainData = "rm -rf " + bcPath + "*";
  // Geth_Wrapper::exec(rmBlockchainData.c_str());  
  // // Regenerate blockchain folders
  // string regenerateFolders = "bash " + regenFile;
  // Geth_Wrapper::exec(regenerateFolders.c_str());      
// }

void CBlockchainVotingController::fromLoopFunctionResPrepare(){
  cout << "Called fromLoopFunctionRes" << endl;

  CCI_EPuckRangeAndBearingActuator::TData toSend;
  toSend[0]=5;
  toSend[1]=5;
  toSend[2]=5;
  toSend[3]=5;
  m_pcRABA->SetData(toSend);
  m_pcRABS->ClearPackets();
  //receivedOpinions.clear();

  TurnLeds();

  // /* Assign the initial state of the robots: all in exploration state*/
  // m_sStateData.State = SStateData::STATE_EXPLORING;

  // /* Assign the exploration time (random generated) */
  // m_sStateData.remainingExplorationTime = (m_pcRNG->Exponential((Real)simulationParams.sigma));
  // m_sStateData.explorDurationTime = m_sStateData.remainingExplorationTime;

  cout << "ID Raw is: " << GetId() << endl;
  int robotId = Geth_Wrapper::Id2Int(GetId());

  beginning = true;
  
  /* Ethereum */
  //if (!simulationParams.useClassicalApproach) {
    //nodeInt = robotIdToNode[robotId];       
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
    
    // Geth_Wrapper::geth_init(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath, genesisPath);
    // Geth_Wrapper::start_geth(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
    //Geth_Wrapper::createAccount(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
    Geth_Wrapper::initGethNode(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath, genesisPath);
    coinbaseAddresses[robotId] = Geth_Wrapper::getCoinbase(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
    address = coinbaseAddresses[robotId];
    //Geth_Wrapper::prepare_for_new_genesis(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
  //}
}

void CBlockchainVotingController::fromLoopFunctionResStart(){
  //if (!simulationParams.useClassicalApproach) {
    int robotId = Geth_Wrapper::Id2Int(GetId());

    ostringstream genesisPathStream;
    //genesisPathStream << simulationParams.baseDirRaw << "/genesis/genesis" << simulationParams.basePort  << ".json";
    genesisPathStream << simulationParams.baseDirRaw << "/genesis/genesis.json";
    string genesisPath = genesisPathStream.str();
    
    //Geth_Wrapper::geth_init(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath, genesisPath);
    //Geth_Wrapper::start_geth(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
    enodes[robotId] = Geth_Wrapper::get_enode(robotId, nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
    enode = enodes[robotId];
    Geth_Wrapper::unlockAccount(robotId, "test", nodeInt, simulationParams.basePort, simulationParams.blockchainPath);
    //Geth_Wrapper::start_mining(robotId, 1, nodeInt, simulationParams.blockchainPath);
    //mining = true;
  //}
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
