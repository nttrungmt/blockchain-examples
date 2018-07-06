#include "blockchain_loop_functions.h"
#include <argos3/core/utility/math/rng.h>
//#include "controllers/geth_wrapper.h" /* Use geth from C++ */
//#include "controllers/footboot_diffusion.h"
#include "../controllers/geth_wrapper.h" /* Use geth from C++ */

#include <iostream>
#include <unistd.h>
#include <time.h>
/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

using namespace std;

#define ALPHA_CHANNEL		         0
#define COLOR_STRENGHT               255
#define TOTAL_CELLS		             400
#define ENVIRONMENT_CELL_DIMENSION   1000.0f
#define N_COL					     3
#define ARENA_SIZE_X				 1.9f
#define ARENA_SIZE_Y				 1.9f

#define DEBUGLOOP true

static const int maxTime = 200; /* Maximum amount of time per robot to wait until they received their ether */
static const int maxContractAddressTrials = 300; /* Repeats getting the contract address procedure (can be TypeError at 1st*/
static const int trialsMiningNotWorking = 40; /* If after x trials the number of white votes is still zero  */

std::string contractAddress;
std::string minerAddressGlobal;
std::string interface; // Smart contract interface

/****************************************/
/****************************************/
CBlockchainVotingLoopFunctions::CBlockchainVotingLoopFunctions() :
	zeroOne(0.0f,1.0f),
	//bigRange(0.0f,30000.0f),
	//arenaSizeRangeX(0.0f, ARENA_SIZE_X),
	//arenaSizeRangeY(0.0f, ARENA_SIZE_Y),
	errorOccurred(false),
	miningNotWorkingAnymore(false),
	gethStaticErrorOccurred(false),
	incorrectParameters(false),
	m_bExperimentFinished(false)//,
	//m_pcFloor(NULL)
{
}

void CBlockchainVotingLoopFunctions::fillSettings(TConfigurationNode& tEnvironment) {
  try
    {
      /* Retrieving information about arena */
      // GetNodeAttribute(tEnvironment, "number_of_red_cells", colorOfCell[0]);
      // GetNodeAttribute(tEnvironment, "number_of_white_cells", colorOfCell[1]);
      // GetNodeAttribute(tEnvironment, "number_of_black_cells",colorOfCell[2]);
      // GetNodeAttribute(tEnvironment, "percent_red", percentageOfColors[0]);
      // GetNodeAttribute(tEnvironment, "percent_white", percentageOfColors[1]);
      // GetNodeAttribute(tEnvironment, "percent_black", percentageOfColors[2]);
      // GetNodeAttribute(tEnvironment, "using_percentage", using_percentage);
      //GetNodeAttribute(tEnvironment, "exit", exitFlag);

      /* Retrieving information about initial state of robots */
      // GetNodeAttribute(tEnvironment, "r_0", initialOpinions[0]);
      // GetNodeAttribute(tEnvironment, "w_0", initialOpinions[1]);
      // GetNodeAttribute(tEnvironment, "b_0", initialOpinions[2]);
      GetNodeAttribute(tEnvironment, "number_of_robots", n_robots);
      // GetNodeAttribute(tEnvironment, "number_of_qualities", number_of_qualities);

      /* Retrieving information about simulation paramaters */
      // GetNodeAttribute(tEnvironment, "g", g);
      // GetNodeAttribute(tEnvironment, "sigma", sigma);
      GetNodeAttribute(tEnvironment, "lamda", LAMDA);
      GetNodeAttribute(tEnvironment, "turn", turn);
      // GetNodeAttribute(tEnvironment, "decision_rule", decisionRule);
      //GetNodeAttribute(tEnvironment, "number_of_runs", number_of_runs);

      /* Retrieving information about how to catch and where to save statistics */
      // GetNodeAttribute(tEnvironment, "save_every_ticks", timeStep);
      // GetNodeAttribute(tEnvironment, "save_every_ticks_flag", everyTicksFileFlag);
      // GetNodeAttribute(tEnvironment, "save_every_run_flag", runsFileFlag);
      // GetNodeAttribute(tEnvironment, "save_every_quality_flag", qualityFileFlag);
      // GetNodeAttribute(tEnvironment, "save_every_robot_flag", oneRobotFileFlag);
      // GetNodeAttribute(tEnvironment, "save_global_stat_flag", globalStatFileFlag);
      // GetNodeAttribute(tEnvironment, "save_blockchain_flag", blockChainFileFlag);
      // GetNodeAttribute(tEnvironment, "radix", passedRadix);
      GetNodeAttribute(tEnvironment, "base_dir_loop", baseDirLoop);
      GetNodeAttribute(tEnvironment, "base_dir_raw", baseDirRaw);
      GetNodeAttribute(tEnvironment, "data_dir", dataDir);
      GetNodeAttribute(tEnvironment, "miningdiff", miningDiff);
      GetNodeAttribute(tEnvironment, "use_multiple_nodes", useMultipleNodes);
      GetNodeAttribute(tEnvironment, "miner_id", minerId);
      GetNodeAttribute(tEnvironment, "miner_node", minerNode);
      GetNodeAttribute(tEnvironment, "base_port", basePort);
      // GetNodeAttribute(tEnvironment, "num_byzantine", numByzantine);
      // GetNodeAttribute(tEnvironment, "byzantine_swarm_style", byzantineSwarmStyle);
      //GetNodeAttribute(tEnvironment, "use_classical_approach", useClassicalApproach);
      //GetNodeAttribute(tEnvironment, "subswarm_consensus", subswarmConsensus);
      GetNodeAttribute(tEnvironment, "regenerate_file", regenerateFile);
      GetNodeAttribute(tEnvironment, "blockchain_path", blockchainPath);
    }
  catch(CARGoSException& ex) {
    THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
  }
}

void CBlockchainVotingLoopFunctions::PreinitMiner() {
  /* Change mining difficulty and rebuild geth */
  ostringstream genesisRawStream;
  genesisRawStream << baseDirRaw << "/genesis/genesis1.json";
  string genesisRaw = genesisRawStream.str();
  ostringstream genesisPathStream;
  //genesisPathStream << baseDirRaw << "/genesis/genesis" << basePort << ".json";
  genesisPathStream << baseDirRaw << "/genesis/genesis.json";
  string genesisPath = genesisPathStream.str();

  // std::ostringstream fullCommandStream;
  // std::string minerAddress;
  // std::string newAcc = Geth_Wrapper::createAccountInit(robotId, nodeInt, basePort, blockchainPath);
  // string genesisTemplate = Geth_Wrapper::readAllFromFile(genesisRaw);
  // genesisTemplate = Geth_Wrapper::replaceAll(genesisTemplate, "ADDRESS", newAcc);
  // std::ofstream out(genesisPath.c_str());
  // out << genesisTemplate;
  // out.close();
  // /* Initialize the miner */
  // Geth_Wrapper::geth_init(minerId, minerNode, basePort, blockchainPath, genesisPath);
  // //cout << "[LoopFunctions::PreinitMiner]geth_init" << endl;
  // //cout << "Now Im here";
  // sleep(1);
  // Geth_Wrapper::start_geth(minerId, minerNode, basePort, blockchainPath);
  // //cout << "[LoopFunctions::PreinitMiner]start_geth" << endl;
  // //Geth_Wrapper::createAccount(minerId, minerNode, basePort, blockchainPath);
  // //cout << "[LoopFunctions::PreinitMiner]createAccount" << endl;
  
  Geth_Wrapper::initGethNode(minerId, minerNode, basePort, blockchainPath, genesisPath);
  string minerAddress = Geth_Wrapper::getCoinbase(minerId, minerNode, basePort, blockchainPath);
  minerAddressGlobal = minerAddress;
  // //cout << "[LoopFunctions::PreinitMiner]getCoinbase, minerAddress=" << minerAddress << endl;
  // //Geth_Wrapper::prepare_for_new_genesis(minerId, minerNode, basePort, blockchainPath);
  // //cout << "[LoopFunctions::PreinitMiner]prepare_for_new_genesis" << endl;
}

void CBlockchainVotingLoopFunctions::PreallocateEther() {
  ostringstream genesisBlockStream;
  genesisBlockStream << "{\n\"nonce\": \"0x0000000000000001\",\n\"mixhash\": \"0x0000000000000000000000000000000000000000000000000000000000000000\",\n\"difficulty\": \"0x1000\",\n\"alloc\": {\n";

  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
    std::string id = cController.GetId();
    int robotId = Geth_Wrapper::Id2Int(id);
    cout << "Coinbase address is " << coinbaseAddresses[robotId] << endl;
    genesisBlockStream << Geth_Wrapper::removeSpace(coinbaseAddresses[robotId]) << ": {\n\"balance\": \"100000000000000000000000\"\n},";
  }

  genesisBlockStream << Geth_Wrapper::removeSpace(minerAddressGlobal) << ": {\n\"balance\": \"100000000000000000000000\"\n}";
  genesisBlockStream << "\n},\n\"coinbase\": \"0xcbfbd4c79728b83eb7c3aa50455a78ba724c53ae\",\n\"timestamp\": \"0x00\",\n\"parentHash\": \"0x0000000000000000000000000000000000000000000000000000000000000000\",\n\"extraData\": \"0x\",\n\"gasLimit\": \"0x8000000\"\n}";

  string genesisBlock = genesisBlockStream.str();
  ostringstream genesisPathStream;
  genesisPathStream << baseDirRaw << "/genesis/genesis" << basePort  << ".json";
  string genesisPath = genesisPathStream.str();
  ofstream out(genesisPath.c_str());
  out << genesisBlock;
  out.close();
}

void CBlockchainVotingLoopFunctions::RestartGeths() {
  cout << "[LoopFunctions::RestartGeths]" << endl;
  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
    cController.fromLoopFunctionResStart();
  }
}

/* Assign a new state and a new position to the robots */
void CBlockchainVotingLoopFunctions::AssignNewStateAndPosition() {
  // CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  // for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
      // CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
      // CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
      // CQuaternion cNewOrientation = cEpuck. GetEmbodiedEntity().GetOriginAnchor().Orientation;

      // /* Generating Uniformly distribuited x and y coordinates for the new position of the robot */
      // Real xp = m_pcRNG->Uniform(arenaSizeRangeX);
      // Real yp = m_pcRNG->Uniform(arenaSizeRangeY);
      // CVector3 cNewPosition = CVector3(xp,yp,0.1f);

      // UInt32 i, unMaxRepositioningTrials = 100;

      // for(i = 0; i < unMaxRepositioningTrials; i++) {
		// if(cEpuck.GetEmbodiedEntity().MoveTo(cNewPosition, cNewOrientation, false)) break;
		// xp = m_pcRNG->Uniform(arenaSizeRangeX);
		// yp = m_pcRNG->Uniform(arenaSizeRangeY);
		// cNewPosition.SetX(xp);
		// cNewPosition.SetY(yp);
      // }

      // cController.GetReceivedOpinions().clear();
      // /* Resetting initial state of the robots: exploring for everyone */
      // cController.GetStateData().State = CBlockchainVotingController::SStateData::STATE_EXPLORING;
      // cController.GetStateData().remainingExplorationTime = (m_pcRNG->Exponential((Real)sigma));
      // cController.GetStateData().explorDurationTime =  cController.GetStateData().remainingExplorationTime;
    // }
}

/* Set up the miner, deploy the smart contract, etc. */
void CBlockchainVotingLoopFunctions::InitEthereum() {
  ostringstream genesisRawStream;
  genesisRawStream << baseDirRaw << "/genesis/genesis1.json";
  string genesisRaw = genesisRawStream.str();

  ostringstream genesisPathStream;
  genesisPathStream << baseDirRaw << "/genesis/genesis" << basePort  << ".json";
  string genesisPath = genesisPathStream.str();

  std::ostringstream fullCommandStream;
  std::string minerAddress;

  /* Start geth again after the preallocation */
  cout << "[LoopFunctions::InitEthereum]unlockAccount and begin deploy contract" << endl;
  //Geth_Wrapper::geth_init(minerId, minerNode, basePort, blockchainPath, genesisPath);
  //sleep(1);
  //Geth_Wrapper::start_geth(minerId, minerNode, basePort, blockchainPath);
  string command = "mkdir " + blockchainPath;
  system(command.c_str());
  Geth_Wrapper::unlockAccount(minerId, "test", minerNode, basePort, blockchainPath);
  minerAddress = Geth_Wrapper::getCoinbase(minerId, minerNode, basePort, blockchainPath);
  Geth_Wrapper::start_mining(minerId, 4, minerNode, blockchainPath);

  /* Deploy contract */
  cout << "[LoopFunctions::InitEthereum]deploy_contract" << endl;
  //string interfacePath = baseDirLoop + "interface.txt";
  //interface = Geth_Wrapper::readStringFromFile(interfacePath);
  //string dataPath = baseDirLoop + "data.txt";
  //string templatePath = baseDirLoop + "contractTemplate.txt";
  string scriptFilePath = baseDirRaw + "/voting.js";
  string txHash;
  //txHash = Geth_Wrapper::deploy_contract(minerId, interfacePath, dataPath, templatePath, minerNode, blockchainPath);
  txHash = Geth_Wrapper::deploy_contract_script(minerId, minerNode, blockchainPath, scriptFilePath);
  int u = 0;
  do {
    contractAddress = Geth_Wrapper::getContractAddress(minerId, txHash, minerNode, blockchainPath);
    //if (DEBUGLOOP) {
      if (contractAddress.find("TypeError") == 0)
		cout << "Contract address not yet available. Number of trials is " << u << endl;
      else {
		cout << "Address of deployed contract is " << contractAddress << endl;
		break;
	  }
    //}
    u++;
  } while (u < maxContractAddressTrials && contractAddress.find("TypeError") == 0);

  /* Remove space in contract address */
  contractAddress.erase(std::remove(contractAddress.begin(), contractAddress.end(), '\n'), contractAddress.end());

  /* Set the address of the deployed contract in each robot */
  setContractAddressAndDistributeEther(contractAddress, minerAddress);
  // int l1 = Geth_Wrapper::getBlockChainLength(minerId, minerNode, blockchainPath);
  // registerAllRobots();
  // int l2;
  // do {
    // l2 = Geth_Wrapper::getBlockChainLength(minerId, minerNode, blockchainPath);
    // cout << "Checking BC length" << endl;
    // sleep(1);
    // } while (l2 < (l1 + 3));
  Geth_Wrapper::stop_mining(minerId, minerNode, blockchainPath);
  
  // bool etherReceived;
  // for (int t = 0; t < maxTime; ++t) {
    // etherReceived = CheckEtherReceived();
    // /* Check if an error in the geths call occurred  */
    // IsExperimentFinished();
    // //if (DEBUGLOOP)
    // //  cout << "time step of CheckEtherReceived is " << t << " and result is " << etherReceived << std::endl;
    // if (etherReceived) {
      // break;
    // } else {
      // /* Wait a bit, maybe send ether again, and connect to miner again */
      // sleep(3);
      // if (t % 10 == 0)
		// setContractAddressAndDistributeEther(contractAddress, minerAddress);
      // if (t == maxTime - 1) {
		// errorOccurred = true;
		// IsExperimentFinished();
      // }
    // }
  // }

  //cout << "Waiting until all robots have the same blockchain" << endl;
  ///* Wait until all robots have the same blockchain */
  //bool allSameHeight = allSameBCHeight();
  ///* Check that all robots received their ether */
  ////vector<int> allRobotIds = getAllRobotIds();

  // int trialssameheight = 0;
  // while (! allSameHeight) {
    // // Check if an error in the geths call occurred
    // IsExperimentFinished();
    // // Connect again
    // if (trialssameheight % 5 == 0) {
      // connectMinerToEveryone();
      // cout << "Troubles in getting same BC height; connect miner to everyone now" << endl;
    // }
    // allSameHeight = allSameBCHeight();
    // if (trialssameheight > 40) {
      // errorOccurred = true;
      // cout << "Fatal Error: more than 30 trials in allSameBCHeight" << endl;
      // IsExperimentFinished();
    // }
    // trialssameheight++;
  // }

  // cout << "Disconnecting everyone by killing mining thread" << endl;
  // Geth_Wrapper::kill_geth_thread(minerId, basePort, minerNode, blockchainPath);
  // // And a second time (since ssh creates two processes)
  // Geth_Wrapper::kill_geth_thread(minerId, basePort, minerNode, blockchainPath);
  // // Remove folder (to be really sure)
  // std::ostringstream rmMinerStream;
  // rmMinerStream << "rm -rf " << blockchainPath << minerId;
  // std::string rmMinerCmd = rmMinerStream.str();
  // system(rmMinerCmd.c_str());
}

void CBlockchainVotingLoopFunctions::setContractAddressAndDistributeEther(string contractAddress, string minerAddress) {
  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
    /* Get handle to e-puck entity and controller */
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());

    std::string& address = cController.GetAddress();
    std::string id = cController.GetId();
    int robotId = Geth_Wrapper::Id2Int(id);

    /* Set the smart contract address for the robot */
    cController.setContractAddress(contractAddress);
    //string e = cController.getEnode();
    //Geth_Wrapper::add_peer(minerId, e, minerNode, basePort, blockchainPath);
	string e = Geth_Wrapper::get_enode(minerId, minerNode, basePort, blockchainPath);
	Geth_Wrapper::add_peer(robotId, e, cController.getNodeInt(), basePort, blockchainPath);
  }
}

bool CBlockchainVotingLoopFunctions::allSameBCHeight() {
  //int maxChecks = 11;
  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  int s, a, b;
  bool canExit = false;
  bool success = true;
  b = -1;
  cout << "Checking if all robots have the same blockchain height" << endl;
  s = 0;
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
    std::string id = cController.GetId();
    int robotNodeInt = cController.getNodeInt();
    int robotId = Geth_Wrapper::Id2Int(id);
    
    /* Check one robot (the first one) */
    if (s == 0) {
      a = Geth_Wrapper::getBlockChainLength(robotId, robotNodeInt, blockchainPath);
    } else {
      b = Geth_Wrapper::getBlockChainLength(robotId, robotNodeInt, blockchainPath);
      if (abs(a - b) > 1) {
	    cout << "a is " << a << " and b is " << b << endl;
		canExit = false;
		success = false;
		break;
      }
    }
    cout << "a is " << a << " and b is " << b << endl;
    s++;
  }
  return success;
}

bool CBlockchainVotingLoopFunctions::CheckEtherReceived() {
  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  bool everyoneReceivedSomething = true;
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin(); it != m_cEpuck.end(); ++it) {
    /* Get handle to e-puck entity and controller */
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
    std::string id = cController.GetId();
    int robotId = Geth_Wrapper::Id2Int(id);
    int robotNodeInt = cController.getNodeInt();
    long long balance;
    balance = Geth_Wrapper::check_balance(robotId, robotNodeInt, blockchainPath);
    //if (DEBUGLOOP)
      cout << "Checking account balance. It is " << balance << std::endl;
    if (balance == 0) {
      everyoneReceivedSomething = false; /* At least one robot did not receive ether */
      break;
    }
  }
  return everyoneReceivedSomething;
}

void CBlockchainVotingLoopFunctions::registerAllRobots() {
  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
    /* Get handle to e-puck entity and controller */
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
    cController.registerRobot();
  }
}

void CBlockchainVotingLoopFunctions::UpdateRegistrationAllRobots() {
  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
    /* Get handle to e-puck entity and controller */
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
    cController.updateRegistration();
  }
}

void CBlockchainVotingLoopFunctions::connectMinerToEveryone() {
  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
    /* Get handle to e-puck entity and controller */
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
    std::string& address = cController.GetAddress();
    std::string id = cController.GetId();
    int robotId = Geth_Wrapper::Id2Int(id);
    /* Make sure that the robot is connected */
    string e = cController.getEnode();    
    cout << "enode in connecttominer is" << e << endl;
    Geth_Wrapper::add_peer(minerId, e, minerNode, basePort, blockchainPath);
  }
}

bool CBlockchainVotingLoopFunctions::InitRobots() {
  cout  << "[CBlockchainVotingLoopFunctions::InitRobots]Initializing loop function" << endl;
  miningNotWorkingAnymore = false;
  errorOccurred = false;
  //if (!useClassicalApproach) {
    /* Preallocate money to the miner */
    PreinitMiner();
  //}
  // int temp1;
  // /* Mix the colours in the vector of cells to avoid the problem of eventual correlations*/
  // for (int k = 0; k < 8; k++){
    // for (int i = TOTAL_CELLS-1; i >= 0; --i){
      // int j = ((int)m_pcRNG->Uniform(bigRange)%(i+1));
      // temp1 = grid[i];
      // grid[i] = grid[j];
      // grid[j] = temp1;
    // }
  // }

  /* Variable i is used to check the vector with the mixed opinion to assign a new opinion to every robots*/
  int i = 0;
  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
    /* Get handle to e-puck entity and controller */
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
    // CBlockchainVotingController::Opinion& opinion = cController.GetOpinion();
    // CBlockchainVotingController::CollectedData& collectedData = cController.GetColData();

    // /* Resetting initial state of the robots: exploring for everyone */
    // cController.GetReceivedOpinions().clear();
    // cController.GetStateData().State = CBlockchainVotingController::SStateData::STATE_EXPLORING;
    // Real tmpValue = (m_pcRNG->Exponential((Real)sigma));
    // cController.GetStateData().remainingExplorationTime = tmpValue;
    // cController.GetStateData().explorDurationTime =  cController.GetStateData().remainingExplorationTime;

    // /* Assign a random actual opinion using the shuffled vector */
    // opinion.actualOpinion = opinionsToAssign[i];
    // i++;

    // opinion.countedCellOfActualOpinion = 0;
    // collectedData.count = 1;
    // if(opinion.actualOpinion == 1)
      // opinion.actualOpCol = CColor::WHITE;
    // if(opinion.actualOpinion == 2)
      // opinion.actualOpCol = CColor::BLACK;
    // /* Setting robots initial states: exploring state */

    cController.fromLoopFunctionResPrepare();

    // if( gethStaticErrorOccurred ) {
      // cout << "gethStaticErrorOccurred was true in InitRobots" << endl;
      // Reset();
      // cout << "Finished Reset, returning true now" << endl;
      // return true;
    // }
  }

  //if (!useClassicalApproach) {
    //PreallocateEther();
    RestartGeths();
  //}

  AssignNewStateAndPosition();

  //if (!useClassicalApproach) {
    /* Initialize miner, distribute ether, and more */
    InitEthereum();
  //}    
}

void CBlockchainVotingLoopFunctions::Init(TConfigurationNode& t_node) {
   TConfigurationNode& tEnvironment = GetNode(t_node, "cells");
   fillSettings(tEnvironment);
   
   m_pcRNG = CRandom::CreateRNG("argos");
   
   InitRobots();
   
   // /*
    // * Go through all the robots in the environment
    // * and create an entry in the waypoint map for each of them
    // */
   // /* Get the map of all foot-bots from the space */
   // CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("epuck");
   // /* Go through them */
   // for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       // it != tFBMap.end();
       // ++it) {
      // /* Create a pointer to the current foot-bot */
      // CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      // /* Create a waypoint vector */
      // m_tWaypoints[pcFB] = std::vector<CVector3>();
      // /* Add the initial position of the foot-bot */
      // m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   // }
}

/****************************************/
/****************************************/

void CBlockchainVotingLoopFunctions::Reset() {
  gethStaticErrorOccurred = false;
  /* Clean up Ethereum stuff  */
  //if (!useClassicalApproach) {
    // Kill all geth processes
    string bckiller = "bash " + blockchainPath + "/bckillerccall";
    Geth_Wrapper::exec(bckiller.c_str());    
    // Remove blockchain folders
    string rmBlockchainData = "rm -rf " + blockchainPath + "*";
    Geth_Wrapper::exec(rmBlockchainData.c_str());    
    // Regenerate blockchain folders
    string regenerateFolders = "bash " + regenerateFile;
    Geth_Wrapper::exec(regenerateFolders.c_str());    
  //}
  InitRobots();
}

/****************************************/
/****************************************/
bool CBlockchainVotingLoopFunctions::IsExperimentFinished() {
  /* If parameters are uncorrect then finish the experiment (Eg: number of robots vs sum of the initial opinion,
   * or the colours of the cells mismatching */
  if( incorrectParameters ) {
    cout << "incorrectParameters was true" << endl;
    Reset();
    return true;
  }

  if ( miningNotWorkingAnymore ) {
    // system("echo \"true\" > regeneratedag.txt"); // set flag that the DAG should be regenerated
    cout << "mininNotWorkingAnymore was true" << endl;
    Reset();
    return true;
  }

  if( errorOccurred ) {
    cout << "errorOccured was true" << endl;
    Reset();
    return true;
  }

  if( gethStaticErrorOccurred ) {
    cout << "gethStaticErrorOccurred was true" << endl;
    Reset();
    return true;
  }
}

void CBlockchainVotingLoopFunctions::Destroy(){
  //Clean up Ethereum stuff
  //if (!useClassicalApproach) {
      // Kill all geth processes
      //Geth_Wrapper::kill_geth_thread(minerId, minerNode, basePort, blockchainPath);
      Geth_Wrapper::kill_geth_thread(minerId, basePort, minerNode, blockchainPath);
      string bckiller = "bash " + blockchainPath + "/bckillerccall";
      Geth_Wrapper::exec(bckiller.c_str());
      // Remove blockchain folders
      string rmBlockchainData = "rm -rf " + blockchainPath + "*";
      Geth_Wrapper::exec(rmBlockchainData.c_str());
  //}
}

void CBlockchainVotingLoopFunctions::PreStep() {
  CSpace::TMapPerType& m_cEpuck = GetSpace().GetEntitiesByType("epuck");
  for(CSpace::TMapPerType::iterator it = m_cEpuck.begin();it != m_cEpuck.end();++it){
    /* Get handle to e-puck entity and controller */
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);    
    CBlockchainVotingController& cController =  dynamic_cast<CBlockchainVotingController&>(cEpuck.GetControllableEntity().GetController());
    
    Real x = cEpuck. GetEmbodiedEntity().GetOriginAnchor().Position.GetX(); // X coordinate of the robot
    Real y = cEpuck. GetEmbodiedEntity().GetOriginAnchor().Position.GetY(); // Y coordinate of the robot
    
    CVector2 cPos;
    cPos.Set(x,y);	// Vector position of the robot
    /* Figure out in which cell (EG: which is the index of the array grid) the robot is */
    UInt32 cell = (UInt32) ((y+0.009)*10000)/(Real)ENVIRONMENT_CELL_DIMENSION;
    cell = (UInt32) 40*cell + ((x+0.009)*10000)/(Real)ENVIRONMENT_CELL_DIMENSION;
    
    /* Get parameters of the robot: color, state, opinion and movement datas*/
    //CBlockchainVotingController::CollectedData& collectedData = cController.GetColData();
    //CBlockchainVotingController::SStateData& sStateData = cController.GetStateData();
    CBlockchainVotingController::Movement& movement = cController.GetMovement();
    //CBlockchainVotingController::Opinion& opinion = cController.GetOpinion();
    std::string id = cController.GetId();
    //CBlockchainVotingController::SimulationState& simulationParam = cController.GetSimulationState();
    
    /* Update statistics about the robot opinions*/
    //bool isByzantine = (bool) cController.getByzantineStyle();
    //UpdateStatistics(opinion, sStateData, isByzantine);
    //if(cController.IsExploring())
    //  UpdateCount(collectedData, cell, cPos, opinion, sStateData, id, simulationParam);
    RandomWalk(movement);
  }
}

void CBlockchainVotingLoopFunctions::PostStep() {
   // /* Get the map of all foot-bots from the space */
   // CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   // /* Go through them */
   // for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       // it != tFBMap.end();
       // ++it) {
      // /* Create a pointer to the current foot-bot */
      // CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      // /* Add the current position of the foot-bot if it's sufficiently far from the last */
      // if(SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
                        // m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
         // m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
      // }
   // }
}

/* Implement random walk */
void CBlockchainVotingLoopFunctions::RandomWalk(CBlockchainVotingController::Movement& movement) {
  /* walkTime represents the number of clock cycles (random number) of walk in a random direction*/
  if ( movement.walkTime == 0 )                            // Is the walkTime in that direction finished? ->
    { 				                                         // -> YES: change direction//
      if ( movement.actualDirection == 0 )                  // If robot was going straight then turn standing in ->
	// -> a position for an uniformly distribuited time //
	{
	  Real p = m_pcRNG->Uniform(zeroOne);
	  p = p*turn - (turn/2);
	  if ( p > 0 )
	    movement.actualDirection = 1;
	  else
	    movement.actualDirection = 2;
	  movement.walkTime = (UInt32) abs(p);
	}
      else 						// The robot was turning, time to go straight for ->
	// -> an exponential period of time //
	{
	  movement.walkTime = (m_pcRNG->Exponential((Real)LAMDA))*4; // Exponential random generator. *50 is a scale factor for the time
	  movement.actualDirection = 0;
	}
    }
  else 							// NO: The period of time is not finished, decrement the ->
    // -> walkTime and keep the direction //
    movement.walkTime--;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CBlockchainVotingLoopFunctions, "blockchain_voting_loop_functions")
