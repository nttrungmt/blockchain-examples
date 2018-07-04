#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
//#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <stddef.h>

#include "../controllers/footbot_diffusion.h"

using namespace argos;
using namespace std;

extern std::map<int, std::string> coinbaseAddresses;

class CBlockchainVotingLoopFunctions : public CLoopFunctions {
public:
   //typedef std::map<CFootBotEntity*, std::vector<CVector3> > TWaypointMap;
   //TWaypointMap m_tWaypoints;
   
public:
   CBlockchainVotingLoopFunctions();
   virtual ~CBlockchainVotingLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual bool InitRobots();
   virtual bool IsExperimentFinished();
   virtual void Reset();
   virtual void Destroy();
   virtual void PreStep();
   virtual void PostStep();

   //inline const TWaypointMap& GetWaypoints() const {
   //   return m_tWaypoints;
   //}

private:
	void fillSettings(TConfigurationNode& tEnvironment);
	void PreinitMiner();
	void PreallocateEther();
	void RestartGeths();
	void AssignNewStateAndPosition();
	void InitEthereum();
	void RandomWalk(CBlockchainVotingController::Movement& movement);
	
	void setContractAddressAndDistributeEther(string contractAddress, string minerAddress);
	bool allSameBCHeight();
	bool CheckEtherReceived();
	void registerAllRobots();
	void UpdateRegistrationAllRobots();
	void connectMinerToEveryone();
	
	CRange<Real> zeroOne;
	CRandom::CRNG* m_pcRNG;
	bool errorOccurred;
	bool miningNotWorkingAnymore;
	bool gethStaticErrorOccurred;
	bool incorrectParameters;
	bool m_bExperimentFinished;
	
	UInt32 LAMDA,turn; // Parameters for the randomWalk: Lamda is the exponential mean and turn is the uniform parameter
	
	UInt32 n_robots;  /* Swarm Size */
	//UInt32 g;
	//UInt32 sigma;
	//UInt32 decisionRule;
	UInt32 miningDiff;
	int minerId;
	int minerNode;
	std::string blockchainPath;
	std::string regenerateFile;
	std::string baseDirLoop;
	std::string baseDirRaw;
	std::string dataDir;
	std::string datadirBase;
	bool useMultipleNodes;
	int basePort;
	//int numByzantine;
	//int byzantineSwarmStyle;
	//bool useClassicalApproach;
	//bool subswarmConsensus;
};

#endif
