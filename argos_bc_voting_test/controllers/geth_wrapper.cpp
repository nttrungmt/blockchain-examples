#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdexcept>
#include <string>
#include <sstream>
#include <fstream>
#include <limits>
#include <algorithm>

#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "geth_wrapper.h"

#define DEBUG true
#define USE_MULTIPLE_NODES true

using namespace std;

string Geth_Wrapper::datadir_base =  "~/Documents/eth_data/data";
const string Geth_Wrapper::genesis = "~/genesis/genesis1.json ";
const string Geth_Wrapper::genesisTemplate = "~/genesis/genesis_template.json ";
const int Geth_Wrapper::rpc_base_port = 8300;
const int Geth_Wrapper::ipc_base_port = 33500;
const int Geth_Wrapper::maxtrials = 3000;
const string Geth_Wrapper::rack = "3";
bool Geth_Wrapper::gethStaticErrorOccurred = false;

void Geth_Wrapper::geth_init(int i, int nodeInt, int basePort, string datadirBase, string genesisPath) {
  cout << "DEBUG-- geth_init for robot " << i << endl;

  std::ostringstream datadirStream;
  datadirStream << datadirBase << i;
  
  string str_datadir = datadirStream.str();
  ostringstream fullCommandStream;

  fullCommandStream << "geth" << nodeInt << " --verbosity 1" << " --datadir " << str_datadir << " init " << genesisPath;
  
  string commandStream = fullCommandStream.str();

  if (true)
    cout << "geth init: " << commandStream << endl; 
  
  exec(commandStream.c_str());

  sleep(5);
}

void Geth_Wrapper::start_geth(int i, int nodeInt, int basePort, string datadirBase){
  sleep(1);
  cout << "DEBUG--  start_geth for robot " << i << endl;

  ostringstream fullCommandStream;
  
  /* Run geth command on this node  */
  //string username = getUsername();
  fullCommandStream << "geth" << nodeInt << " --verbosity 1 --networkid 2 --nodiscover ";
  
  std::ostringstream datadirStream;
  datadirStream << datadirBase << i << + "/";
  
  string str_datadir = datadirStream.str();
  ostringstream portStream;

  portStream << " --port " << (basePort + i);
  string str_port = portStream.str();

  // Make folders
  string mk_folders = "mkdir -p " + str_datadir;  
  system(mk_folders.c_str());

  // Compose geth command
  string ipc_path = "--ipcpath " + str_datadir + "geth.ipc";
  fullCommandStream << ipc_path << " --datadir " << str_datadir << str_port << " --maxpeers 130" << "&";
  cout << "Running command " << fullCommandStream.str() << endl;
  
  FILE* pipe = popen(fullCommandStream.str().c_str(), "r");
  pclose(pipe);

  string IPCfile = str_datadir + "geth.ipc";
 
  struct stat buffer;   

  sleep(5); // TODO: rather check for the IPC path instead  
}

void Geth_Wrapper::createAccount(int i, int nodeInt, int basePort, string datadirBase) {
  sleep(1);
  string cmd = "personal.newAccount(\"test\")";
  cout << "DEBUG-- createAccount: " << cmd << std::endl;
  string res = exec_geth_cmd_with_geth_restart(i, cmd, nodeInt, basePort, datadirBase);
}

// Take a geth command, execute it on the selected robot, and return the result string
std::string Geth_Wrapper::createAccountInit(int i, int nodeInt, int basePort, string datadirBase){
  ostringstream fullCommandStream;
  fullCommandStream << "geth" <<  nodeInt << " --datadir " <<  datadirBase << i << " --password ./genesis/password.txt account new";
  std::string fullCommand = fullCommandStream.str();
  string res = exec(fullCommand.c_str()); 
  size_t t1 = res.find("{");
  size_t t2 = res.find("}"); 
  res = "0x" + res.substr(t1+1,t2-t1-1);
  cout << "createAccountInit: " << fullCommand << ", res:" << res << endl;
  return res;  
}

/* Unlock account */
string Geth_Wrapper::unlockAccount(int i, string pw, int nodeInt, int basePort, string datadirBase) {
  ostringstream fullCommandStream;

  fullCommandStream << "personal.unlockAccount(eth.coinbase, \"" << pw << "\", 0)";
  
  string cmd = fullCommandStream.str();
  string res = exec_geth_cmd_with_geth_restart(i, cmd, nodeInt, basePort, datadirBase);

  if (DEBUG)
    cout << "DEBUG -- unlockAccount: " << res << endl;
    
  return res;
}

std::string Geth_Wrapper::setEtherbase(int i, int nodeInt, int basePort, string datadirBase) {
  std::ostringstream fullCommandStream;
  fullCommandStream << "miner.setEtherbase(eth.accounts[0])";
  
  string cmd = fullCommandStream.str();
  string res = exec_geth_cmd_with_geth_restart(i, cmd, nodeInt, basePort, datadirBase);

  if (DEBUG)
    cout << "DEBUG -- setEtherbase: " << res << endl;
    
  return res;
}

/* Deploy contract using robot number i and return the transaction hash */
string Geth_Wrapper::deploy_contract(int i, string interfacePath, string dataPath, string templatePath, int nodeInt, string datadirBase) {
  // Get smart contract interface
  string interface = readStringFromFile(interfacePath);
  string data = readStringFromFile(dataPath);
  data = "0x" + data; // Make data hexadecimal
  string contractTemplate = readStringFromFile(templatePath);

  replace(contractTemplate, "INTERFACE", interface);
  replace(contractTemplate, "DATA", data);

  string tmpPath = datadirBase + "/tmp.txt";

  ofstream out(tmpPath.c_str());
  out << contractTemplate;
  out.close();

  cout << contractTemplate << endl;
  cout << "Current balance: " << check_balance(i, nodeInt, datadirBase) << endl;
  cout << "Current ether: " << check_ether(i, nodeInt, datadirBase) << endl;

  for (int trials = 0; trials < maxtrials; ++trials) {
    string txHashRaw = exec_geth_cmd(i, "loadScript(\"" + tmpPath + "\")", nodeInt, datadirBase);
    if (DEBUG)
      cout << "DEBUG -- deploy_contract: trial " << trials << endl;
    cout << "txHashRaw: " << txHashRaw << endl; 
    string txHash;
    istringstream f(txHashRaw);
    getline(f, txHash);

    cout << "txHash: " << txHash << endl; 

    /* If a transaction hash was generated, i.e., neither true nor false nor Error were found */
    if (txHash.find("true") == string::npos && txHash.find("false") == string::npos 
	    && txHash.find("Error") == string::npos && txHash.find("Fatal") == string::npos) {
	  return txHash;
    }
  }  

  /* If the maximum number of trials is reached */
  cout << "Maximum number of trials is reached!" << endl;

  //string bckiller = "bash " + datadirBase + "/bckillerccall";
  //exec(bckiller.c_str());    
  
  //throw;
  gethStaticErrorOccurred = true;
}

// Get contract address from transaction receipt
string Geth_Wrapper::getContractAddress(int i, string txHash, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  //cout << "txHash is: " << txHash << endl;  
  fullCommandStream << "eth.getTransactionReceipt(\"" << txHash << "\").contractAddress";  
  string cmd = fullCommandStream.str();
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);
  //  if (DEBUG)
    cout << "DEBUG -- getContractAddress: txtHash=" << txHash << ", result=" << res << endl;
    
  return res;
}

string Geth_Wrapper::get_enode(int i, int nodeInt, int basePort, string datadirBase) {
  string cmd = "admin.nodeInfo.enode";
  string res = exec_geth_cmd_with_geth_restart(i, cmd, nodeInt, basePort, datadirBase);
  // Print the received enode
  cout << "DEBUG -- get_enode: enode=" << res << endl;

  return res;  
}

/* Start the mining process for robot i using t threads */
string Geth_Wrapper::start_mining(int i, int t, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "miner.start(" << t << ")";
  string cmd = fullCommandStream.str();
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);
  return res;
}

/* Start the mining process for robot i using t threads */
void Geth_Wrapper::start_mining_bg(int i, int t, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "miner.start(" << t << ")";
  string cmd = fullCommandStream.str();
  exec_geth_cmd_background(i, cmd, nodeInt, datadirBase);
}

/* Stop the mining process for robot i */
string Geth_Wrapper::stop_mining(int i, int nodeInt, string datadirBase) {
  string cmd = "miner.stop()";
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);
  return res;
}

/* Stop the mining process for robot i */
void Geth_Wrapper::stop_mining_bg(int i, int nodeInt, string datadirBase) {
  string cmd = "miner.stop()";
  exec_geth_cmd_background(i, cmd, nodeInt, datadirBase);
}

// Add enode to robot i
string Geth_Wrapper::add_peer(int i, string enode, int nodeInt, int basePort, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "admin.addPeer(" << enode << ")";
  string cmd = fullCommandStream.str();
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);

  return res;
}

// Add enode to robot i
void Geth_Wrapper::add_peer_bg(int i, string enode, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "admin.addPeer(" << enode << ")";
  string cmd = fullCommandStream.str();
  exec_geth_cmd_background(i, cmd, nodeInt, datadirBase);
}

// Remove enode from robot i
string Geth_Wrapper::remove_peer(int i, string enode, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "admin.removePeer(" << enode << ")";
  string cmd = fullCommandStream.str();
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);

  return res;
}

// Remove enode from robot i
void Geth_Wrapper::remove_peer_bg(int i, string enode, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "admin.removePeer(" << enode << ")";
  string cmd = fullCommandStream.str();
  exec_geth_cmd_background(i, cmd, nodeInt, datadirBase);
}

// Get coinbase address of robot i
string Geth_Wrapper::getCoinbase(int i, int nodeInt, int basePort, string datadirBase){
  ostringstream fullCommandStream;
  fullCommandStream << "eth.coinbase";
  string cmd = fullCommandStream.str();
  string res = exec_geth_cmd_with_geth_restart(i, cmd, nodeInt, basePort, datadirBase);

  if (DEBUG)
    cout << "DEBUG  -- getCoinbase " << "(robot " << i << "): " << res << endl;
  
  return res;
}

// Send v ether from robot i to address to
string Geth_Wrapper::sendEther(int i, string from, string to, int v, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "eth.sendTransaction({from:"  << from << ",to:" << to << ", value: web3.toWei(" << v << ", \"ether\")})";
  string cmd = fullCommandStream.str();
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);

  if (DEBUG)
    cout << "DEBUG -- sendEther (from " << i << " to " << to << "): " << res << endl;
    
  return res;
}

string Geth_Wrapper::kill_geth_thread(int i) {
  //string username = getUsername();
  int port = ipc_base_port + i;

  ostringstream fullCommandStream;
  if (USE_MULTIPLE_NODES) {
    /* Run geth command on this node  */
    fullCommandStream << "ps ax | grep \\\"\\-\\-port " << port << "\\";
    fullCommandStream << "\"";
  } else {
    fullCommandStream << "ps ax | grep \"\\-\\-port " << port << "\"";
  }

  string cmd = fullCommandStream.str();
  string res = exec(cmd.c_str());
  if (DEBUG)
    cout << "DEBUG -- kill_geth_thread -- command:" << cmd << " result: " << res << endl;

  /* Only get the first word, i.e., the PID from the command */
  istringstream iss(res);
  string pid;
  iss >> pid;

  string cmd2;
  if (USE_MULTIPLE_NODES) {
    ostringstream fullCommandStream2;
    fullCommandStream2 << "kill -HUP " << pid;
    cmd2 = fullCommandStream2.str();
  } else {
	cmd2 = "kill -HUP " + pid;
  }

  res = exec(cmd2.c_str());
  
  if (DEBUG)
    cout << "DEBUG -- kill_geth_thread: " << res << endl;  
  
  return res;
}

void Geth_Wrapper::kill_geth_thread(int i, int basePort, int nodeInt, string datadirBase) {
  int port = basePort + i;

  string mykill = "killall geth0";
  system(mykill.c_str());
  
  ostringstream fullCommandStream;

  /* Run geth command on this node  */
  //string username = getUsername();
  fullCommandStream << "ps ax | grep \"port " << port << "\"";

  cout << "the full command stream for grep is: \n" << fullCommandStream.str();
  
  string cmd = fullCommandStream.str();
  string res = exec(cmd.c_str());

  cout << "The grep results is:\n" << res << endl;
  
  /* Only get the first word, i.e., the PID from the command */
  istringstream iss(res);
  string pid;

  iss >> pid;

  string cmd2;
  ostringstream fullCommandStream2;
  fullCommandStream2 << "kill -HUP " << pid;
  cmd2 = fullCommandStream2.str();

  cout << "the full command stream for kill is: \n" << fullCommandStream2.str();
  
  system(cmd2.c_str());
  
  /* Kill again locally */
  string pid2;
  string cmd3;
  ostringstream fullCommandStream3;
  iss.ignore(numeric_limits<streamsize>::max(), '\n');
  iss >> pid2;
  fullCommandStream3 << "kill -9 " << pid2;
  cmd3 = fullCommandStream3.str();

  system(cmd3.c_str());

  /* And again via SSH */
  string cmd4;
  ostringstream fullCommandStream4;
  fullCommandStream4 << "kill -HUP " << pid2;
  cmd4 = fullCommandStream4.str();
  system(cmd4.c_str());  
}

string Geth_Wrapper::eventInterface(int i, string interface, string contractAddress, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "var cC = web3.eth.contract(" << interface << ");var c = cC.at(" << contractAddress << ");var ev = c.strategyApplied({sender:eth.coinbase},{fromBlock: 0, toBlock: \"latest\"});var allStratEvents = ev.get();allStratEvents[allStratEvents.length - 1].args;console.log(allStratEvents[allStratEvents.length - 1].args.blockHash, allStratEvents[allStratEvents.length - 1].args.blockNumber, allStratEvents[allStratEvents.length - 1].args.opinion);";

  string fullCommand = fullCommandStream.str();

  string res = exec_geth_cmd(i, fullCommand, nodeInt, datadirBase);
  cout << "DEBUG-- eventInterface: result=" << res << endl;

  return res;
}

// Interact with a function of a smart contract
// v: Amount of wei to send
string Geth_Wrapper::smartContractInterface(int i, string interface, string contractAddress,
				   string func, int args[], int argc, int v, int nodeInt, string datadirBase) {  
  ostringstream fullCommandStream;
  fullCommandStream << "var cC = web3.eth.contract(" << interface << ");var c = cC.at(" << contractAddress << ");c." << func << "(";
  for(int k = 0; k < argc; k++) {
    fullCommandStream << args[k] << ",";  
  }
  fullCommandStream << "{" << "value: " << v << ", from: eth.coinbase, gas: '3000000'});";

  string fullCommand = fullCommandStream.str();
  string res = exec_geth_cmd(i, fullCommand, nodeInt, datadirBase);
  cout << "Result received from SC is: " << res << endl;
  return res; 
}

// Interact with a function of a smart contract
// v: Amount of wei to send
/* Calls are just executed locally but no transactions are send to the blockchain  */
string Geth_Wrapper::smartContractInterfaceCall(int i, string interface, string contractAddress,
				   string func, int args[], int argc, int v, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "var cC = web3.eth.contract(" << interface << ");var c = cC.at(" << contractAddress << ");c." << func << ".call(";
  for(int k = 0; k < argc; k++) {
    fullCommandStream << args[k] << ",";  
  }
  fullCommandStream << "{" << "value: " << v << ", from: eth.coinbase, gas: '3000000'});";
  string fullCommand = fullCommandStream.str();

  string res = exec_geth_cmd(i, fullCommand, nodeInt, datadirBase);
  cout << "Result received from SC is: " << res << endl;

  return res;
}

/* Calls are just executed locally but no transactions are send to the blockchain  */
string Geth_Wrapper::smartContractInterfaceStringCall(int i, string interface, string contractAddress, string func, string args[], int argc, int v, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "var cC = web3.eth.contract(" << interface << ");var c = cC.at(" << contractAddress << ");c." << func << ".call(";
  for(int k = 0; k < argc; k++) {
    fullCommandStream << args[k] << ",";  
  }
  fullCommandStream << "{" << "value: " << v << ", from: eth.coinbase, gas: '3000000'});";
  string fullCommand = fullCommandStream.str();

  string res = exec_geth_cmd(i, fullCommand, nodeInt, datadirBase);
  cout << "Result received from SC is: " << res << endl;

  return res;
}

// Interact with a function of a smart contract
// v: Amount of wei to send
void Geth_Wrapper::smartContractInterfaceBg(int i, string interface, string contractAddress,
				   string func, int args[], int argc, int v, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "var cC = web3.eth.contract(" << interface << ");var c = cC.at(" << contractAddress << ");c." << func << "(";
  for(int k = 0; k < argc; k++) {
    fullCommandStream << args[k] << ",";  
  }
  fullCommandStream << "{" << "value: " << v << ", from: eth.coinbase, gas: '3000000'});";
  string fullCommand = fullCommandStream.str();

  cout << "REGISTRATION ROBOT" << endl;
  cout << fullCommand << endl;
  
  exec_geth_cmd_background(i, fullCommand, nodeInt, datadirBase);
  //cout << "Result received from SC is: " << res << endl;
}

// Interact with a function of a smart contract
// v: Amount of wei to send
void Geth_Wrapper::smartContractInterfaceStringBg(int i, string interface, string contractAddress,
				   string func, string args[], int argc, int v, int nodeInt, string datadirBase) {
  ostringstream fullCommandStream;
  fullCommandStream << "var cC = web3.eth.contract(" << interface << ");var c = cC.at(" << contractAddress << ");c." << func << "(";
  for(int k = 0; k < argc; k++) {
    fullCommandStream << args[k] << ",";  
  }
  fullCommandStream << "{" << "value: " << v << ", from: eth.coinbase, gas: '3000000'});";
  
  string fullCommand = fullCommandStream.str();
  //cout << "Executing full command: " << fullCommand << endl;
  exec_geth_cmd_background(i, fullCommand, nodeInt, datadirBase);
  //cout << "Result received from SC is: " << res << endl;
}

/* Check account balance of robot i (in wei) */
long long Geth_Wrapper::check_balance(int i, int nodeInt, string datadirBase) {
  string cmd = "eth.getBalance(eth.coinbase)";
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);  
  long long balance;// = atoi(res.c_str());

  if (res.find("true") != string::npos || res.find("Error") != string::npos) {
    balance = 0;
  } else {  
    istringstream ss(res);
    ss >> balance;
  }
  
  return balance;
}

/* Check account ether of robot i (in wei) */
long long Geth_Wrapper::check_ether(int i, int nodeInt, string datadirBase) {
  string cmd = "web3.fromWei(eth.getBalance(eth.coinbase),\"ether\")";
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);  
  long long ether;// = atoi(res.c_str());

  if (res.find("true") != string::npos || res.find("Error") != string::npos) {
    ether = 0;
  } else {  
    istringstream ss(res);
    ss >> ether;
  }
  
  return ether;
}

/* Check gas price of robot i (in wei) */
long long Geth_Wrapper::check_gasPrice(int i, int nodeInt, string datadirBase) {
  string cmd = "eth.gasPrice";
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);  
  long long gasPrice;// = atoi(res.c_str());

  if (res.find("true") != string::npos || res.find("Error") != string::npos) {
    gasPrice = 0;
  } else {  
    istringstream ss(res);
    ss >> gasPrice;
  }
  
  return gasPrice;
}

// Get blockchain length of robot i
int Geth_Wrapper::getBlockChainLength(int i, int nodeInt, string datadirBase) {
  string cmd = "eth.blockNumber";
  string res = exec_geth_cmd(i, cmd, nodeInt, datadirBase);  
  int blockNumber;

  /* TODO: A check would be better. Sometime eth.blockNumber returns
     true which is converted to 32515 */
  if (res.find("true") != string::npos) {
    cout << "Result in getBlockChainLength was " << res << endl;
    blockNumber = -1;
  } else {  
    istringstream ss(res);
    ss >> blockNumber;
  }
  return blockNumber;
}

/* Get the raw transaction based on the tx hash */
string Geth_Wrapper::getRawTransaction(int i, string txHash, int nodeInt, string datadirBase) {
  string cmd = "eth.getRawTransaction(" + txHash + ")";  
  string rawTx = exec_geth_cmd(i, cmd, nodeInt, datadirBase);  
  return rawTx;
}

/* Send raw transaction and include it in the tx pool */
string Geth_Wrapper::sendRawTransaction(int i, string rawTx, int nodeInt, string datadirBase) {
  string cmd = "eth.sendRawTransaction(" + rawTx + ")";
  string txHash = exec_geth_cmd(i, cmd, nodeInt, datadirBase);  
  return txHash;
}

/* Measure time and print it */
double Geth_Wrapper::measure_time(double ref_time, string part_name) {
  string alarm = "";
  double diff = get_wall_time() - ref_time;
  if (diff > 0.5)
    alarm = " ALARM";  
  cout << fixed << part_name << " took (ms): " << diff << alarm << endl;
  return get_wall_time();  
}

void Geth_Wrapper::generate_genesis(string address, int basePort) {
  ostringstream fullCommandStream;
  string addressNoSpace = address;
  addressNoSpace.erase(remove(addressNoSpace.begin(), addressNoSpace.end(), '\n'), addressNoSpace.end());  
  cout << "Generating new genesis file" << endl;
  fullCommandStream << "bash replace_genesis.sh " << addressNoSpace << " " << basePort;
  string fullCommand = fullCommandStream.str();
  cout << fullCommand << endl;
  system(fullCommand.c_str());
}

/* Prepare a robot for a new genesis block, i.e., remove all files except for the keystore */
void Geth_Wrapper::prepare_for_new_genesis(int i, int nodeInt, int basePort, string blockchainPath){
  /* Kill miner geth thread to replace genesis block */
  kill_geth_thread(i, basePort, nodeInt, blockchainPath);
  /* And a second time (since ssh creates two processes) */
  kill_geth_thread(i, basePort, nodeInt, blockchainPath);  
  /* Remove all files except for the keystore */
  ostringstream rmFilesStream;
  rmFilesStream << "rm -rf " << blockchainPath << i << "/geth*";
  string rmFilesStreamCmd = rmFilesStream.str();
  system(rmFilesStreamCmd.c_str());
}

/* Execute command line program and return string result */
string Geth_Wrapper::exec(const char* cmd) {
    char buffer[128];
    string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

string Geth_Wrapper::exec_geth_cmd(int i, string command, int nodeInt, string datadirBase){
  string res = exec_geth_cmd_helper(i, command, nodeInt, datadirBase);
  int trials = 20;
  // Retry to execute the command if it failed
  while ( (res.find("Fatal") != string::npos) || (res.find("Error") != string::npos)) {
    /* Reconstruct the full command stream */
    ostringstream fullCommandStream;
    fullCommandStream << "geth" <<  nodeInt << " --exec " << "'" << command << "'" << " attach " << datadirBase << i << "/" << "geth.ipc";    
    string fullCommand = fullCommandStream.str();
    sleep(1); // Wait for a second and retry
    res = exec_geth_cmd_helper(i, command, nodeInt, datadirBase);
    cout << "Trial" << (20-trials) << "\texec_geth_cmd: " << fullCommand << "\tResult: " << res << endl;
    if (trials == 0)
      break;
    trials --;
  }
  // TODO: if everything is implemented correctly and safely, I can
  //    let fatal errors occur both with Fatal error and normal
  //    errors if (res.find("Fatal") != string::npos || (res.find("Error") != string::npos)) {
  if (res.find("Fatal") != string::npos) {
    cout << "Fatal error!!!" << endl;
    cout << "res was " << res << endl;
    cout << "Called exec_geth_cmd with" << i << " " << command << endl;    
    sendMail(res);    
    gethStaticErrorOccurred = true;
  }
  
  return res;
}

// Take a geth command, execute it on the selected robot, and return the result string
string Geth_Wrapper::exec_geth_cmd_helper(int i, string command, int nodeInt, string datadirBase){
  ostringstream fullCommandStream;
  fullCommandStream << "geth" <<  nodeInt << " --exec " << "'" << command << "'" << " attach " << datadirBase << i << "/" << "geth.ipc";  
  string fullCommand = fullCommandStream.str();
  string res = exec(fullCommand.c_str());
  cout << "Command in helper is: " << fullCommand << ", result:" << res << endl;
  return res;  
}

string Geth_Wrapper::exec_geth_cmd_with_geth_restart(int i, string command, int nodeInt, int basePort, string datadirBase) {
  string res = exec_geth_cmd_helper(i, command, nodeInt, datadirBase);

  int trials = 20;
  // Retry to execute the command if it failed
  while ( (res.find("Fatal") != string::npos) || (res.find("Error") != string::npos) || (res.find("ILLEGAL") != string::npos)) {
    cout << res << endl;
    /* Reconstruct the full command stream */
    ostringstream fullCommandStream;
    fullCommandStream << "geth" <<  nodeInt << " --exec " << "'" << command << "'" << " attach " << datadirBase << i << "/" << "geth.ipc";
    string fullCommand = fullCommandStream.str();
    
    if (res.find("no such file") != string::npos) {
      /* Init geth again */
      start_geth(i, nodeInt, basePort, datadirBase);
    }
    
    sleep(1); // Wait for a second and retry
    res = exec_geth_cmd_helper(i, command, nodeInt, datadirBase);
    
    if (trials == 0)
      break;
    
    trials --;
  }

  // TODO: if everything is implemented correctly and safely, I can
  //    let fatal errors occur both with Fatal error and normal
  //    errors if (res.find("Fatal") != string::npos || (res.find("Error") != string::npos)) {
  if (res.find("Fatal") != string::npos) {
    cout << "Fatal error!!!" << endl;
    cout << "res was " << res << endl;
    cout << "Called exec_geth_cmd_with_geth_restart" << i << " " << command << endl;     
    sendMail(res);    
    gethStaticErrorOccurred = true;
  }
  
  return res;
}

bool Geth_Wrapper::exec_geth_cmd_wait(int i, string command, int nodeInt, int basePort, string datadirBase) {
  string res = exec_geth_cmd_helper(i, command, nodeInt, datadirBase);
  if (res.find("no such file") != string::npos) {
    return false;   
  } else {
    return true;
  }
}

// Take a geth command, execute it on the selected robot, and return the result string
void Geth_Wrapper::exec_geth_cmd_background(int i, string command, int nodeInt, string datadirBase){
  ostringstream fullCommandStream;
  /* The ampersand symol here makes the difference to the normal exec_geth_cmd function */
  fullCommandStream << "geth" <<  nodeInt << " --exec " << "'" << command << "'" << " attach " << datadirBase << i << "/" << "geth.ipc&";  
  string fullCommand = fullCommandStream.str();

  /* TODO: I cannot check for errors here since it is a background
     call; maybe I can come up with some error handling */  
  //  if (DEBUG)
  cout << "exec_geth_cmd_background: " << fullCommand << endl;  
  system(fullCommand.c_str());
}

void Geth_Wrapper::sendMail(string body){
  ofstream out("mail.txt");
  out << body;
  out.close();
  
  ostringstream fullCommandStream;
  /* Run geth command on this node  */
  //fullCommandStream << "echo " << body << " | mail -s Clustererror volker.strobel87@gmail.com";
  cout << "Sending mail" << endl;
  fullCommandStream << "mail -s Clustererror nttrungmt@gmail.com < mail.txt";
  string fullCommand = fullCommandStream.str();  
  system(fullCommand.c_str());
}

template<typename Out>
void Geth_Wrapper::split(const string &s, char delim, Out result) {
    stringstream ss;
    ss.str(s);
    string item;
    while (getline(ss, item, delim)) {
        *(result++) = item;
    }
}

vector<string> Geth_Wrapper::split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, back_inserter(elems));
    return elems;
}

string Geth_Wrapper::getUsername() {
  char *lgn;
  lgn = getlogin();
  string username(lgn);

  return username;
}

double Geth_Wrapper::get_wall_time(){
  struct timeval time;
  if (gettimeofday(&time,NULL)){
    //  Handle error
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

/*
  Convert a robot Id (fbxxx) to an integer (xxx)
*/
uint Geth_Wrapper::Id2Int(string id) {
  uint idConversion = id[2] - '0';
  if(id[3]!='\0')
    idConversion = (idConversion * 10) + (id[3] - '0');

    return idConversion;
}

string Geth_Wrapper::removeSpace(string str) {
  string noSpace = str;
  noSpace.erase(remove(noSpace.begin(), 
			    noSpace.end(), '\n'),
		       noSpace.end());
  return noSpace;
}

/* Replace the pattern from with to in the string str  */
bool Geth_Wrapper::replace(string& str, const string& from, const string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

/* Replace all occurrences of search with replace and return new string */
string Geth_Wrapper::replaceAll(string subject, const string& search,
                          const string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != string::npos) {
         subject.replace(pos, search.length(), replace);
         pos += replace.length();
    }
    return subject;
}

void Geth_Wrapper::ReplaceStringInPlace(string& subject, const string& search, const string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != string::npos) {
         subject.replace(pos, search.length(), replace);
         pos += replace.length();
    }
}

/* Reads the first line from a file */
string Geth_Wrapper::readStringFromFile(string fileName){
  string s;
  ifstream infile;
  infile.open(fileName.c_str());
  getline(infile, s); // Saves the line in s.
  infile.close();
  return s;
}

string Geth_Wrapper::readAllFromFile(string fileName){
  ifstream infile;
  infile.open(fileName.c_str());
  std::stringstream buffer;
  buffer << infile.rdbuf();
  infile.close();
  return buffer.str();
}