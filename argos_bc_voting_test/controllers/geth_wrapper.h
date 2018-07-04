#ifndef GETH_WRAPPER_H
#define GETH_WRAPPER_H

#include <string>
#include <vector>

using namespace std;

struct blockWithHash {
  int blockNumber;
  std::string hash;
};

class Geth_Wrapper {
public:
	static string datadir_base;
	static const string genesis;
	static const string genesisTemplate;
	static const int rpc_base_port;
	static const int ipc_base_port;
	static const int maxtrials;
	static const string rack;
	static bool gethStaticErrorOccurred;
	
public:
    //init and create account, init ether if need
	static void initGethNode(int i, nodeInt, int basePort, string datadirBase, string genesisPath);
	
	// Initialize geth
	static void geth_init(int i, int nodeInt, int basePort, string datadirBase, string genesisPath);
	// Run geth for robot i
	static void start_geth(int i, int nodeInt, int basePort, string datadirBase); 
	
	/* Create a new account for robot i */
	static void createAccount(int i, int nodeInt, int basePort, string datadirBase);
	static string createAccountInit(int i, int nodeInt, int basePort, string datadirBase);

	/* Unlock account */
	static string unlockAccount(int i, string pw, int nodeInt, int basePort, string datadirBase);
	static string setEtherbase(int i, int nodeInt, int basePort, string datadirBase);
	
	/* Deploy contract using robot number i */
	static string deploy_contract(int i, string interfacePath, string dataPath, string templatePath, int nodeInt, string datadirBase);
	
	static string deploy_contract_script(int i, int nodeInt, string datadirBase, string script_file_path);

	// Get contract address from transaction receipt
	static string getContractAddress(int i, string txHash, int nodeInt, string datadirBase);
	
	// Get the enode information for robot i
	static string get_enode(int i, int nodeInt, int basePort, string datadirBase);

	// Start mining (robot i) using t threads
	static string start_mining(int i, int t, int nodeInt, string datadirBase);
	static void start_mining_bg(int i, int t, int nodeInt, string datadirBase);

	// Stop mining (robot i)
	static string stop_mining(int i, int nodeInt, string datadirBase);
	static void stop_mining_bg(int i, int nodeInt, string datadirBase);

	// Add a peer (specified via the enode) to robot i
	static string add_peer(int i, string enode, int nodeInt, int basePort, string datadirBase);
	static void add_peer_bg(int i, string enode, int nodeInt, string datadirBase);

	// Remove a peer (specified via the enode) from robot i
	static string remove_peer(int i, string enode, int nodeInt, string datadirBase);
	static void remove_peer_bg(int i, string enode, int nodeInt, string datadirBase);

	// Get coinbase address of robot i
	static string getCoinbase(int i, int nodeInt, int basePort, string datadirBase);

	// Get blockchain length of robot i
	static int getBlockChainLength(int i, int nodeInt, string datadirBase);

	// Send ether from robot i to address addr
	static string sendEther(int i, string from, string to, int v, int nodeInt, string datadirBase);

	// Interact with a generic smart contract
	static string smartContractInterface(int i, string interface, string contractAddress, string func, int args[], int argc, int v, int nodeInt, string datadirBase);
	static string eventInterface(int i, string interface, string contractAddress, int nodeInt, string datadirBase);
	static string smartContractInterfaceCall(int i, string interface, string contractAddress, string func, int args[], int argc, int v, int nodeInt, string datadirBase);
	static string smartContractInterfaceStringCall(int i, string interface, string contractAddress, string func, string args[], int argc, int v, int nodeInt, string datadirBase);
	static void smartContractInterfaceBg(int i, string interface, string contractAddress, string func, int args[], int argc, int v, int nodeInt, string datadirBase);
	static void smartContractInterfaceStringBg(int i, string interface, string contractAddress, string func, string args[], int argc, int v, int nodeInt, string datadirBase);

	/* Kill geth thread based on robot number i */
	static string kill_geth_thread(int i);
	static void kill_geth_thread(int i, int basePort, int nodeInt, string datadirBase);

	/* Check account balance of robot i (in wei)*/
	static long long check_balance(int i, int nodeInt, string datadirBase);
	static long long check_ether(int i, int nodeInt, string datadirBase);
	static long long check_gasPrice(int i, int nodeInt, string datadirBase);

	/* Get the raw transaction based on the tx hash */
	static string getRawTransaction(int i, string txHash, int nodeInt, string datadirBase);
	/* Send raw transaction and include it in the tx pool */
	static string sendRawTransaction(int i, string rawTx, int nodeInt, string datadirBase);

	/* Measure time and print it */
	static double measure_time(double ref_time, string part_name);
	static void generate_genesis(string address, int basePort);

	/* Prepare a robot for a new genesis block, i.e., remove all files except for the keystore */
	static void prepare_for_new_genesis(int i, int nodeInt, int basePort, string blockchainPath);
	
	// Execute a command line command and return the result as string
	static string exec(const char* cmd);
	// Take a geth command, execute it on the selected robot, and return the result string
	static string exec_geth_cmd(int i, string command, int nodeInt, string datadirBase);
	static string exec_geth_cmd_helper(int i, string command, int nodeInt, string datadirBase);
	static string exec_geth_cmd_with_geth_restart(int i, string command, int nodeInt, int basePort, string datadirBase);
	static bool exec_geth_cmd_wait(int i, string command, int nodeInt, int basePort, string datadirBase);
	static void exec_geth_cmd_background(int i, string command, int nodeInt, string datadirBase);

	// Send error messages per email
	static void sendMail(string body);
	
	//utilities methods
	template<typename Out>
	static void split(const string &s, char delim, Out result);
	static vector<string> split(const string &s, char delim);
	static double get_wall_time();
	static string getUsername();
	static uint Id2Int(string id);
	static bool replace(string& str, const string& from, const string& to);
	static string replaceAll(string subject, const string& search, const string& replace);
	static string removeSpace(string str);
	static void ReplaceStringInPlace(string& subject, const string& search, const string& replace);
	// Read first line of file fileName and return as string
	static string readStringFromFile(string fileName);
	static string readAllFromFile(string fileName);
};

#endif