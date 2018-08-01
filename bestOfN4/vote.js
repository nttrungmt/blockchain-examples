web3.personal.unlockAccount(eth.coinbase, "test");
nEther=web3.fromWei(eth.getBalance(eth.coinbase),"ether");
console.log("Current ether=" + nEther);
web3.miner.start(1);
while(nEther <= 10) {
	admin.sleep(1);
	nEther=web3.fromWei(eth.getBalance(eth.coinbase),"ether");
	//console.log("Current ether=" + nEther);
}
var cC = web3.eth.contract([{"constant":true,"inputs":[{"name":"candidate","type":"uint256"}],"name":"validCandidate","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"numVoters","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"uid","type":"uint256"},{"name":"candidate","type":"uint256"}],"name":"voteForCandidate","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"getNumOfVoters","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"consensus","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"candidate","type":"uint256"}],"name":"totalVotesFor","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"candidateList","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"maxVoters","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"getNumOfCandidates","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"votesReceived","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"inputs":[{"name":"candidateNames","type":"uint256[]"},{"name":"maxNumVoters","type":"uint256"}],"payable":false,"stateMutability":"nonpayable","type":"constructor"}]);
var c = cC.at("0x2ebf1886caf9bcfc75597ba73915e0528fe45633");
c.voteForCandidate(0,2,{ from: eth.coinbase, gas: '3000000'});
//web3.admin.sleepBlocks(1);
while(true) {
	admin.sleep(1);
	nPendingTxLen = web3.eth.pendingTransactions.length;
	//console.log("Current PendingTxLen=" + nPendingTxLen);
	if(nPendingTxLen <= 0) 
		break;
}
web3.miner.stop();