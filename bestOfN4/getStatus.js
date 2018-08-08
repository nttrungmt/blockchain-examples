var VotingContract = web3.eth.contract([{"constant":true,"inputs":[{"name":"candidate","type":"uint256"}],"name":"validCandidate","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"numVoters","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"uid","type":"uint256"},{"name":"newOpinion","type":"uint256"},{"name":"infFactor","type":"uint256"}],"name":"updateInfFactor","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":false,"inputs":[{"name":"uid","type":"uint256"},{"name":"candidate","type":"uint256"}],"name":"voteForCandidate","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"getNumOfVoters","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"consensus","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"candidate","type":"uint256"}],"name":"totalVotesFor","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"getBestOpinionIdx","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"candidateList","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"bestOpinionIdx","type":"uint256"}],"name":"getBestOpinion","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"uid","type":"uint256"}],"name":"getVotesFor","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"maxVoters","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"getNumOfCandidates","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"bestOpinionIdx","type":"uint256"}],"name":"getBestInfFactor","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"inputs":[{"name":"candidateNames","type":"uint256[]"},{"name":"maxNumVoters","type":"uint256"}],"payable":false,"stateMutability":"nonpayable","type":"constructor"}]);
var deployedContract = VotingContract.at('0x98e779f56d997da44376a25b6709e3ce2c9c1cb0');
var idx = deployedContract.getBestOpinionIdx.call();
console.log("BestOpinion="+deployedContract.getBestOpinion.call(idx));
console.log("BestInfFactor="+deployedContract.getBestInfFactor.call(idx));
console.log("nCandidates="+deployedContract.getNumOfCandidates.call());
console.log("nVoters="+deployedContract.getNumOfVoters.call());
console.log("Vote of 0="+deployedContract.getVotesFor.call(0));
console.log("Vote of 1="+deployedContract.getVotesFor.call(1));
console.log("Vote of 2="+deployedContract.getVotesFor.call(2));
console.log("Vote of 3="+deployedContract.getVotesFor.call(3));
console.log("Vote of 4="+deployedContract.getVotesFor.call(4));
console.log("Vote of 5="+deployedContract.getVotesFor.call(5));
