var candidateNames = [1,2] /* var of type bytes32[] here */ ;
var VotingContract = web3.eth.contract([{"constant":true,"inputs":[{"name":"candidate","type":"uint256"}],"name":"validCandidate","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"numVoters","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"uid","type":"uint256"},{"name":"candidate","type":"uint256"}],"name":"voteForCandidate","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"getNumOfVoters","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"consensus","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"candidate","type":"uint256"}],"name":"totalVotesFor","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"candidateList","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"maxVoters","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[],"name":"getNumOfCandidates","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"votesReceived","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"inputs":[{"name":"candidateNames","type":"uint256[]"},{"name":"maxNumVoters","type":"uint256"}],"payable":false,"stateMutability":"nonpayable","type":"constructor"}]);
var deployedContract = VotingContract.new(
   candidateNames, 4,
   {
     from: web3.eth.accounts[0], 
     data: '0x608060405234801561001057600080fd5b5060405161060d38038061060d833981018060405281019080805182019291906020018051906020019092919050505081600090805190602001906100569291906100ad565b508060028190555060025460405190808252806020026020018201604052801561008f5781602001602082028038833980820191505090505b50600490805190602001906100a59291906100ad565b50505061011f565b8280548282559060005260206000209081019282156100e9579160200282015b828111156100e85782518255916020019190600101906100cd565b5b5090506100f691906100fa565b5090565b61011c91905b80821115610118576000816000905550600101610100565b5090565b90565b6104df8061012e6000396000f3006080604052600436106100a4576000357c0100000000000000000000000000000000000000000000000000000000900463ffffffff16806321b4bdde146100a95780634cbe32b8146100ee578063626b2c981461011957806365fc783c146101505780638ef3f7611461017b578063a805f1ca146101aa578063b13c744b146101eb578063d5e50a631461022c578063e8685ba114610257578063f3084a6314610282575b600080fd5b3480156100b557600080fd5b506100d4600480360381019080803590602001909291905050506102c3565b604051808215151515815260200191505060405180910390f35b3480156100fa57600080fd5b5061010361031a565b6040518082815260200191505060405180910390f35b34801561012557600080fd5b5061014e6004803603810190808035906020019092919080359060200190929190505050610320565b005b34801561015c57600080fd5b5061016561038a565b6040518082815260200191505060405180910390f35b34801561018757600080fd5b50610190610394565b604051808215151515815260200191505060405180910390f35b3480156101b657600080fd5b506101d5600480360381019080803590602001909291905050506103f9565b6040518082815260200191505060405180910390f35b3480156101f757600080fd5b5061021660048036038101908080359060200190929190505050610466565b6040518082815260200191505060405180910390f35b34801561023857600080fd5b50610241610489565b6040518082815260200191505060405180910390f35b34801561026357600080fd5b5061026c61048f565b6040518082815260200191505060405180910390f35b34801561028e57600080fd5b506102ad6004803603810190808035906020019092919050505061049b565b6040518082815260200191505060405180910390f35b600080600090505b60008054905081101561030f57826000828154811015156102e857fe5b906000526020600020015414156103025760019150610314565b80806001019150506102cb565b600091505b50919050565b60015481565b610329816102c3565b151561033457600080fd5b600160036000838152602001908152602001600020600082825401925050819055506001600081548092919060010191905055508060048381548110151561037857fe5b90600052602060002001819055505050565b6000600154905090565b60008060008091505b6000805490508210156103ef576103cc6000838154811015156103bc57fe5b90600052602060002001546103f9565b9050600254811015156103e257600192506103f4565b818060010192505061039d565b600092505b505090565b6000806000610407846102c3565b151561041257600080fd5b60009150600090505b60025481101561045c578360048281548110151561043557fe5b9060005260206000200154141561044f5781806001019250505b808060010191505061041b565b8192505050919050565b60008181548110151561047557fe5b906000526020600020016000915090505481565b60025481565b60008080549050905090565b600360205280600052604060002060009150905054815600a165627a7a723058205b4dd8cd8c1c2af0cae1bc60db4002e1a339e4101eac299134b82755b9b150120029', 
     gas: 3000000
   }, function(e, contract){
    console.log(contract.transactionHash);
    if (typeof contract.address != 'undefined') {
         console.log('Contract mined! address: ' + contract.address + ' transactionHash: ' + contract.transactionHash);
    }
 });
//contractInstance = VotingContract.at(deployedContract.address);
