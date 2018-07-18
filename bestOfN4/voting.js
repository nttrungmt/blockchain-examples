var candidateNames = ['RED','GREEN','BLUE']/* var of type bytes32[] here */ ;
var VotingContract = web3.eth.contract([{"constant":true,"inputs":[{"name":"candidate","type":"bytes32"}],"name":"totalVotesFor","outputs":[{"name":"","type":"uint8"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"candidate","type":"bytes32"}],"name":"validCandidate","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"","type":"bytes32"}],"name":"votesReceived","outputs":[{"name":"","type":"uint8"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"candidateList","outputs":[{"name":"","type":"bytes32"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"candidate","type":"bytes32"}],"name":"voteForCandidate","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"inputs":[{"name":"candidateNames","type":"bytes32[]"}],"payable":false,"stateMutability":"nonpayable","type":"constructor"}]);
var deployedContract = VotingContract.new(
   candidateNames,
   {
     from: web3.eth.accounts[0], 
     data: '0x608060405234801561001057600080fd5b50604051610412380380610412833981018060405281019080805182019291905050508060019080519060200190610049929190610050565b50506100c8565b828054828255906000526020600020908101928215610092579160200282015b82811115610091578251829060001916905591602001919060010190610070565b5b50905061009f91906100a3565b5090565b6100c591905b808211156100c15760008160009055506001016100a9565b5090565b90565b61033b806100d76000396000f30060806040526004361061006d576000357c0100000000000000000000000000000000000000000000000000000000900463ffffffff1680632f265cf714610072578063392e6678146100bd5780637021939f14610106578063b13c744b14610151578063cc9ab2671461019a575b600080fd5b34801561007e57600080fd5b506100a160048036038101908080356000191690602001909291905050506101cb565b604051808260ff1660ff16815260200191505060405180910390f35b3480156100c957600080fd5b506100ec6004803603810190808035600019169060200190929190505050610210565b604051808215151515815260200191505060405180910390f35b34801561011257600080fd5b50610135600480360381019080803560001916906020019092919050505061026f565b604051808260ff1660ff16815260200191505060405180910390f35b34801561015d57600080fd5b5061017c6004803603810190808035906020019092919050505061028f565b60405180826000191660001916815260200191505060405180910390f35b3480156101a657600080fd5b506101c960048036038101908080356000191690602001909291905050506102b2565b005b60006101d682610210565b15156101e157600080fd5b600080836000191660001916815260200190815260200160002060009054906101000a900460ff169050919050565b600080600090505b60018054905081101561026457826000191660018281548110151561023957fe5b90600052602060002001546000191614156102575760019150610269565b8080600101915050610218565b600091505b50919050565b60006020528060005260406000206000915054906101000a900460ff1681565b60018181548110151561029e57fe5b906000526020600020016000915090505481565b6102bb81610210565b15156102c657600080fd5b6001600080836000191660001916815260200190815260200160002060008282829054906101000a900460ff160192506101000a81548160ff021916908360ff160217905550505600a165627a7a72305820573fcb673954112290d8852a5df215be0f9601f873f084ea4342e2f02ecfbbf20029', 
     gas: 3000000
   }, function(e, contract){
    console.log(contract.transactionHash);
    if (typeof contract.address != 'undefined') {
         console.log('Contract mined! address: ' + contract.address + ' transactionHash: ' + contract.transactionHash);
    }
 });
//contractInstance = VotingContract.at(deployedContract.address);