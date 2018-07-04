var simplestorageContract = web3.eth.contract([{"constant":false,"inputs":[{"name":"x","type":"uint256"}],"name":"set","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"get","outputs":[{"name":"retVal","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"}]);
var simplestorage = simplestorageContract.new(
   {
     from: web3.eth.accounts[0], 
     data: '0x608060405234801561001057600080fd5b5060df8061001f6000396000f3006080604052600436106049576000357c0100000000000000000000000000000000000000000000000000000000900463ffffffff16806360fe47b114604e5780636d4ce63c146078575b600080fd5b348015605957600080fd5b5060766004803603810190808035906020019092919050505060a0565b005b348015608357600080fd5b50608a60aa565b6040518082815260200191505060405180910390f35b8060008190555050565b600080549050905600a165627a7a72305820729b6fb330fedd414f69bf706030faf67a6f9256748fb5d3768da5049c581d540029', 
     gas: 3000000
   }, function(e, contract){
    console.log('error: ' + e);
    console.log('transactionHash: ' + contract.transactionHash);
    console.log('address: ' + contract.address);
    //if(!e) {
      while(!contract.transactionHash) {
        admin.sleep(1);
      }
      if (typeof contract.address != 'undefined') {
         console.log('Contract mined! address: ' + contract.address + ' transactionHash: ' +  contract.transactionHash);
      }
      else {
        console.log('Error in deploying contract! Null address!!!');
      }
    //}
 });
