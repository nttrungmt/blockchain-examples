var simplestorageContract = web3.eth.contract([{"constant":false,"inputs":[{"name":"x","type":"uint256"}],"name":"set","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"get","outputs":[{"name":"retVal","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"}]);
var simplestorageInst = simplestorageContract.at('0x1e02b7368b3a4e91c682cfde55280925b78ce1d0');
console.log(simplestorageInst.get());
