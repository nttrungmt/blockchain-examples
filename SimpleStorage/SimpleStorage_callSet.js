var simplestorageContract = web3.eth.contract([{"constant":false,"inputs":[{"name":"x","type":"uint256"}],"name":"set","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"get","outputs":[{"name":"retVal","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"}]);
personal.unlockAccount(eth.coinbase,"test");
var simplestorageInst = simplestorageContract.at('0x2b806250aff623e10eae6c4b8144b84aac7f24af');
simplestorageInst.set(5,{from: eth.accounts[0], gas: 3000000})
