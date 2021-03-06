pragma solidity ^0.4.18;
// We have to specify what version of compiler this code will compile with

contract Voting {
  /* mapping field below is equivalent to an associative array or hash.
  The key of the mapping is candidate name stored as type bytes32 and value is
  an unsigned integer to store the vote count
  */
  // describes a Voter, which has an id and the ID of the candidate they voted for
  /*struct Voter {
    uint uid; 			// bytes32 type are basically strings
    uint candidateVote;
  }*/
  
  /* Solidity doesn't let you pass in an array of strings in the constructor (yet).
  We will use an array of bytes32 instead to store the list of candidates
  */
  uint[] public candidateList;
  uint   public numVoters;
  uint   public maxVoters;

  //mapping (uint => uint) public votesReceived;
  //mapping (uint => Voter) public voters;
  uint[] voters;
  uint[] voterNewOpinion;
  uint[] voterInfFactors;

  /* This is the constructor which will be called once when you
  deploy the contract to the blockchain. When we deploy the contract,
  we will pass an array of candidates who will be contesting in the election
  */
  constructor(uint[] candidateNames, uint maxNumVoters) public {
    candidateList = candidateNames;
    maxVoters = maxNumVoters;
    voters = new uint[](maxVoters);
    voterNewOpinion = new uint[](maxVoters); 
    voterInfFactors = new uint[](maxVoters);
  }

  // This function increments the vote count for the specified candidate. This
  // is equivalent to casting a vote
  function updateInfFactor(uint uid, uint newOpinion, uint infFactor) public {
    voterNewOpinion[uid] = newOpinion;
    voterInfFactors[uid] = infFactor;
  }
  
  // This function returns the total votes a candidate has received so far
  function getBestOpinionIdx() view public returns (uint) {
    uint bestOpinionIdx = 0;
    uint bestInfFactor = 0;
    for (uint i = 0; i < maxVoters; i++) {
      // if the voter votes for this specific candidate, we increment the number
      if (voterInfFactors[i] > bestInfFactor) {
        bestInfFactor = voterInfFactors[i];
        bestOpinionIdx = i;
      }
    }
    return bestOpinionIdx;
  }
  
  function getBestOpinion(uint bestOpinionIdx) view public returns (uint) {
    return voterNewOpinion[bestOpinionIdx];
  }
  
  function getBestInfFactor(uint bestOpinionIdx) view public returns (uint) {
    return voterInfFactors[bestOpinionIdx];
  }
  
  // This function increments the vote count for the specified candidate. This
  // is equivalent to casting a vote
  function voteForCandidate(uint uid, uint candidate) public {
    require(validCandidate(candidate));
    numVoters++;
    voters[uid] = candidate;
  }

  // This function returns the total votes a candidate has received so far
  function totalVotesFor(uint candidate) view public returns (uint) {
    require(validCandidate(candidate));
    uint numOfVotes = 0; // we will return this
    for (uint i = 0; i < maxVoters; i++) {
      // if the voter votes for this specific candidate, we increment the number
      if (voters[i] == candidate) {
        numOfVotes++;
      }
    }
    return numOfVotes;
  }
  
  // This function returns the voted candidate of the specified robot
  function getVotesFor(uint uid) view public returns (uint) {
    return voters[uid];
  }

  function consensus() view public returns (bool) {
    for(uint i = 0; i < candidateList.length; i++) {
      uint numOfVotes = totalVotesFor(candidateList[i]);
      if (numOfVotes >= maxVoters) {
        return true;
      }
    }
    return false;
  }

  function getNumOfCandidates() public view returns(uint) {
    return candidateList.length;
  }

  function getNumOfVoters() public view returns(uint) {
    return numVoters;
  }

  function validCandidate(uint candidate) view public returns (bool) {
    for(uint i = 0; i < candidateList.length; i++) {
      if (candidateList[i] == candidate) {
        return true;
      }
    }
    return false;
  }
}
