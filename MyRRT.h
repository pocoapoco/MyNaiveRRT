#include <vector>
#include <cmath>
#include <iostream>

using namespace std;
using namespace OpenRAVE;

typedef std::vector<double> Configuration;
typedef int Index;

class RRTNode
{
public:

	RRTNode(EnvironmentBasePtr envPtr, const Configuration &config)
	{
		_config = config;
		_parentIndex = -1;

		envPtr->GetRobot("PR2")->GetActiveDOFWeights(_DOFWeights);
	}

    // add parent node's index in NodeTree
	void addParent(Index parentIndex)
	{
		_parentIndex = parentIndex;
	}

    // Euclidean distance to sample point
	float distanceTo(const Configuration &sampleConfig)
	{
		double distance = 0.0;

		for (unsigned int i = 0; i < _config.size(); ++i)
			distance += (_config[i]-sampleConfig[i])*(_config[i]-sampleConfig[i])*_DOFWeights[i]*_DOFWeights[i];

		return sqrt(distance);
	}

	Configuration getConfig()
	{
		return _config;
	}

	Index getParentIndex()
	{
		return _parentIndex;
	}

	bool isGoal(const Configuration &goalConfig)
	{
		for (unsigned int i = 0; i < _config.size(); ++i)
			if (fabs(_config[i] - goalConfig[i]) > 0.01)
				return false;

		return true;
	}

private:

	Configuration _config;  // store a C-space point
	Index _parentIndex;
	vector<double> _DOFWeights;  // coefficients to reweight resolution for each joint

};


class NodeTree
{
public:
	
	bool init(EnvironmentBasePtr envPtr, const double step_size, const Configuration &initConfig, const Configuration &goalConfig)
	{
		_envPtr = envPtr;
		_robotPtr = envPtr->GetRobot("PR2");

		_goalConnected = false;
		_stepSize = step_size;
		_goalConfig = goalConfig;

		addNode(initConfig); // initialize a new tree by adding initConfig as the first node

		_spaceDim = goalConfig.size();
		_nSample = 0;

		return true;
	}

	int treeSize()
	{
		return _nodes.size();
	}

    // check if sample is in C_free
	bool inCollision(const Configuration &sampleConfig)
	{
		_robotPtr->SetActiveDOFValues(sampleConfig);

        if ( _envPtr->CheckCollision(_robotPtr) || _robotPtr->CheckSelfCollision() )
        	return true;
        else
        	return false;

	}

    // check if Euclidean distance between sample and NN is within step size
	bool withinStepSize(const Configuration &sampleConfig)
	{

		if (_nodes[nearestNodeIndex(sampleConfig)].distanceTo(sampleConfig) > _stepSize)
			return false;

		return true;
	}

	// find nearest node from sample point
	// older version using pointer: RRTNode* nearestNode(const Configuration &sampleConfig)
	Index nearestNodeIndex(const Configuration &sampleConfig)
	{
		float minDist = 1E6;
		int minIndex = -1;

        for (unsigned int i = 0; i < _nodes.size(); ++i)
		{

			if (_nodes[i].distanceTo(sampleConfig) < minDist)
			{
				if (minDist <= 0.01)
				{
					minIndex = i;
					break;
				}
				else
				{
					minDist = _nodes[i].distanceTo(sampleConfig);
					minIndex = i;
				}
			}

		}

		return minIndex;
	}

    // the EXTEND algorithm
	void addNode(const Configuration &sampleConfig)
	{

		_nSample++;

		// if tree is empty, add first node without finding NN
		if (treeSize() == 0)
		{
			RRTNode initNode(_envPtr, sampleConfig);
			_nodes.push_back(initNode);
			return;
		}

        // find NN to current sample point
		Index NNIndex = nearestNodeIndex(sampleConfig);

        // if sample is outside of step size, incrementally add nodes for every step on local path
		if (!withinStepSize(sampleConfig))
		{
	    	int numSteps = _nodes[NNIndex].distanceTo(sampleConfig)/_stepSize;  // how many steps to add until distance to sample < step size
	    	float newSamplePos = (numSteps*_stepSize)/_nodes[NNIndex].distanceTo(sampleConfig);  // position of furthest step on local path

	    	// add one node each step forward on local path
	    	for (int step = 1; step <= numSteps; ++step)
	    	{
	    		// initialize step point
		        Configuration newStepConfig(_spaceDim);

                // construct step point
		        for (int i = 0; i < _spaceDim; ++i)
		        	newStepConfig[i] = _nodes[NNIndex].getConfig()[i] + ((sampleConfig[i]-_nodes[NNIndex].getConfig()[i])*newSamplePos*step)/numSteps;

                // if step point is in C_free, construct node and add to tree; otherwise, terminate local planning
		        if (!inCollision(newStepConfig))
		        {

		        	RRTNode newStepNode(_envPtr, newStepConfig);
		        	newStepNode.addParent(NNIndex);
		        	_nodes.push_back(newStepNode);

	    	        NNIndex = _nodes.size()-1;  // redirect nearest node to newly added step node
	    	    }
	    	    else
	    	    {
	    	    	return;		    	 
	    	    }
	    	}

	    }

	    // sample is now within stepsize to NN
	    // if sample is in C_free, add to tree directly; otherwise, do nothing
	    // _sampleTimes += 1;
	    if (!inCollision(sampleConfig))
	    {

	    	RRTNode newNode(_envPtr, sampleConfig);
	    	newNode.addParent(NNIndex);
	    	_nodes.push_back(newNode);

            // after sample node is added into tree, check if it's goal
            // if sample is goal, planning succeeds
	    	if (newNode.isGoal(_goalConfig))
	    	{

	    		_goalConnected = true;

	        	// construct path by backtracking sequence of parent nodes from goal to init
	    		Index wayIndex = _nodes.size()-1;
	    		
	    		while (wayIndex != -1)
	    		{

	    			_path.push_back(_nodes[wayIndex].getConfig());
	    			wayIndex = _nodes[wayIndex].getParentIndex();
	    		}

	    	}

	    }
	    // if sample is in collision, do nothing and terminate local planning
	    else
	    {
	    	return;
		}

	}

	void smooth(vector<Configuration> &path)
	{

		// cout << "original len: " << path.size() << endl;

		for (int i = 0; i < 200; ++i)
		{

			// if less than 10 nodes on path, stop shortcutting
			if (path.size() <= 5)
				break;

			int sample1 = rand()%path.size();
			int sample2 = rand()%path.size();

			// if 
			while ((sample1 == 0 && sample2 == 0) || abs(sample1-sample2 <= 1))
			{
				sample1 = rand()%path.size();
				sample2 = rand()%path.size();
			}

			int first = min(sample1, sample2);
			int second = max(sample1, sample2);

			RRTNode firstNode(_envPtr, path[first]);

			// if without one step, try each step
			if (firstNode.distanceTo(path[second]) > _stepSize)
			{
				int numSteps = firstNode.distanceTo(path[second])/_stepSize;  // how many steps to add until distance to sample < step size
	    		float newSamplePos = (numSteps*_stepSize)/firstNode.distanceTo(path[second]);  // position of furthest step on local path

		    	bool shortCutFailed = false;
		    	// add one node each step forward on local path
		    	for (int step = 1; step <= numSteps; ++step)
		    	{
		    		// initialize step point
			        Configuration newStepConfig(_spaceDim);

	                // construct step point
			        for (int d = 0; d < _spaceDim; ++d)
			        	newStepConfig[d] = path[first][d] + ((path[second][d]-path[first][d])*newSamplePos*step)/numSteps;

			        // if step in collision, discard connection
	                if (inCollision(newStepConfig))
	                {
	                	shortCutFailed = true;
	                	break;
	                }
	            }

	            // if all steps are collision-free, do shortcut
            	if (shortCutFailed == false)
            		path.erase(path.begin()+first+1, path.begin()+second);

            	// cout << "path len: " << path.size() << endl;

            }

		}

		// cout << "smoothed len: " << path.size() << endl;

	}

	void clear()
	{
		_nodes.clear();
		_path.clear();
	}

	bool goalConnected()
	{
		return _goalConnected;
	}

	vector<Configuration> getPath()
	{
		return _path;
	}

	// debug
	void printStatistics()
	{
		cout << "sampled: " << _nSample << endl;
		cout << "tree size: " << _nodes.size() << endl;
	}



private:

    EnvironmentBasePtr _envPtr; // pointer to environment passed in by user
    RobotBasePtr _robotPtr; // pointer to specified robot in environment

	vector<RRTNode> _nodes; // nodes stored in RRT
	Configuration _goalConfig;
	bool _goalConnected;
	float _stepSize;
	vector<Configuration> _path; // path found
	int _nSample;
	int _spaceDim;  // dimension of C-space

};

