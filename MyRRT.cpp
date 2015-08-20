#include <openrave/plugin.h>
#include <openrave/planningutils.h>
#include <boost/bind.hpp>

#include <vector>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include "MyRRT.h"

using namespace std;
using namespace OpenRAVE;


class MyRRT : public ModuleBase
{
public:
    MyRRT(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv), _initConfig(7), _goalConfig(7)
    {
        RegisterCommand("setPlannerPar", boost::bind(&MyRRT::setPlannerPar, this, _1, _2), "Set the goal bias, max interation, and step size");
        RegisterCommand("setInitialConfig", boost::bind(&MyRRT::setInitialConfig, this, _1, _2), "Set initial configuration");
        RegisterCommand("setGoalConfig", boost::bind(&MyRRT::setGoalConfig, this, _1, _2), "Set goal configuration");
        RegisterCommand("init", boost::bind(&MyRRT::init, this, _1, _2), "Initialize the RRT planner");
        RegisterCommand("run", boost::bind(&MyRRT::run, this, _1, _2), "Run the searching");
        RegisterCommand("getFingerPath", boost::bind(&MyRRT::getFingerPath, this, _1, _2), "Get work-space Path for end effector");
        RegisterCommand("smoothPath", boost::bind(&MyRRT::smoothPath, this, _1, _2), "smooth path");        
        RegisterCommand("executeTraj", boost::bind(&MyRRT::executeTraj, this, _1, _2), "construct trajectory and execute on robot");
        RegisterCommand("clearTree", boost::bind(&MyRRT::clearTree, this, _1, _2), "remove all nodes from tree");
    }
    virtual ~MyRRT() {}

    bool setPlannerPar(std::ostream& sout, std::istream& sinput)
    {
        vector<string> parsedInput = tokenize(sinput);

        _goalFrequency = boost::lexical_cast<double>(parsedInput[0].c_str());
        _maxIteration = boost::lexical_cast<int>(parsedInput[1].c_str());
        _stepSize = boost::lexical_cast<double>(parsedInput[2].c_str());
        _spaceDim = 7;

        return true;
    }

    bool setInitialConfig(std::ostream& sout, std::istream& sinput)
    {
        vector<string> parsedInput = tokenize(sinput);
        for(int i = 0; i < _spaceDim; ++i)
        {
            _initConfig[i] = boost::lexical_cast<double>(parsedInput[i].c_str());
        }

        return true;
    }

    bool setGoalConfig(std::ostream& sout, std::istream& sinput)
    {
        vector<string> parsedInput = tokenize(sinput);
        for(int i = 0; i < _spaceDim; ++i)
        {
            _goalConfig[i] = boost::lexical_cast<double>(parsedInput[i].c_str());
        }

        return true;
    }

    bool init(std::ostream& sout, std::istream& sinput)
    {
        //Initialize the tree
        _tree.init(GetEnv(), _stepSize, _initConfig, _goalConfig);

        //Set Robot limits
        GetEnv()->GetRobot("PR2")->GetActiveDOFLimits(_lowerDOFLimits, _upperDOFLimits);
        _lowerDOFLimits[4] = -3.1416;
        _lowerDOFLimits[6] = -3.1416;
        _upperDOFLimits[4] = 3.1416;
        _upperDOFLimits[6] = 3.1416;

        return true;
    }

    bool run(std::ostream& sout, std::istream& sinput)
    {

        if(pathFound())
        {
            sout << "success";
        }
        else
        {
            sout << "failed";
        }

        _tree.printStatistics();
        return true;
    }

    bool getFingerPath(std::ostream& sout, std::istream& sinput)
    {

        EnvironmentBasePtr env = GetEnv();
        RobotBasePtr robot = env->GetRobot("PR2");
        
        for (unsigned int i = 0; i < _myPath.size(); ++i)
        {
            robot->SetActiveDOFValues(_myPath[i]);

            RaveVector<double> Trans = robot->GetManipulators()[6]->GetEndEffectorTransform().trans;

            sout << Trans.x << ",";
            sout << Trans.y << ",";
            sout << Trans.z << ";";
        }

        return true;

    }

    bool smoothPath(std::ostream& sout, std::istream& sinput)
    {
        _tree.smooth(_myPath);
        return true;

    }

    bool executeTraj(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentBasePtr env = GetEnv();
        RobotBasePtr robot = GetEnv()->GetRobot("PR2");

        TrajectoryBasePtr traj = RaveCreateTrajectory(env, "");

        ConfigurationSpecification configSpec = robot->GetActiveConfigurationSpecification();
        // int timeoffset = configSpec.AddDeltaTimeGroup();

        traj->Init(configSpec);


        for (unsigned int i = 0; i < _myPath.size(); ++i)
        {
            traj->Insert(i, _myPath[i]);
        }
        
        // planningutils::SmoothActiveDOFTrajectory(traj, robot, 0.2);
        planningutils::RetimeActiveDOFTrajectory(traj, robot, false, 0.3);
        GetEnv()->GetRobot("PR2")->GetController()->SetPath(traj);

        return true;
        
    }

    bool clearTree(std::ostream& sout, std::istream& sinput)
    {
        _tree.clear();
        _myPath.clear();
        return true;
    }

    // run RRT for maxIteration times and report if path is found
    bool pathFound()
    {
        for (int i = 0; i < _maxIteration; ++i)
        {

            // if goal is connected, planning succeeded and construct path
            if (_tree.goalConnected())
            {
                
                for (unsigned int j = _tree.getPath().size(); j > 0; --j)
                {
                    _myPath.push_back(_tree.getPath()[j-1]);
                }

                return true;
            }

            // if goal is not found, generate new sample and extend
            Configuration sampleConfig = sampleNewConfig();

            _tree.addNode(sampleConfig);

        }

        // if goal is not connected after max iteration is reached, planning failed
        return false;
    }

    Configuration sampleNewConfig()
    {
        if ((float) rand()/(float) RAND_MAX < _goalFrequency)
        {
            return _goalConfig;
        }
        else
        {
            Configuration sampleConfig(_spaceDim);

            for(int i = 0; i < _spaceDim; ++i) 
            {

                sampleConfig[i] = _lowerDOFLimits[i] + (float) rand()*(_upperDOFLimits[i]-_lowerDOFLimits[i])/(float) RAND_MAX;
                // sampleConfig[i] = _lowerDOFLimits[i] + RaveRandomFloat()*(_upperDOFLimits[i]-_lowerDOFLimits[i]);
            }

            return sampleConfig;
        }
    }

    vector<string> tokenize(std::istream& sinput)
    {
        string input;
        sinput >> input;

        cout << input << endl;

        vector<string> vals;
        boost::char_separator<char> sep(",");
        boost::tokenizer<boost::char_separator<char> > tok(input, sep);
        for(boost::tokenizer<boost::char_separator<char> >::iterator iter = tok.begin(); iter != tok.end(); ++iter)
        {
            vals.push_back(*iter);
            cout << *iter << endl;
        }

        return vals;
    }

private:

    // search tree
    NodeTree _tree;

    // planner parameters
    double _goalFrequency;
    int _maxIteration;
    double _stepSize;
    int _spaceDim;

    // init and goal
    Configuration _initConfig;
    Configuration _goalConfig;

    // DOF limits
    Configuration _lowerDOFLimits;
    Configuration _upperDOFLimits;

    // path found
    vector<Configuration> _myPath;

};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "myrrt" ) {
        return InterfaceBasePtr(new MyRRT(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("MyRRT");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}