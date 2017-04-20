#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include "environment_handler.h"
#include "motion_plan.h"
#include <iostream>
#include <unistd.h>


using namespace OpenRAVE;
using std::vector;
using std::string;
using std::cout; using std::endl;

class EscherMotionPlanning : public ModuleBase
{
    public:
        EscherMotionPlanning(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
            RegisterCommand("StartPlanning",boost::bind(&EscherMotionPlanning::Planning,this,_1,_2),
                            "Start the planning process.");
        }
        virtual ~EscherMotionPlanning() {}
        
        bool Planning(std::ostream& sout, std::istream& sinput)
        {
            string robot_name;

            string param;

            while(!sinput.eof())
            {
                sinput >> param;
                if(!sinput)
                    break;

                if(strcmp(param.c_str(), "robotname") == 0)
                {
                    sinput >> robot_name;
                }

                if(strcmp(param.c_str(), "goal") == 0)
                {
                    _goal.resize(3);
                    for(int i = 0; i < 3; i++)
                    {
                        sinput >> _goal[i];
                    }
                    std::cout<<"The goal is: (x,y,theta) = ("<<_goal[0]<<","<<_goal[1]<<","<<_goal[2]<<")"<<std::endl;
                }

                if(strcmp(param.c_str(), "parallelization") == 0)
                {
                    sinput >> param;
                    if(strcmp(param.c_str(), "0") == 0)
                    {
                        _is_parallel = false;
                        std::cout<<"Don't do parallelization."<<std::endl;
                    }
                    else
                    {
                        _is_parallel = true;
                        std::cout<<"Do parallelization."<<std::endl;
                    }
                }

            }

            vector<RobotBasePtr> robots;

            _penv = GetEnv();

            GetEnv()->GetRobots(robots);
            SetActiveRobots(robot_name,robots);
            try {
                // Construct the environment objects. (See KinBody in OpenRAVE API, and env_handler.py) 
                Environment_handler env_handler{GetEnv()};
                // sout << "Nearest boundary: " << env_handler.dist_to_boundary(0, 0, 0) << "\n";
                // ****************************************************************************//
                // Something about constructing environment objects. (walls, ground, and etc.)//
                // ****************************************************************************//

                // After loading all the parameters, environment object and robot objects, you can execute the main planning function.

                // **************************//
                // **************************//
                // Something about planning //
                Motion_plan_library mpl;
                vector<Contact> last_plan {
                    {{0, -0.15, 0}, Manip::L_foot},
                    {{0, 0.15, 0}, Manip::R_foot},
                    {{.15, -0.15, 0}, Manip::L_foot},
                    {{.3, 0.15, 0}, Manip::R_foot},
                    {{.45, -0.15, 0}, Manip::L_foot},
                    {{.6, 0.15, 0}, Manip::R_foot},
                    {{.75, -0.15, 0}, Manip::L_foot},
                    {{.75, 0.15, 0}, Manip::R_foot},
                };

                vector<Vector> s{
                    // 0, 0, 1.15, 1.3, .3, .3, .3, .3
                    {}, {0, .3, 0}, {.15, -0.3, 0}, {.45, .3, 0}, {.15, -.3, 0}, {.15, .3, 0}, {.15, -.3, 0}, {0, .3, 0}
                };

                // for(size_t m = 2; m < 8; ++m) {
                Drawing_handler dh{GetEnv()};

                //     s[m] = s[m] * 1.1;

                //     vector<Contact> t_plan = mpl.transform_plan(last_plan, 0, 0, 0, 0, s);
                //     cout << "---------------" << endl;
                //     cout << t_plan.size() << endl;
                //     for(size_t i = 0; i < t_plan.size(); ++i) {
                //         dh.DrawRegion({t_plan[i].tf.x, t_plan[i].tf.y, t_plan[i].tf.z}, {0, 0, 1}, 0.05, 1);
                //         cout << "x: " << t_plan[i].tf.x << " y: " << t_plan[i].tf.y << " z: " << t_plan[i].tf.z << endl;
                //     }
                //     cout << "---------------" << endl;
                //     usleep(1000000);
                // }

                // mpl.learn(last_plan);
                mpl.query(dh, env_handler.get_contact_regions(),{},{});
                
                int a;
                std::cin>>a; // block
            } catch(std::exception & e) {
                sout << "Exception caught: " << e.what() << "\n";
            }

            //return the result
            return true;
        }

    private:
        void SetActiveRobots(string robot_name, const vector<RobotBasePtr >& robots);

        OpenRAVE::RobotBasePtr _probot; // Robot object using in the plugin
        OpenRAVE::EnvironmentBasePtr _penv; // Environment object using in the plugin
        vector<double>_goal; // Goal coordinate for the planning. [x,y,theta]
        bool _is_parallel = false; // a flag to turn or off parallelization. (just for example)
};


void EscherMotionPlanning::SetActiveRobots(string robot_name, const vector<RobotBasePtr>& robots)
{
    if( robots.size() == 0 ) {
        RAVELOG_WARNA("No robots to plan for\n");
        return;
    }

    for(vector<RobotBasePtr>::const_iterator it = robots.begin(); it != robots.end(); it++)
    {
        if( strcmp((*it)->GetName().c_str(), robot_name.c_str() ) == 0  ) {
            _probot = *it;
            break;
        }
    }

    if( _probot == NULL )
    {
        RAVELOG_ERRORA("Failed to find %S\n", robot_name.c_str());
        return;
    }
}

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "eschermotionplanning" ) {
        return InterfaceBasePtr(new EscherMotionPlanning(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("EscherMotionPlanning");
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

