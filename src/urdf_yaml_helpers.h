#ifndef URDF_JOINT_LIST_H
#define URDF_JOINT_LIST_H

#include <yaml-cpp/yaml.h>

namespace or_urdf
{

    void operator >> (const YAML::Node& node, std::map<std::string, int>& jmap){
        
         for(unsigned int i=0; i< node.size(); i++){

            const YAML::Node& n = node[i];
            std::string joint_name;
            int joint_index;
            n["name"] >> joint_name;
            n["index"] >> joint_index;
            
            jmap[joint_name] = joint_index;
        }
    }

}

#endif
