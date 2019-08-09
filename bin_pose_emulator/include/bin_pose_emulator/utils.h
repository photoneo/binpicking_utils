//
// Created by controller on 8/9/19.
//

#ifndef PROJECT_UTILS_H
#define PROJECT_UTILS_H

#include <ros/console.h>
#include <rosconsole/macros_generated.h>
#define PARAMETER_LOG_NAME "param"


#define GET_PARAM_LOGGED(nodeHandle,paramName,paramVariable,defaultValue) \
    do{ \
        bool ret = nodeHandle.param(paramName,paramVariable,defaultValue);\
        ROS_WARN_COND_NAMED(!ret,PARAMETER_LOG_NAME,"%s",(std::string(paramName) + " not found, using default").c_str());\
        phoLog::printParam(paramName,paramVariable);\
    }while(false)

#define GET_PARAM_REQUIRED(nodeHandle,paramName,paramVariable) \
    if(!nodeHandle.getParam(paramName,paramVariable)){\
        ROS_FATAL_NAMED(PARAMETER_LOG_NAME,"%s",(std::string("Failed to get required parameter: ") + paramName).c_str());\
        std::terminate();\
    }


#endif //PROJECT_UTILS_H
