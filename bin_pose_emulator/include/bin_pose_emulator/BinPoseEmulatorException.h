//
// Created by controller on 8/12/19.
//

#ifndef PROJECT_BINPOSEEMULATOREXCEPTION_H
#define PROJECT_BINPOSEEMULATOREXCEPTION_H

#include <exception>
#include <string>
#include <ros/console.h>

/**
 * \class BinPickingException BinPickingException.h BinPickingException.cpp
 *
 * \brief BinPickingException is base class of all binpicking exceptions
 *
 * \author Matej Bartosovic, bartosovic@photoneo.com
 *
 * \version 1.0
 *
 * \date 23.10.2018
 *
 */
class BinPoseEmulatorException : public std::exception {
public:
    BinPoseEmulatorException(std::string message) : message(message){
        ROS_ERROR_STREAM(message);
    }
    virtual const char* what() const throw(){
        return message.c_str();
    }
private:
    std::string message;
};

/**
 * \class FailedToIntializeWaypointGenerator BinPickingException.h BinPickingException.cpp
 *
 * \brief FailedToIntializeWaypointGenerator exception is thrown when initialization form parameter server fail.
 *
 * \author Matej Bartosovic, bartosovic@photoneo.com
 *
 * \date 26.11.2018
 *
 */
class ResourceNotFound : public BinPoseEmulatorException{
public:
    ResourceNotFound(std::string resourcePath) : BinPoseEmulatorException("Resource " + resourcePath + " not found"){

    }
};

#endif //PROJECT_BINPOSEEMULATOREXCEPTION_H
