//
// Created by Lukas on 7/31/19.
//

#include <iostream>
#include <string>
#include <unistd.h>
#include <urdf_std_format_converter/urdf_std_format_converter.h>

int main(int argc, char **argv) {
    std::string cmd;
    std::string robotPath;
    std::string robot;

    if (argc == 2) {
        std::cout << "Robot: " << argv[1] << std::endl;
        robot = argv[1];
    } else {
        std::cout << "Please enter robot name as the first argument." << std::endl;
        return 1;
    }

    robotPath = "/home/controller/catkin_ws/src/" + robot + "_support";

    cmd = "cd " + robotPath + "/urdf && ";
    cmd += "rosrun xacro xacro --inorder -o robot.urdf robot.xacro";
    std::cout << cmd << std::endl;
    if (system(cmd.c_str())) {
        std::cout << "Xacro to URDF conversion failed." << std::endl;
        return 1;
    }

    UrdfStdFormatConverter urdfConverter(robotPath + "/urdf/robot.urdf");

    if (urdfConverter.existLinkOffsets() == 1) {
        std::cout << "URDF model has some offsets of the links. Conversion is not prepared for it." << std::endl;
        return 4;
    }

    if (urdfConverter.isConversionNeeded() == 0) {
        std::cout << "URDF model does not have any rotation of the joints. Conversion is not needed." << std::endl;
        return 2;
    }

    if (urdfConverter.createMatricesFromUrdf() == 1) {
        return 3;
    }

    urdfConverter.calculateNewUrdfValues();

    urdfConverter.modifyXacroXmlFile(robotPath + "/urdf/robot_macro.xacro");

    std::cout << "Robot URDF model conversion was successful." << std::endl;
    return 0;
}

