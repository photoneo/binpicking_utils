//
// Created by Lukas on 7/31/19.
//

#include <iostream>
#include <string>
#include <unistd.h>
#include <urdf_std_format_converter/urdf_std_format_converter.h>

int main(int argc, char **argv) {
    std::string cmd;
    std::string robot;

    if(argc == 2){
        std::cout << "Robot: " << argv[1] << std::endl;
    } else {
        std::cout << "Please enter robot name as the first argument." << std::endl;
        return 1;
    }
    robot = argv[1];

    cmd = "cd ~/catkin_ws/src/" + robot + "/" + robot + "_support/urdf && ";
    cmd += "rosrun xacro xacro --inorder -o robot.urdf robot.xacro";
    std::cout << cmd << std::endl;
    if (system(cmd.c_str())) {
        std::cout << "Xacro to URDF conversion failed." << std::endl;
        return 1;
    }

    UrdfStdFormatConverter urdfConverter("/home/controller/catkin_ws/src/" + robot + "/" + robot + "_support/urdf/robot.urdf");
    if(urdfConverter.isConversionNeeded() == 0) {
        std::cout << "URDF model does not have any rotation of the joints. Conversion is not needed." << std::endl;
        return 2;
    }

    if(urdfConverter.createMatricesFromUrdf() == 1) {
        return 3;
    }

    urdfConverter.calculateNewUrdfValues();

    urdfConverter.modifyXacroXmlFile("/home/controller/catkin_ws/src/" + robot + "/" + robot + \
            "_support/urdf/robot_macro.xacro");

    std::cout << "Robot URDF model conversion was successful." << std::endl;
    return 0;
}

