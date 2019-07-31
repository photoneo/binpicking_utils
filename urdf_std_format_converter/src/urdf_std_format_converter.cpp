//
// Created by Lukas on 7/23/19.
//

#include <iostream>
#include <string>
#include <unistd.h>
#include <Eigen/Dense>
#include <urdf/model.h>
#include <tinyxml.h>


class RobotModel : public urdf::Model {
//    using urdf::Model;
public:
    std::vector<urdf::JointSharedPtr> getChildJoints() {
        std::vector<urdf::JointSharedPtr> joints;
        urdf::LinkConstSharedPtr actual_link = getRoot();
        for (int i = 0; i < joints_.size(); i++) {
            joints.push_back(actual_link->child_joints[0]);
            actual_link = actual_link->child_links[0];
        }
        return joints;
    }

    std::vector<urdf::LinkSharedPtr> getChildLinks() {
        std::vector<urdf::LinkSharedPtr> links;
        urdf::LinkConstSharedPtr actual_link = getRoot();
        for (int i = 0; i < links_.size() - 2; i++) {
            links.push_back(actual_link->child_links[0]);
            actual_link = actual_link->child_links[0];
        }
        return links;
    }
};

int main(int argc, char **argv) {
    std::string cmd;
    std::string robot;
    robot = "abb_irb1520id_4_150";
    cmd = "cd ~/catkin_ws/src/" + robot + "/" + robot + "_support/urdf && ";
    cmd += "rosrun xacro xacro --inorder -o robot.urdf robot.xacro";
    std::cout << cmd << std::endl;
    if (system(cmd.c_str())) {
        std::cout << "Xacro to URDF conversion failed." << std::endl;
        return 1;
    }

    RobotModel model;
    model.initFile("/home/controller/catkin_ws/src/" + robot + "/" + robot + "_support/urdf/robot.urdf");

    auto joints = model.getChildJoints();
    auto links = model.getChildLinks();

    std::vector<Eigen::Matrix4d> LMatR; // Link Rotational matrices
    for (const auto &link : links) {
        Eigen::Quaterniond quat;
        link->visual->origin.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
        std::cout << link->name << std::endl;
        Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
        temp.block<3, 3>(0, 0) = quat.toRotationMatrix();
        std::cout << quat.toRotationMatrix().eulerAngles(2, 1, 0) << "\n\n";
        LMatR.push_back(temp);
    }

    std::vector<Eigen::Matrix4d> jMatR; // Joint Rotational matrices
    std::vector<Eigen::Matrix4d> jMatT; // Joint Translational matrices
    for (const auto &joint : joints) {
        if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::FIXED) {
            std::cout << "There is PRISMATIC type of the joint. Do you want to continue? y/n ";
            std::string input;
            std::cin >> input;
            if(input == "n") {
                return 2;
            }
        }
        double x = joint->parent_to_joint_origin_transform.position.x;
        double y = joint->parent_to_joint_origin_transform.position.y;
        double z = joint->parent_to_joint_origin_transform.position.z;
        std::cout << joint->name << std::endl;
        Eigen::Matrix4d temp;
        temp << Eigen::Matrix3d::Identity(), Eigen::Vector3d(x, y, z),
                0, 0, 0, 1;
//        std::cout << temp << std::endl;
        jMatT.push_back(temp);


        Eigen::Quaterniond quat;
        joint->parent_to_joint_origin_transform.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
        temp = Eigen::Matrix4d::Identity();
        temp.block<3, 3>(0, 0) = quat.toRotationMatrix();
        std::cout << quat.toRotationMatrix().eulerAngles(2, 1, 0) << "\n\n";
//        std::cout << quaternion.vec() << "\n" << quaternion.w() << std::endl;
//        std::cout << temp << std::endl << std::endl;

        jMatR.push_back(temp);
    }

    std::vector<Eigen::Vector3d> newJointRPYs;
    std::vector<Eigen::Vector3d> newJointOffsets;
    std::vector<Eigen::Vector3d> newJointAxes;
    std::vector<Eigen::Vector3d> newLinkRPYs;

    std::cout << "---Joint Offsets---" << std::endl;
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < joints.size(); i++) {
        Eigen::Vector4d nullvec;
        nullvec << 0, 0, 0, 1;
        newJointOffsets.emplace_back(((mat * jMatT[i]) * Eigen::Vector4d(0, 0, 0, 1)).head(3));
        std::cout << joints[i]->name << std::endl;
        std::cout << newJointOffsets[i] << std::endl << std::endl;
        mat *= jMatR[i];
        if (i < joints.size() - 1) {
            newJointRPYs.emplace_back(Eigen::Vector3d::Zero());
        } else {
            newJointRPYs.push_back(mat.block<3, 3>(0, 0).eulerAngles(2, 1, 0));
            std::cout << newJointRPYs[i] << std::endl;
//            std::cout << mat << std::endl;
        }
    }

    std::cout << "---Joint Axes---" << std::endl;
    mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < joints.size() - 1; i++) {
        Eigen::Vector4d axis;
        axis = Eigen::Vector4d(joints[i]->axis.x, joints[i]->axis.y, joints[i]->axis.z, 1);

        mat *= jMatR[i];
        newJointAxes.emplace_back((mat * axis).head(3));

        double eps = 0.01;
        for (int j = 0; j < 3; j++) {
            double absval = std::round(newJointAxes[i](j));
            if ((absval < eps) || (std::abs(absval - 1) < eps)) {
                newJointAxes[i](j) = round(newJointAxes[i](j));
            }
        }
        newJointAxes[i](0) = std::round(newJointAxes[i](0));
        std::cout << joints[i]->name << std::endl;
        std::cout << newJointAxes[i] << std::endl << std::endl;
    }

    std::cout << "---Link RPYs---" << std::endl;
    mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < links.size(); i++) {
        mat *= jMatR[i];
        newLinkRPYs.push_back((mat * LMatR[i]).block<3, 3>(0, 0).eulerAngles(2, 1, 0));
        std::cout << links[i]->name << std::endl;
        std::cout << newLinkRPYs[i] << std::endl << std::endl;
    }

    TiXmlDocument xacroXML = TiXmlDocument("/home/controller/catkin_ws/src/" + robot + "/" + robot + \
            "_support/urdf/robot_macro.xacro");
    if (xacroXML.LoadFile()) {
        TiXmlElement *xacroNode = xacroXML.FirstChild("robot")->FirstChild("xacro:macro")->ToElement();
        TiXmlElement *ljNode = xacroNode->FirstChildElement();

        std::vector<TiXmlElement *> linkElements;
        std::vector<TiXmlElement *> jointElements;

        // Prepare vectors of link and joint elements
        while (ljNode) {
            std::string ljName = ljNode->FirstAttribute()->ValueStr();
            if (ljName.find("link_") != std::string::npos) {
                int i = ljName[ljName.find("link_") + 5] - '0';
                if (i == linkElements.size() + 1) {
                    linkElements.push_back(ljNode);
                }
            } else if (ljName.find("joint_") != std::string::npos) {
                int i = ljName[ljName.find("joint_") + 6] - '0';
                if (i == jointElements.size() + 1) {
                    jointElements.push_back(ljNode);
                } else if (ljName.find("tool0") != std::string::npos) {
                    jointElements.push_back(ljNode);
                }
            }
            ljNode = ljNode->NextSiblingElement();
        }

        // Print and modify link elements
        for (int i = 0; i < linkElements.size(); i++) {
            auto lE = linkElements[i];
            std::cout << lE->FirstAttribute()->ValueStr() << std::endl;

            std::string rpyString = std::to_string(newLinkRPYs[i][2]) + " ";
            rpyString += std::to_string(newLinkRPYs[i][1]) + " ";
            rpyString += std::to_string(newLinkRPYs[i][0]);
            TiXmlElement originElement("origin");
            originElement.SetAttribute("xyz", "0 0 0");
            originElement.SetAttribute("rpy", rpyString);

            TiXmlNode* tempNode = lE->FirstChild("visual");
            if (tempNode != nullptr) {
                tempNode->ReplaceChild(tempNode->FirstChild("origin"),originElement);
            } else return 1;

            tempNode = lE->FirstChild("collision");
            if (tempNode != nullptr) {
                tempNode->InsertBeforeChild(tempNode->FirstChild("geometry"),originElement);
            } else return 1;
        }

        // Print and modify joint elements
        for (int i = 0; i < jointElements.size(); i++) {
            auto jE = jointElements[i];
            std::cout << jE->FirstAttribute()->ValueStr() << std::endl;

            std::string xyzString = std::to_string(newJointOffsets[i][0]) + " ";
            xyzString += std::to_string(newJointOffsets[i][1]) + " ";
            xyzString += std::to_string(newJointOffsets[i][2]);

            std::string rpyString = std::to_string(newJointRPYs[i][0]) + " ";
            rpyString += std::to_string(newJointRPYs[i][1]) + " ";
            rpyString += std::to_string(newJointRPYs[i][2]);

            std::string axisString = std::to_string((int)newJointAxes[i][0]) + " ";
            axisString += std::to_string((int)newJointAxes[i][1]) + " ";
            axisString += std::to_string((int)newJointAxes[i][2]);

            TiXmlElement originElement("origin");
            originElement.SetAttribute("xyz", xyzString);
            originElement.SetAttribute("rpy", rpyString);

            TiXmlElement axisElement("axis");
            axisElement.SetAttribute("xyz", axisString);

            jE->ReplaceChild(jE->FirstChild("origin"), originElement);
            jE->ReplaceChild(jE->FirstChild("axis"), axisElement);
        }

        xacroXML.SaveFile("/home/controller/catkin_ws/src/" + robot + "/" + robot + \
            "_support/urdf/robot_macro_mod.xacro");

    } else {
        std::cout << "XML file open failed." << std::endl;
    }

}