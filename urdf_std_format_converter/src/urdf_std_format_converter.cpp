//
// Created by Lukas on 7/23/19.
//

#include <iostream>
#include <string>
#include <unistd.h>
#include <Eigen/Dense>
#include <urdf/model.h>
#include <tinyxml.h>
#include <urdf_std_format_converter/urdf_std_format_converter.h>


UrdfStdFormatConverter::UrdfStdFormatConverter(std::string inputFilePath) {
    this->robotModel.initFile(inputFilePath);
    createChildJoints();
    createChildLinks();
}

UrdfStdFormatConverter::~UrdfStdFormatConverter() = default;

int UrdfStdFormatConverter::createMatricesFromUrdf() {
    std::cout << "---Previous Link Rotations---" << std::endl;
    for (const auto &link : links) {
        Eigen::Quaterniond quat;
        link->visual->origin.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
        std::cout << link->name << std::endl;
        Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
        temp.block<3, 3>(0, 0) = quat.toRotationMatrix();
        std::cout << rpyVector3toString(quat.toRotationMatrix().eulerAngles(2, 1, 0)) << "\n\n";
        lMatR.push_back(temp);
    }

    std::cout << "---Previous Joint Rotations---" << std::endl;
    for (const auto &joint : joints) {
        if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::FIXED) {
            std::cout << "There is PRISMATIC type of the joint. Do you want to continue? y/n ";
            std::string input;
            std::cin >> input;
            if (input == "n") {
                return 1;
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
        std::cout << rpyVector3toString(quat.toRotationMatrix().eulerAngles(2, 1, 0)) << "\n\n";
//        std::cout << quaternion.vec() << "\n" << quaternion.w() << std::endl;
//        std::cout << temp << std::endl << std::endl;

        jMatR.push_back(temp);
    }

    return 0;
}

void UrdfStdFormatConverter::calculateNewUrdfValues() {
    std::cout << "---New Joint Offsets---" << std::endl;
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < joints.size(); i++) {
        Eigen::Vector4d nullvec;
        nullvec << 0, 0, 0, 1;
        newJointOffsets.emplace_back(((mat * jMatT[i]) * Eigen::Vector4d(0, 0, 0, 1)).head(3));
        std::cout << joints[i]->name << std::endl;
        std::cout << xyzVector3toString(newJointOffsets[i]) << std::endl << std::endl;
        mat *= jMatR[i];
        if (i < joints.size() - 1) {
            newJointRPYs.emplace_back(Eigen::Vector3d::Zero());
        } else {
            std::cout << "---New Tool0 RPY---" << std::endl;
            newJointRPYs.push_back(mat.block<3, 3>(0, 0).eulerAngles(2, 1, 0));
            std::cout << rpyVector3toString(newJointRPYs[i]) << std::endl << std::endl;
        }
    }

    std::cout << "---New Joint Axes---" << std::endl;
    mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < joints.size() - 1; i++) {
        Eigen::Vector4d axis;
        axis = Eigen::Vector4d(joints[i]->axis.x, joints[i]->axis.y, joints[i]->axis.z, 1);

        mat *= jMatR[i];
        newJointAxes.emplace_back((mat * axis).head(3));

        for (int j = 0; j < 3; j++) {
            double absval = std::abs(newJointAxes[i](j));
            if (absval < ROUND_EPS) {
                newJointAxes[i](j) = std::abs(std::round(newJointAxes[i](j)));
            } else if(std::abs(absval - 1) < ROUND_EPS) {
                newJointAxes[i](j) = std::round(newJointAxes[i](j));
            }
        }
        std::cout << joints[i]->name << std::endl;
        std::cout << newJointAxes[i] << std::endl << std::endl;
    }

    std::cout << "---New Link RPYs---" << std::endl;
    mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < links.size(); i++) {
        mat *= jMatR[i];
        newLinkRPYs.push_back((mat * lMatR[i]).block<3, 3>(0, 0).eulerAngles(2, 1, 0));
        std::cout << links[i]->name << std::endl;
        std::cout << rpyVector3toString(newLinkRPYs[i]) << std::endl << std::endl;
    }
}

int UrdfStdFormatConverter::modifyXacroXmlFile(std::string xacroFilePath) {
    xacroXML = TiXmlDocument(xacroFilePath);

    if (xacroXML.LoadFile()) {
        TiXmlElement *xacroNode = xacroXML.FirstChild("robot")->FirstChild("xacro:macro")->ToElement();
        TiXmlElement *ljNode = xacroNode->FirstChildElement();

        std::vector<TiXmlElement *> linkElements;
        std::vector<TiXmlElement *> jointElements;

        std::cout << "---These elements was modified in new Xacro file---" << std::endl;
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

            TiXmlElement originElement("origin");
            originElement.SetAttribute("xyz", "0 0 0");
            originElement.SetAttribute("rpy", rpyVector3toString(newLinkRPYs[i]));

            TiXmlNode *tempNode = lE->FirstChild("visual");
            if (tempNode != nullptr) {
                tempNode->ReplaceChild(tempNode->FirstChild("origin"), originElement);
            } else return 1;

            tempNode = lE->FirstChild("collision");
            if (tempNode != nullptr) {
                tempNode->InsertBeforeChild(tempNode->FirstChild("geometry"), originElement);
            } else return 1;
        }

        // Print and modify joint elements
        for (int i = 0; i < jointElements.size(); i++) {
            auto jE = jointElements[i];
            std::cout << jE->FirstAttribute()->ValueStr() << std::endl;

            std::string axisString = std::to_string((int) newJointAxes[i][0]) + " ";
            axisString += std::to_string((int) newJointAxes[i][1]) + " ";
            axisString += std::to_string((int) newJointAxes[i][2]);

            TiXmlElement originElement("origin");
            originElement.SetAttribute("xyz", xyzVector3toString(newJointOffsets[i]));
            originElement.SetAttribute("rpy", rpyVector3toString(newJointRPYs[i]));

            TiXmlElement axisElement("axis");
            axisElement.SetAttribute("xyz",  axisString);

            jE->ReplaceChild(jE->FirstChild("origin"), originElement);
            jE->ReplaceChild(jE->FirstChild("axis"), axisElement);
        }

        xacroXML.SaveFile(xacroFilePath + ".xacro");

    } else {
        std::cout << "XML file open failed." << std::endl;
        return 1;
    }

    return 0;
}

std::string UrdfStdFormatConverter::rpyVector3toString(Eigen::Vector3d vec) {
    std::string output = "";
    for (int i = 2; i >= 0; i--) {
        if (std::abs(vec[i]) < ROUND_EPS) {
            output += std::to_string((int) std::abs(std::round(vec[i])));
        } else {
            output += std::to_string(vec[i]);
        }
        if (i > 0) {
            output += " ";
        }
    }
    return output;
}

std::string UrdfStdFormatConverter::xyzVector3toString(Eigen::Vector3d vec) {
    std::string output = "";
    for (int i = 0; i <= 2; i++) {
        if (std::abs(vec[i]) < ROUND_EPS) {
            output += std::to_string((int) std::abs(std::round(vec[i])));
        } else {
            output += std::to_string(vec[i]);
        }
        if (i < 2) {
            output += " ";
        }
    }
    return output;
}

int UrdfStdFormatConverter::isConversionNeeded() {
    for (const auto &joint : joints) {
        if (joint->type == urdf::Joint::REVOLUTE)
        {
            Eigen::Matrix4d mat;
            Eigen::Quaterniond quat;
            Eigen::Vector3d vec;
            joint->parent_to_joint_origin_transform.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
            mat = Eigen::Matrix4d::Identity();
            mat.block<3, 3>(0, 0) = quat.toRotationMatrix();
            vec = quat.toRotationMatrix().eulerAngles(2, 1, 0);
            for(int i = 0; i<=2; i++) {
                if(std::abs(vec[i]) > ROUND_EPS) {
                    return 1;
                }
            }
        }
    }
    return 0;
}

int UrdfStdFormatConverter::existLinkOffsets() {
    for (auto link : links) {
        double sum = 0;
        sum = std::abs(link->visual->origin.position.x);
        sum += std::abs(link->visual->origin.position.y);
        sum += std::abs(link->visual->origin.position.z);
        if(sum > ROUND_EPS) {
            return 1;
        }
    }
    return 0;
}

std::vector<Eigen::Vector3d> UrdfStdFormatConverter::getNewJointAxes() {
    return newJointAxes;
}

std::vector<Eigen::Vector3d> UrdfStdFormatConverter::getNewJointOffsets() {
    return newJointOffsets;
}

std::vector<Eigen::Vector3d> UrdfStdFormatConverter::getNewJointRPYs() {
    return newJointRPYs;
}

std::vector<Eigen::Vector3d> UrdfStdFormatConverter::getNewLinkRPYs() {
    return newLinkRPYs;
}

void UrdfStdFormatConverter::createChildJoints() {
    urdf::LinkConstSharedPtr actual_link = robotModel.getRoot();
    for (int i = 0; i < robotModel.joints_.size(); i++) {
        joints.push_back(actual_link->child_joints[0]);
        actual_link = actual_link->child_links[0];
    }
}

void UrdfStdFormatConverter::createChildLinks() {
    urdf::LinkConstSharedPtr actual_link = robotModel.getRoot();
    for (int i = 0; i < robotModel.links_.size() - 2; i++) {
        links.push_back(actual_link->child_links[0]);
        actual_link = actual_link->child_links[0];
    }
}