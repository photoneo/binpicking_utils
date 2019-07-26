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

    std::vector<Eigen::Matrix4d> JMatR; // Joint Rotational matrices
    std::vector<Eigen::Matrix4d> JMatT; // Joint Translational matrices
    for (const auto &joint : joints) {
        double x = joint->parent_to_joint_origin_transform.position.x;
        double y = joint->parent_to_joint_origin_transform.position.y;
        double z = joint->parent_to_joint_origin_transform.position.z;
        std::cout << joint->name << std::endl;
        Eigen::Matrix4d temp;
        temp << Eigen::Matrix3d::Identity(), Eigen::Vector3d(x, y, z),
                0, 0, 0, 1;
//        std::cout << temp << std::endl;
        JMatT.push_back(temp);


        Eigen::Quaterniond quat;
        joint->parent_to_joint_origin_transform.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
        temp = Eigen::Matrix4d::Identity();
        temp.block<3, 3>(0, 0) = quat.toRotationMatrix();
        std::cout << quat.toRotationMatrix().eulerAngles(2, 1, 0) << "\n\n";
//        std::cout << quaternion.vec() << "\n" << quaternion.w() << std::endl;
//        std::cout << temp << std::endl << std::endl;

        JMatR.push_back(temp);
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
        newJointOffsets.emplace_back(((mat * JMatT[i]) * Eigen::Vector4d(0, 0, 0, 1)).head(3));
        std::cout << joints[i]->name << std::endl;
        std::cout << newJointOffsets[i] << std::endl << std::endl;
        mat *= JMatR[i];
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

        mat *= JMatR[i];
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
        mat *= JMatR[i];
        newLinkRPYs.push_back((mat * LMatR[i]).block<3, 3>(0, 0).eulerAngles(2, 1, 0));
        std::cout << links[i]->name << std::endl;
        std::cout << newLinkRPYs[i] << std::endl << std::endl;
    }

    //TiXmlDocument XacroXML = TiXmlDocument("/home/controller/catkin_ws/src/" + robot + "/" + robot + "_support/urdf/robot_macro.xacro");
    //XacroXML.LoadFile("/home/controller/catkin_ws/src/" + robot + "/" + robot + "_support/urdf/robot_macro.xacro.urdf");
    //std::cout << XacroXML << std::endl;
}