//
// Created by Lukas on 7/23/19.
//

#include <iostream>
#include <string>
#include <unistd.h>
#include <Eigen/Dense>
#include <urdf/model.h>
//#include "../../../../../../../opt/ros/kinetic/include/urdf/model.h"


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
};

class Matrix4x4d : public Eigen::Matrix4d {
public:
    static Eigen::Matrix4d RotMat_X(double a){
        Eigen::Matrix4d mat;
        mat <<  1, 0, 0, 0,
                0, std::cos(a), -std::sin(a), 0,
                0, std::sin(a), std::cos(a), 0,
                0, 0, 0, 1;
        return mat;
    }

    static Eigen::Matrix4d RotMat_Y(double a){
        Eigen::Matrix4d mat;
        mat <<  std::cos(a), 0, std::sin(a), 0,
                0, 1, 0, 0,
                -std::sin(a), 0, std::cos(a), 0,
                0, 0, 0, 1;
        return mat;
    }

    static Eigen::Matrix4d RotMat_Z(double a){
        Eigen::Matrix4d mat;
        mat <<  std::cos(a), -std::sin(a), 0, 0,
                std::sin(a), std::cos(a), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        return mat;
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

    std::vector<Eigen::Matrix4d> MatR; // Rotational matrices
    std::vector<Eigen::Matrix4d> MatT; // Translational matrices
    for (int i = 0; i < joints.size(); i++) {
        auto joint = joints[i];
        //if (joint->type != urdf::Joint::REVOLUTE) continue;

        double x = joint->parent_to_joint_origin_transform.position.x;
        double y = joint->parent_to_joint_origin_transform.position.y;
        double z = joint->parent_to_joint_origin_transform.position.z;
        std::cout << joint->name << std::endl;
        Eigen::Matrix4d temp;
        temp << Eigen::Matrix3d::Identity(), Eigen::Vector3d(x, y, z),
                0, 0, 0, 1;
//        std::cout << temp << std::endl;
        MatT.push_back(temp);


       /* Eigen::Quaterniond quaternion = (Eigen::Quaterniond(joint->parent_to_joint_origin_transform.rotation.w, \
        joint->parent_to_joint_origin_transform.rotation.x, \
        joint->parent_to_joint_origin_transform.rotation.y, \
        joint->parent_to_joint_origin_transform.rotation.z));*/
        Eigen::Quaterniond quaternion;
        joint->parent_to_joint_origin_transform.rotation.getQuaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        temp = Eigen::Matrix4d::Identity();
        temp.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
        std::cout << quaternion.toRotationMatrix().eulerAngles(0, 1, 2) << "\n\n";
        std::cout << quaternion.vec() << "\n" << quaternion.w() << std::endl;
//        std::cout << temp << std::endl << std::endl;

//        joint->parent_to_joint_origin_transform.rotation.getRPY(x, y, z);
//        std::cout << x << std::endl;
//        std::cout << y << std::endl;
//        std::cout << z << std::endl;
//        std::cout << Matrix4x4d::RotMat_X(x) * Matrix4x4d::RotMat_Y(y) * Matrix4x4d::RotMat_Z(z) << std::endl;
//        temp = Matrix4x4d::RotMat_X(x) * Matrix4x4d::RotMat_Y(y) * Matrix4x4d::RotMat_Z(z);
        MatR.push_back(temp);
    }

    std::cout << "---Joint RPYs---" << std::endl;
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < joints.size(); i++) {
        Eigen::Vector4d nullvec;
        nullvec << 0, 0, 0, 1;
        std::cout << joints[i]->name << std::endl;
        std::cout << ((mat * MatT[i]) * nullvec).head(3) << std::endl << std::endl;
        if (i == joints.size() - 1) {
            Eigen::Matrix4d tool0 = mat.inverse()*MatR[i];
            std::cout << tool0.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;
        }
        mat = mat * MatR[i];
    }

    std::cout << "---Joint Axis---" << std::endl;
    mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < joints.size() - 1; i++) {
        Eigen::Vector4d axis;
        axis = Eigen::Vector4d(joints[i]->axis.x, joints[i]->axis.y, joints[i]->axis.z, 1);

        mat = mat * MatR[i];
        std::cout << joints[i]->name << std::endl;
        std::cout << (mat * axis).head(3) << std::endl << std::endl;
    }


//    Eigen::Matrix<double, 4, 4> mat(Eigen::Matrix4d::Identity());
//    Eigen::Matrix4d mat2 = Eigen::Matrix4d::Identity();
//
//
//    mat * mat2;
//    mat = Eigen::Matrix4d::Identity();
//    Eigen::Vector4d vec = Eigen::Vector4d::Ones();
//    mat * vec;
//    std::cout << mat << std::endl;
//    std::cout << "Hello world" << std::endl;
}