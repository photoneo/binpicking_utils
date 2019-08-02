//
// Created by controller on 7/31/19.
//

#ifndef PROJECT_URDF_STD_FORMAT_CONVERTER_H
#define PROJECT_URDF_STD_FORMAT_CONVERTER_H

#include <iostream>
#include <urdf/model.h>
#include <Eigen/Dense>

#define ROUND_EPS 0.00001

class UrdfStdFormatConverter{
public:
    explicit UrdfStdFormatConverter(std::string inputFilePath);
    ~UrdfStdFormatConverter();
    int createMatricesFromUrdf();
    void calculateNewUrdfValues();
    std::vector<Eigen::Vector3d> getNewJointRPYs();
    std::vector<Eigen::Vector3d> getNewJointOffsets();
    std::vector<Eigen::Vector3d> getNewJointAxes();
    std::vector<Eigen::Vector3d> getNewLinkRPYs();
    int modifyXacroXmlFile(std::string xacroFilePath);
    int isConversionNeeded();
    int existLinkOffsets();

private:
    urdf::Model robotModel;
    std::vector<urdf::JointSharedPtr> joints;
    std::vector<urdf::LinkSharedPtr> links;
    std::vector<Eigen::Matrix4d> lMatR; // Link Rotational matrices
    std::vector<Eigen::Matrix4d> jMatR; // Joint Rotational matrices
    std::vector<Eigen::Matrix4d> jMatT; // Joint Translational matrices
    std::vector<Eigen::Vector3d> newJointRPYs;
    std::vector<Eigen::Vector3d> newJointOffsets;
    std::vector<Eigen::Vector3d> newJointAxes;
    std::vector<Eigen::Vector3d> newLinkRPYs;
    TiXmlDocument xacroXML;
    void createChildJoints();
    void createChildLinks();
    std::string rpyVector3toString(Eigen::Vector3d vec);
    std::string xyzVector3toString(Eigen::Vector3d vec);
};


#endif //PROJECT_URDF_STD_FORMAT_CONVERTER_H
