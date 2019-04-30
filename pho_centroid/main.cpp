#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_base.h>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>

#define DENSITY 1 //mm

class minMaxXYZ{
public:
    double Xmin;
    double Xmax;
    double Ymin;
    double Ymax;
    double Zmin;
    double Zmax;
    minMaxXYZ() = default;
    minMaxXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pPcl);
    void begin(pcl::PointXYZ &point);
    void roundValues();
};

void subsamplePcl(const pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr pSampledPcl);

std::ostream& operator<<(std::ostream& os, const minMaxXYZ& obj);

void roundPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pInPcl, pcl::PointCloud<pcl::PointXYZ>::Ptr pOutPcl);

void fillPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pInPcl, pcl::PointCloud<pcl::PointXYZ>::Ptr pOutPcl);

pcl::PointXYZ getCenterOfPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pInPcl);

int main() {
    using namespace std::chrono_literals;
    // Load PLY file as a PolygonMesh
    pcl::PolygonMesh mesh;
    //std::string plyFile = "/home/controller/catkin_ws/model-T-fitting.ply";
    std::string plyFile =   "/home/controller/catkin_ws/valcek_MTS.stl";
    //pcl::io::loadPLYFile(plyFile, mesh);
    pcl::io::loadPolygonFileSTL(plyFile, mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrPcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pRoundedPcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pFilledPcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSampledPcl(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(mesh.cloud, *ptrPcl);

    std::cout << "Centroid before : " << getCenterOfPointCloud(ptrPcl) << "\n";

    subsamplePcl(mesh, pSampledPcl);

    roundPointCloud(pSampledPcl, pRoundedPcl);

    // fillPointCloud(pRoundedPcl, pFilledPcl);
    auto centroid = getCenterOfPointCloud(pSampledPcl);
    pRoundedPcl->push_back(centroid);

    std::cout << "Centroid after sampling: " << centroid << "\n";


#ifndef NDEBUG
    pcl::visualization::CloudViewer pclViewer("Test");
    pclViewer.showCloud(ptrPcl);
    std::this_thread::sleep_for(5s);
    pclViewer.showCloud(pSampledPcl);
    std::this_thread::sleep_for(5s);
    pclViewer.showCloud(pRoundedPcl);
    while(!pclViewer.wasStopped());
#endif //NDEBUG

    return EXIT_SUCCESS;

}

void minMaxXYZ::begin(pcl::PointXYZ &point) {
    Xmax = point.x;
    Xmin = point.x;
    Ymax = point.y;
    Ymin = point.y;
    Zmax = point.z;
    Zmin = point.z;
}

minMaxXYZ::minMaxXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pPcl) {
    auto point = pPcl->points.front();
    Xmax = point.x;
    Xmin = point.x;
    Ymax = point.y;
    Ymin = point.y;
    Zmax = point.z;
    Zmin = point.z;

    for(pcl::PointXYZ point:pPcl->points){
        double x = point.x;
        double y = point.y;
        double z = point.z;

        if(x < Xmin){
            Xmin = x;
        }else if(x > Xmax){
            Xmax = x;
        }
        if(y < Ymin){
            Ymin = y;
        }else if(y > Ymax){
            Ymax = y;
        }
        if(z < Zmin){
            Zmin = z;
        }else if(z > Zmax){
            Zmax = z;
        }
    }
}

void minMaxXYZ::roundValues() {
    using std::round;
    Xmax = round(Xmax);
    Xmin = round(Xmin);
    Ymax = round(Ymax);
    Ymin = round(Ymin);
    Zmax = round(Zmax);
    Zmin = round(Zmin);
}

std::ostream& operator<<(std::ostream& os, const minMaxXYZ& obj){
    os << "Xmin: " << obj.Xmin << ", Xmax: " << obj.Xmax << ", Ymin: " << obj.Ymin << ", Ymax: " << obj.Ymax << ", Zmin: " << obj.Zmin << ", Zmax: " << obj.Zmax;
    return os;
}

void roundPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pInPcl, pcl::PointCloud<pcl::PointXYZ>::Ptr pOutPcl) {
    using std::round;
    for(pcl::PointXYZ inPoint:pInPcl->points){
        pOutPcl->push_back(pcl::PointXYZ(round(inPoint.x), round(inPoint.y), round(inPoint.z)));
    }
}

double average(const std::vector<double> &vec){
    double sum = 0;
    int n = 0;
    for(auto elem:vec){
        sum += elem;
        n++;
    }
    return sum/n;
}

pcl::PointXYZ getCenterOfPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pInPcl) {
    std::vector<double> vX;
    std::vector<double> vY;
    std::vector<double> vZ;
    pcl::PointXYZ centroid;

    for(pcl::PointXYZ point:*pInPcl){
        vX.push_back(point.x);
        vY.push_back(point.y);
        vZ.push_back(point.z);
    }
    centroid.x = average(vX);
    centroid.y = average(vY);
    centroid.z = average(vZ);

    return centroid;
}

void fillPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pInPcl, pcl::PointCloud<pcl::PointXYZ>::Ptr pOutPcl) {
    minMaxXYZ pclMinMaxes(pInPcl);
    pclMinMaxes.roundValues();

    pcl::PointXYZ start(pclMinMaxes.Xmin - 1, pclMinMaxes.Ymin - 1, pclMinMaxes.Zmin - 1);
    pcl::PointXYZ end(pclMinMaxes.Xmax + 1, pclMinMaxes.Ymax + 1, pclMinMaxes.Zmax + 1);

    std::vector<std::vector<cv::Point2f> > contours;

    for(float z = start.z; z <= end.z; z += DENSITY){
        // Slicing throught model, TODO Finish
    }
}

void swap(pcl::PointXYZ &point1, pcl::PointXYZ &point2){
    pcl::PointXYZ temp(point1);
    point1 = point2;
    point2 = temp;
}

pcl::PointXYZ operator- (const pcl::PointXYZ &lh, const pcl::PointXYZ &rh){
    pcl::PointXYZ sub;
    sub.x = lh.x - rh.x;
    sub.y = lh.y - rh.y;
    sub.z = lh.z - rh.z;
    return sub;
}

pcl::PointXYZ operator+ (const pcl::PointXYZ &lh, const pcl::PointXYZ &rh){
    pcl::PointXYZ add;
    add.x = lh.x + rh.x;
    add.y = lh.y + rh.y;
    add.z = lh.z + rh.z;
    return add;
}

double absLen(const pcl::PointXYZ &vec){
    return pow( ((vec.x*vec.x) + (vec.y*vec.y) + (vec.z*vec.z)), 0.5);
}

pcl::PointXYZ operator* (const pcl::PointXYZ &lh, const double &rh){
    pcl::PointXYZ mul;
    mul.x = lh.x * rh;
    mul.y = lh.y * rh;
    mul.z = lh.z * rh;
    return mul;
}

pcl::PointXYZ operator/ (const pcl::PointXYZ &lh, const double &rh){
    pcl::PointXYZ mul;
    mul.x = lh.x / rh;
    mul.y = lh.y / rh;
    mul.z = lh.z / rh;
    return mul;
}



void subsampleVector(const unsigned int &indice1, const unsigned int &indice2, pcl::PointCloud<pcl::PointXYZ> &originalPcl, pcl::PointCloud<pcl::PointXYZ>::Ptr pSampledPcl){
    pcl::PointXYZ point1 = originalPcl.at(indice1);
    pcl::PointXYZ point2 = originalPcl.at(indice2);
    if(point1.z > point2.z){
        swap(point1, point2);
    }
    // point2 now has higher Z coordinate
    pcl::PointXYZ R = point2 - point1;

    // TODO let it be flexible precision
    double lenVec = absLen(R);
    if(lenVec > 2*DENSITY){
        // vector is long, needs subsambling:
        for(int n=0; n<lenVec; n++){
            pcl::PointXYZ An = point1 + ((R / lenVec) * n);
            pSampledPcl->push_back(An);
        }
    }
    pSampledPcl->push_back(point1);
    pSampledPcl->push_back(point2);
}

void subsamplePcl(const pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr pSampledPcl) {
    pcl::PointCloud<pcl::PointXYZ> originalPcl;
    pcl::fromPCLPointCloud2(mesh.cloud, originalPcl);
    auto polygons = mesh.polygons; // = vector<Vertices> = vector<vector<unsigned int>>
    for(auto polygon:polygons){
        subsampleVector(polygon.vertices.at(0), polygon.vertices.at(1), originalPcl, pSampledPcl);
        subsampleVector(polygon.vertices.at(1), polygon.vertices.at(2), originalPcl, pSampledPcl);
        subsampleVector(polygon.vertices.at(0), polygon.vertices.at(2), originalPcl, pSampledPcl);
    }

}
