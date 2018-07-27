#pragma warning(disable: 4996)

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
// Assimp
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
// Boost
#include <boost/algorithm/string.hpp>



void insertBeeModels(std::vector<std::string> &fileNames) {
    fileNames.clear();
    for (int i = 1; i <= 8; i++) {
        std::stringstream s;
        s << "bee" << i << ".obj";
        fileNames.push_back(s.str());
    }
}

void insertBunnyModels(std::vector<std::string> &fileNames) {
    fileNames.clear();
    for (int i = 1; i <= 10; i++) {
        std::stringstream s;
        s << "bunny" << std::setw(2) << std::setfill('0') << i << ".obj";
        fileNames.push_back(s.str());
    }
}

void importPointsFromPCD(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &models, const std::string filePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr m(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(filePath, *m);
    models.push_back(m);
    return;
}

void importPointsFromOBJ(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &models, const std::string filePath) {
    const struct aiScene* scene = aiImportFile(filePath.c_str(), aiProcessPreset_TargetRealtime_Quality);
    if (scene == NULL) {
        std::cerr << "Could not load mesh from file " << filePath << std::endl;
        exit(-1);
    }
    if (scene->mNumMeshes>1) {
        std::cerr << "Object " << filePath << " consists of more than one mesh." << std::endl;
        exit(-1);
    }
    unsigned numVertices = scene->mMeshes[0]->mNumVertices;
    std::cout << "Read " << numVertices << " vertices" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m(new pcl::PointCloud<pcl::PointXYZ>);
    m->width = numVertices;
    m->height = 1;
    m->is_dense = false;
    m->points.resize(m->width * m->height);

    for (size_t i = 0; i < numVertices; i++) {
        m->points[i].x = scene->mMeshes[0]->mVertices[i].x;
        m->points[i].y = scene->mMeshes[0]->mVertices[i].y;
        m->points[i].z = scene->mMeshes[0]->mVertices[i].z;
    }
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(*m, *m, indices);
    models.push_back(m);

    return;
}

void registration(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &models) {
    int len = models.size();
    for (int i = 0; i < len - 1; i++) {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputCloud(models[i]);
        icp.setInputTarget(models[i + 1]);

        //// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        //icp.setMaxCorrespondenceDistance(0.05);
        //// Set the maximum number of iterations (criterion 1)
        //icp.setMaximumIterations(50);
        //// Set the transformation epsilon (criterion 2)
        //icp.setTransformationEpsilon(1e-8);
        //// Set the euclidean distance difference epsilon (criterion 3)
        //icp.setEuclideanFitnessEpsilon(1);
        
        pcl::PointCloud<pcl::PointXYZ> final;
        icp.align(final);
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << std::endl;
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        std::cout << transformation << std::endl;
        int a = 0;
    }
    
}

void reconstruction() {}


int main(int argc, char **argv) {
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> models;
    std::string filePath = "../obj_models/multi/";
    std::vector<std::string> fileNames;
    
    fileNames.push_back("point_cloud001.pcd");
    fileNames.push_back("point_cloud002.pcd");
    //insertBeeModels(fileNames);
    //insertBunnyModels(fileNames);
    
    // import models
    for (auto s : fileNames) {
        if (boost::algorithm::ends_with(s, ".obj")) {
            importPointsFromOBJ(models, filePath + s);
        }
        else if (boost::algorithm::ends_with(s, ".obj")) {
            importPointsFromPCD(models, filePath + s);
        }
        else {
            std::cerr << "Cannot open file, please check the file name and path." << std::endl;
            exit(-1);
        }
    }
    
    // registration
    registration(models);

    // reconstruction
    reconstruction();

    return 0;
}