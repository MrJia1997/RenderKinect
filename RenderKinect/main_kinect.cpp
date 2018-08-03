/*********************************************************************
 *
 *  Copyright (c) 2014, Jeannette Bohg - MPI for Intelligent System
 *  (jbohg@tuebingen.mpg.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Jeannette Bohg nor the names of MPI
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* main_kinect.cpp 
 * Test program that sets up simulator with specific camera parameters 
 * and object mesh. A number of object poses is sampled from which 
 * a desired measured output (depthmap, label image, point cloud) is 
 * generated and stored.
 */

#include "render_kinect/simulate.h"
#include "render_kinect/camera.h"

#include "set_cover.h"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

/* Sampling of random 6DoF transformations. */
void getRandomTransform(const double &p_x,
			const double &p_y,
			const double &p_z,
			const double &p_angle,
			Eigen::Affine3d &p_tf) {

    Eigen::Vector3d axis(((double)(rand()%1000))/1000.0,
                ((double)(rand()%1000))/1000.0,
                ((double)(rand()%1000))/1000.0);
    Eigen::Vector3d t(p_x*(double)(rand()%2000 -1000)/1000,
                p_y*(double)(rand()%2000 -1000)/1000,
                p_z*(double)(rand()%2000 -1000)/1000);
    p_tf = Eigen::Affine3d::Identity();
    p_tf.translate(t);
    p_tf.rotate(Eigen::AngleAxisd( p_angle*(double)(rand()%2000 - 1000)/1000, axis));
}

std::vector<Eigen::Affine3d> getView(const Eigen::Vector3d& camera_normal, int horizontal_count, int vertical_count)
{
    // horizontal range = 0 .. 2 * PI
    // vertical range   = 0 ..     PI (supposed to be -1/2 * PI .. 1/2 * PI as longitude and latitude)
    
    std::vector<Eigen::Affine3d> views;

    double horizontal_angle = 2 * M_PI / horizontal_count;
    double vertical_angle = M_PI / (vertical_count - 1);
    
    for (int i = 0; i < vertical_count; ++i)
    {
        for (int j = 0; j < horizontal_count; ++j)
        //for (int j = 0; j < 1; ++j)
        {
            if ((i == 0 || i == vertical_count - 1) && j)
            {
                // 1 view is enough for the north & south pole
                continue;
            }

            Eigen::Affine3d transform(Eigen::Affine3d::Identity());
            double horizontal_spin = horizontal_angle * j;
            double vertical_spin = vertical_angle * i;

            Eigen::Matrix3d rotation;
            Eigen::Matrix3d aaa, bbb;
            aaa = Eigen::AngleAxisd(vertical_spin, Eigen::Vector3d::UnitY());
            bbb = Eigen::AngleAxisd(horizontal_spin, camera_normal);
            // std::cout << "hey aaa" << std::endl << aaa << std::endl;
            // std::cout << "hey bbb" << std::endl << bbb << std::endl;
            rotation = Eigen::AngleAxisd(vertical_spin, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(horizontal_spin, camera_normal);
            // std::cout << rotation << std::endl << "=====" << std::endl;
            transform.translate(Eigen::Vector3d(0, 0, 1.5));
            // std::cout << transform.matrix() << std::endl << "=====" << std::endl;
            transform.rotate(rotation);
            // std::cout << transform.matrix() << std::endl << "=====" << std::endl;
            // std::cout << "vertical_spin = " << vertical_spin << std::endl;
            // transform.rotate(Eigen::AngleAxisd(horizontal_spin, camera_normal));
            // transform.rotate(Eigen::AngleAxisd(vertical_spin, Eigen::Vector3d::UnitY()));

            views.push_back(transform);
        }
    }

    return views;
}

// main function that generated a number of sample outputs for a given object mesh. 
int main(int argc, char **argv) {

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " model_file.obj" << std::endl;
        exit(-1);
    }

    // Get the path to the object mesh model.
    std::string object_models_dir = "../obj_models/";
    std::stringstream full_path;
    full_path << object_models_dir << argv[1];

    // Get the path to the dot pattern
    std::string dot_path = "../data/kinect-pattern_3x3.png";

    // Camera Parameters
    render_kinect::CameraInfo cam_info;

    cam_info.width = 640;
    cam_info.height = 480;
    cam_info.cx_ = 320;
    cam_info.cy_ = 240;

    cam_info.z_near = 0.5;
    cam_info.z_far = 6.0;
    cam_info.fx_ = 300.0;
    cam_info.fy_ = 300.0;
    // baseline between IR projector and IR camera
    cam_info.tx_ = 0.075;

    // Type of noise
    //  cam_info.noise_ = render_kinect::GAUSSIAN;
    //  cam_info.noise_ = render_kinect::PERLIN;
    cam_info.noise_ = render_kinect::NONE;

    // Test Transform
    /*double phi = (1.0 + sqrt(5.0)) / 2.0;
    double phi_inv = 1.0 / phi;

    Eigen::Vector3d normals[20];
    int cnt = 0;
    for (int k = 0; k < 5; ++k)
    {
        for (int i = -1; i <= 1; i += 2)
        {
            for (int j = -1; j <= 1; j += 2)
            {
                if (k == 0)
                {
                    normals[cnt] << 0, i * phi, j * phi_inv;
                }
                else if (k == 1)
                {
                    normals[cnt] << i * phi_inv, 0, j * phi;
                }
                else if (k == 2)
                {
                    normals[cnt] << i * phi, j * phi_inv, 0;
                }
                else if (k == 3)
                {
                    normals[cnt] << -1, i, j;
                }
                else if (k == 4)
                {
                    normals[cnt] << 1, i, j;
                }
                ++cnt;
            }
        }
    }*/

    Eigen::Vector3d camera_normal(0, 0, -1);
    // std::vector<Eigen::Affine3d> views = getView(camera_normal, 8, 7);
    std::vector<Eigen::Affine3d> views = getView(camera_normal, 18, 13);

    /*Eigen::Vector3d normals[6];
    normals[0] << -1, 0, 0;
    normals[1] << +1, 0, 0;
    normals[2] << 0, -1, 0;
    normals[3] << 0, +1, 0;
    normals[4] << 0, 0, -1;
    normals[5] << 0, 0, +1;*/

    /*
    Eigen::Affine3d transform(Eigen::Affine3d::Identity());
    Eigen::Vector3d axis;
    axis = normals[1].cross(normals[0]).normalized();
    double costheta = normals[1].dot(normals[0]) / (normals[1].norm() * normals[0].norm());
    transform.rotate(Eigen::AngleAxisd(costheta, axis));
    transform.translate(Eigen::Vector3d(0, 0, 1.5));
    */

    // Number of samples
    // int frames = 20;
    int frames = views.size();
    // Flags for what output data should be generated
    bool store_depth = 1;
    bool store_label = 1;
    bool store_pcd = 1;
    // std::cout << "good1\n";



    // Storage of random transform
    /*Eigen::Affine3d noise;
    Eigen::Affine3d transform(Eigen::Affine3d::Identity());
    transform.translate(Eigen::Vector3d(0, 0, 1.5));
    transform.rotate(Eigen::Quaterniond(0.906614, -0.282680, -0.074009, -0.304411));*/


    //// Kinect Simulator
    //render_kinect::Simulate Simulator(cam_info, full_path.str(), dot_path);
    //// calculate keypoints
    ////Simulator.keypointMeasurement();
    //Simulator.subsampling();
    //std::vector<std::vector<int>> visibleResult;
    //for (int i = 0; i < frames; i++) {
    //    std::cout << "i = " << i << std::endl;
    //    Eigen::Affine3d current_tf = views[i];
    //    std::vector<int> visibleKeypointIndices;
    //    Simulator.calckeypointVisible(current_tf, visibleKeypointIndices);
    //    visibleResult.push_back(visibleKeypointIndices);
    //}
    //ofstream transLog, poseKeypointResult;
    //transLog.open("trans_log_200.txt", ios::out);
    //poseKeypointResult.open("pose_keypoint_200_" + std::string(argv[1]) + ".txt", ios::out);
    //for (int i = 0; i < frames; i++) {
    //    transLog << "Pose Transform Matrix " << i << ":" << std::endl;
    //    transLog << views[i].matrix() << std::endl;
    //    poseKeypointResult << "Pose Visible Keypoints Indices " << i << ":" << std::endl;
    //    for (int id : visibleResult[i])
    //        poseKeypointResult << id << " ";
    //    poseKeypointResult << std::endl;
    //}
    //transLog.close();
    //poseKeypointResult.close();
    //std::cout << "Calculate keypoints visibility finished." << std::endl;


    // test subsample visible
    /*PointCloud::Ptr sample_points(new PointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sample_points_visible(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("../tmp/sample_points.pcd", *sample_points);
    int total = sample_points->size();
    std::vector<bool> total_flag(total, false);
    for (auto set : visibleResult) {
        for (auto ele : set) {
            if (ele >= 0 && ele < total) {
                total_flag[ele] = true;
            }
            else {
                std::cerr << "Element out of range." << std::endl;
                exit(-1);
            }
        }
    }
    sample_points_visible->resize(total);
    for (int i = 0; i < total; i++) {
        sample_points_visible->points[i].x = sample_points->points[i].x;
        sample_points_visible->points[i].y = sample_points->points[i].y;
        sample_points_visible->points[i].z = sample_points->points[i].z;
        if (total_flag[i]) {
            sample_points_visible->points[i].r = 255;
            sample_points_visible->points[i].g = 0;
            sample_points_visible->points[i].b = 0;
        }
        else {
            sample_points_visible->points[i].r = 0;
            sample_points_visible->points[i].g = 255;
            sample_points_visible->points[i].b = 0;
        }
    }
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(sample_points_visible);
    while (!viewer.wasStopped()) {}*/



    // calculate set cover
    ifstream poseKeypointResult;
    std::vector<std::set<int>> visibleResult;
    poseKeypointResult.open("pose_keypoint_200_" + std::string(argv[1]) + ".txt", ios::in);
    std::string s;
    while (getline(poseKeypointResult, s)) {
        std::set<int> set;
        getline(poseKeypointResult, s);
        std::vector<std::string> indices;
        boost::split(indices, s, boost::is_any_of(" "));
        for (auto id : indices)
            set.insert(atoi(id.c_str()));
        visibleResult.push_back(set);
    }
    poseKeypointResult.close();
    
    PointCloud::Ptr sample_points(new PointCloud);
    pcl::io::loadPCDFile("../tmp/sample_points.pcd", *sample_points);
    std::vector<int> result = set_cover::calcSetCoverGreedy(visibleResult, sample_points->size(), 0.5, 1);
    
    ofstream resLog;
    resLog.open("result_" + std::string(argv[1]) + ".txt", ios::out);
    int resLen = result.size();
    resLog << "Total " << resLen << " poses after set cover greedy algorithm." << std::endl;
    for (int i = 0; i < resLen; i++) {
        resLog << "Pose Number " << result[i] << ":" << std::endl;
        resLog << views[result[i]].matrix() << std::endl;
    }
    resLog.close();
    std::cout << "set cover greedy completed." << std::endl;
    

    // get scans from different poses
    //ofstream transLog;
    //transLog.open("trans_log_200.txt", ios::out);
    //for (int i = 0; i < frames; i++) {
    //    std::cout << "i = " << i << std::endl;
    //    //transLog << "Transform Matrix " << i << ":" << endl;
    //    // getRandomTransform(0.02, 0.02, 0.02, 0.1, noise);
    //    // Eigen::Affine3d current_tf = noise * transform;
    //    Eigen::Affine3d current_tf = views[i];
    //    //transLog << current_tf.matrix() << endl;
    //    Simulator.simulateMeasurement(current_tf, store_depth, store_label, store_pcd);
    //}
    //transLog.close();
    
    
    
    
    

    //Simulator.simulateMeasurement(current_tf, store_depth, store_label, store_pcd);
    
    system("pause");
    return 0;
}
