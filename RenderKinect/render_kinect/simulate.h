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
/* Header file that sets up the simulator and triggers the simulation 
 * of the kinect measurements and stores the results under a given directory.
 */
#ifndef SIMULATE_H
#define SIMULATE_H

#pragma warning(disable: 4996)

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define HAVE_PCL

#ifdef HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/cloud_viewer.h>
#endif 

#include <string.h>

#include "kinectSimulator.h"

static unsigned countf = 0;

namespace render_kinect {

    class Simulate {
    
    public:
        KinectSimulator * object_model_;
        cv::Mat depth_im_, scaled_im_, point_cloud_, labels_;
        std::string out_path_;
        Eigen::Affine3d transform_;

    
    public:
    
        Simulate(CameraInfo &cam_info, std::string object_name, std::string dot_path) 
            : out_path_("../tmp/") {
            // allocate memory for depth image
            int w = cam_info.width;
            int h = cam_info.height;

            depth_im_ = cv::Mat(h, w, CV_32FC1);
            scaled_im_ = cv::Mat(h, w, CV_32FC1);

            object_model_ = new KinectSimulator(cam_info, object_name, dot_path);

            transform_ = Eigen::Affine3d::Identity();

        }

        ~Simulate() {
            delete object_model_;
        }

        void keypointMeasurement() {
            for (int i = 0; i < 6; i++) {
                PointCloud_I::Ptr keypoints(new PointCloud_I);
                object_model_->calcKeypoints(keypoints, i);
                if (keypoints->size() <= 0)
                    continue;
                std::stringstream ss;
                ss << out_path_ << "keypoint_" << i << ".pcd";
                pcl::io::savePCDFileASCII(ss.str(), *keypoints);
                
                //pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
                //viewer.showCloud(keypoints);
                //while (!viewer.wasStopped()) {}
                
            }
            
        }

        void subsampling() {
            PointCloud::Ptr sample_points(new PointCloud);
            object_model_->samplePoints(sample_points);
            if (sample_points->size() <= 0) {
                std::cerr << "Subsample failed." << std::endl;
                exit(-1);
            }
            std::stringstream ss;
            ss << out_path_ << "sample_points.pcd";
            pcl::io::savePCDFileASCII(ss.str(), *sample_points);
            
            /*pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
            viewer.showCloud(sample_points);
            while (!viewer.wasStopped()) {}*/
        }

        void calckeypointVisible(const Eigen::Affine3d &new_tf, std::vector<int> &visibleKeypointIndices) {
            visibleKeypointIndices.clear();
            
            PointCloud::Ptr p_cloud(new PointCloud);
            std::stringstream ss;
            ss << out_path_ << "sample_points.pcd";
            pcl::io::loadPCDFile(ss.str(), *p_cloud);
            int size = p_cloud->size();

            Eigen::MatrixXd keypoints;
            keypoints.resize(4, size);
            for (int i = 0; i < size; i++) {
                keypoints(0, i) = p_cloud->points[i].x; 
                keypoints(1, i) = p_cloud->points[i].y;
                keypoints(2, i) = p_cloud->points[i].z;
                keypoints(3, i) = 1;
            }

            object_model_->testVisible(new_tf, keypoints, visibleKeypointIndices);

        }

        void simulateMeasurement(const Eigen::Affine3d &new_tf, bool store_depth, bool store_label, bool store_pcd) {
            countf++;
            
            // update old transform
            transform_ = new_tf;

            // simulate measurement of object and store in image, point cloud and labeled image
            cv::Mat p_result;
            object_model_->intersect(transform_, point_cloud_, depth_im_, labels_);
            
            // in case object is not in view, don't store any data
            // However, if background is used, there will be points in the point cloud
            // although they don't belong to the arm
            int n_vis = 4000;
            if (point_cloud_.rows<n_vis) {
                std::cout << "Object not in view.\n";
                return;
            }

            // store on disk
            if (store_depth) {
                std::stringstream lD;
                lD << out_path_ << "depth_orig" << std::setw(3) << std::setfill('0')
                << countf << ".png";
                convertScaleAbs(depth_im_, scaled_im_, 255.0f);
                cv::imwrite(lD.str().c_str(), scaled_im_);
            }

            // store on disk
            if (store_label) {
                std::stringstream lD;
                lD << out_path_ << "labels" << std::setw(3) << std::setfill('0')
                << countf << ".png";
                cv::imwrite(lD.str().c_str(), labels_);
            }

            //convert point cloud to pcl/pcd format
            if (store_pcd) {

                #ifdef HAVE_PCL
                std::stringstream lD;
                lD << out_path_ << "point_cloud" << std::setw(3)
                << std::setfill('0') << countf << ".pcd";

                pcl::PointCloud<pcl::PointXYZ> cloud;
                // Fill in the cloud data
                cloud.width = point_cloud_.rows;
                cloud.height = 1;
                cloud.is_dense = false;
                cloud.points.resize(cloud.width * cloud.height);

                for (int i = 0; i < point_cloud_.rows; i++) {
                    const float* point = point_cloud_.ptr<float>(i);
                    cloud.points[i].x = point[0];
                    cloud.points[i].y = point[1];
                    cloud.points[i].z = point[2];
                }
            
                if (pcl::io::savePCDFileASCII(lD.str(), cloud) != 0)
                    std::cout << "Couldn't store point cloud at " << lD.str() << std::endl;
                else {
                    /*pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
                    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::io::loadPCDFile(lD.str(), *p_cloud);
                    viewer.showCloud(p_cloud);

                    while (!viewer.wasStopped()) {}*/
                }
                #else
                std::cout << "Couldn't store point cloud since PCL is not installed." << std::endl;
                #endif
            }
        }

    };

} //namespace render_kinect
#endif // SIMULATE_H
