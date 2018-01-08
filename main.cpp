///////////////////////////////////////////////////////////////////////////
// main.cpp - the program builds could points from group images
//            the program is founded on examples from opencv/sfm module
//

#define CERES_FOUND 1           // for work cv::smf::reconstract(.....)

#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <fstream>
#include <string>

// shows information about this program and arguments command line
static void help();

// takes the name of the directory and fills the vector with the file names in this directory
int getdir(const std::string _filename, std::vector<cv::String> &files);

/*MAIN FUNCTION*/
int main(int argc, char* argv[])
{
    // Read input parameters
    if ( argc != 1 )
    {
        help();
        exit(0);
    }
    // Parse the image paths
    std::vector<cv::String> imagesPath;
    std::string pahtFile = "../../data_for_projects/pointCloud_images_Rot/dataset_files.txt";
    getdir( pahtFile, imagesPath );
    // Build instrinsics
    float f  = 350,
            cx = 240, cy = 360;
    cv::Matx33d K = cv::Matx33d( f, 0, cx,
                                 0, f, cy,
                                 0, 0,  1);
    bool is_projective = true;
    std::vector<cv::Mat> Rs_est, ts_est, points3d_estimated;
    cv::sfm::reconstruct(imagesPath, Rs_est, ts_est, K, points3d_estimated, is_projective);

    // Print output
    std::cout << "\n----------------------------\n" << std::endl;
    std::cout << "Reconstruction: " << std::endl;
    std::cout << "============================" << std::endl;
    std::cout << "Estimated 3D points: " << points3d_estimated.size() << std::endl;
    std::cout << "Estimated cameras: " << Rs_est.size() << std::endl;
    std::cout << "Refined intrinsics: " << std::endl << K << std::endl << std::endl;
    std::cout << "3D Visualization: " << std::endl;
    std::cout << "============================" << std::endl;

    cv::viz::Viz3d window("Coordinate Frame");
    window.setWindowSize(cv::Size(500,500));
    window.setWindowPosition(cv::Point(150,150));
    window.setBackgroundColor(); // black by default
    // Create the pointcloud
    std::cout << "Recovering points  ... ";
    // recover estimated points3d
    std::vector<cv::Vec3f> point_cloud_est;
    for (int i = 0; i < points3d_estimated.size(); ++i)
        point_cloud_est.push_back(cv::Vec3f(points3d_estimated[i]));
    std::cout << "[DONE]" << std::endl;
    std::cout << "Recovering cameras ... ";
    std::vector<cv::Affine3d> path;
    for (size_t i = 0; i < Rs_est.size(); ++i)
        path.push_back(cv::Affine3d(Rs_est[i],ts_est[i]));
    std::cout << "[DONE]" << std::endl;
    if ( point_cloud_est.size() > 0 )
    {
        std::cout << "Rendering points   ... ";
        cv::viz::WCloud cloud_widget(point_cloud_est, cv::viz::Color::green());
        window.showWidget("point_cloud", cloud_widget);
        std::cout << "[DONE]" << std::endl;
    }
    else
    {
        std::cout << "Cannot render points: Empty pointcloud" << std::endl;
    }
    if ( path.size() > 0 )
    {
        std::cout << "Rendering Cameras  ... ";
        window.showWidget("cameras_frames_and_lines", cv::viz::WTrajectory(path, cv::viz::WTrajectory::BOTH, 0.1, cv::viz::Color::green()));
        window.showWidget("cameras_frustums", cv::viz::WTrajectoryFrustums(path, K, 0.1,cv:: viz::Color::yellow()));
        window.setViewerPose(path[0]);
        std::cout << "[DONE]" << std::endl;
    }
    else
    {
        std::cout << "Cannot render the cameras: Empty path" << std::endl;
    }
    std::cout << std::endl << "Press 'q' to close each windows ... " << std::endl;
    window.spin();
    return 0;
}
/*END MAIN FUNCTION*/

// shows information about this program and arguments command line
static void help() {
    std::cout
            << "\n------------------------------------------------------------------------------------\n"
            << " This program shows the multiview reconstruction capabilities in the \n"
            << " OpenCV Structure From Motion (SFM) module.\n"
            << " It reconstruct a scene from a set of 2D images \n"
            << " Usage:\n"
            << "        example_sfm_scene_reconstruction <path_to_file> <f> <cx> <cy>\n"
            << " where: path_to_file is the file absolute path into your system which contains\n"
            << "        the list of images to use for reconstruction. \n"
            << "        f  is the focal lenght in pixels. \n"
            << "        cx is the image principal point x coordinates in pixels. \n"
            << "        cy is the image principal point y coordinates in pixels. \n"
            << "------------------------------------------------------------------------------------\n\n"
            << std::endl;
}

// takes the name of the directory and fills the vector with the file names in this directory
int getdir(const std::string _filename, std::vector<cv::String> &files) {
    std::ifstream myfile(_filename.c_str());
    if (!myfile.is_open()) {
        std::cout << "Unable to read file: " << _filename << std::endl;
        exit(0);
    } else {;
        size_t found = _filename.find_last_of("/\\");
        std::string line_str, path_to_file = _filename.substr(0, found);
        while ( getline(myfile, line_str) )
            files.push_back(path_to_file+std::string("/")+line_str);
    }
    return 1;
}

