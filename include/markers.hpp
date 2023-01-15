//
// Created by aloch on 03.01.23.
//

#ifndef FRAMEWORK_MARKERS_MARKERS_HPP
#define FRAMEWORK_MARKERS_MARKERS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <metavision/sdk/base/events/event_cd.h>
#include "camera.hpp"
#include "utils/options.hpp"

class Model3D {
public:
    Model3D();

    void setPose(const Eigen::Vector3f &trans, const Eigen::Quaternionf &rot);

    // Getters
    Eigen::Ref<Eigen::Vector3f> getVertexWorldCs(unsigned int ivert) { return vert_world_cs_[ivert]; }

    Eigen::Vector3f getVertexObjectCs(unsigned int ivert) { return vert_object_cs_[ivert]; }

    Eigen::Ref<Eigen::Vector2f> getVertexFrameCs(unsigned int ivert) { return vert_frame_cs_[ivert]; }

    //Utils
    float calculateDistanceFromCenter(const Metavision::Event2d *ev) const { return (Eigen::Vector2f(ev->x, ev->y) - projected_center_).norm(); }

    bool checkIfWithinRange(const Metavision::Event2d *ev) const { return calculateDistanceFromCenter(ev) < max_projected_distance_; }

private:
    void clearModel();
    void transform();
    Eigen::Vector2f project(const Eigen::Vector3f &point);
    Eigen::Vector2f project(unsigned int ivert);

    Eigen::Vector2f projected_center_;

    std::vector<Eigen::Vector3f> vert_object_cs_; // Coordinates of the vertices on the Object Coordinate System
    std::vector<Eigen::Vector3f> vert_world_cs_;  // Coordinates of the vertices on the World Coordinate System
    std::vector<Eigen::Vector2f> vert_frame_cs_;  // Projection of the vertices

    Eigen::Vector3f translation_;
    Eigen::Quaternionf rot_quaternion_;
    Eigen::Matrix3f camera_matrix_eigen_;

    float max_projected_distance_;

};

class Marker{
public:
    Marker(int id,
           std::vector<cv::Point3f> points_3d,
           std::vector<std::vector<cv::Point>> contours,
           Camera *cam);

    cv::Mat t_vec;
    cv::Mat r_vec;
//    cv::Point2f project(cv::Point3f &point);
    std::vector<cv::Point2f> projected_objectpoints;
    std::vector<cv::Point3f> objectpoints_3d;

    cv::Point2f projected_centerpoint;
    cv::Point3f centerpoint_3d;
    int id;

private:
    Camera *cam_obj;
    std::vector<cv::Point> position_2d;
};

class MarkersManager{
public:
    MarkersManager(Utils::Options::MarkersSetup setup, Camera *cam);

    std::vector<Marker> matchMarkers(std::vector<float> frequencies, std::vector<std::vector<cv::Point>> filtered_contours);

private:
    Camera *cam_obj;
    Utils::Options::MarkersSetup config;


};


#endif //FRAMEWORK_MARKERS_MARKERS_HPP
