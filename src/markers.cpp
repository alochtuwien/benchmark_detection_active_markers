//
// Created by aloch on 03.01.23.
//

#include "../include/markers.hpp"

Model3D::Model3D() {
    clearModel();
}

void Model3D::clearModel() {
        vert_object_cs_.clear();
        vert_world_cs_.clear();
        vert_frame_cs_.clear();
}

void Model3D::setPose(const Eigen::Vector3f &trans, const Eigen::Quaternionf &rot) {
    translation_ = trans;
    rot_quaternion_ = rot;
    transform();
}

Eigen::Vector2f Model3D::project(const Eigen::Vector3f &point)
{
    Eigen::Vector3f tmp = camera_matrix_eigen_ * point;
    if (abs(tmp[2]) > 0)
    {
        tmp /= tmp[2];
    }
    return Eigen::Vector2f{tmp[0], tmp[1]};
}

Eigen::Vector2f Model3D::project(unsigned int ivert)
{
    Eigen::Vector3f tmp = camera_matrix_eigen_ * vert_world_cs_[ivert];
    if (abs(tmp[2]) > 0)
    {
        tmp /= tmp[2];
    }
    return Eigen::Vector2f{tmp[0], tmp[1]};
}

void Model3D::transform()
{
    projected_center_ = project(translation_);

    max_projected_distance_ = 0;
    for (unsigned int ii = 0; ii < vert_object_cs_.size(); ii++)
    {
        vert_world_cs_[ii] = rot_quaternion_ * vert_object_cs_[ii] + translation_;
        vert_frame_cs_[ii] = project(ii);
        if (ii < 4)
        {
            if ((vert_frame_cs_[ii] - projected_center_).norm() > max_projected_distance_)
                max_projected_distance_ = (vert_frame_cs_[ii] - projected_center_).norm();
        }
    }
    max_projected_distance_ *= 1.2;
}


MarkersManager::MarkersManager(Utils::Options::MarkersSetup setup, Camera *cam) {
    config = setup;
    cam_obj = cam;
}

std::vector<Marker> MarkersManager::matchMarkers(std::vector<float> frequencies, std::vector<std::vector<cv::Point>> filtered_contours) {
    std::vector<std::pair<std::vector<int>, int>> output;
    std::vector<int> matches(frequencies.size(), 0);
    std::vector<Marker> detected_markers;

    for (unsigned int i = 0; i < config.ids.size(); ++i){
        std::cout << "Looking for id " << config.ids[i] << std::endl;
        std::vector<int> assigned(config.frequencies[i].size(), -1);
        std::vector<std::vector<cv::Point>> assigned_contours(config.frequencies[i].size());
        bool not_found = false;

        for (unsigned int j = 0; j < config.frequencies[i].size(); ++j){
            auto it = std::find(frequencies.begin(), frequencies.end(), config.frequencies[i][j]);
            if (it != frequencies.end())
            {
                int index = it - frequencies.begin();
                assigned[j] = index;
            }
            else{
                not_found = true;
                break;
            }
        }

        if (!not_found){
            output.push_back(std::make_pair(assigned, i));
            for (unsigned int j = 0; j<assigned.size(); ++j){
                assigned_contours[j] = filtered_contours[assigned[j]];
            }
            auto new_marker = Marker(config.ids[i], config.coordinates[i], assigned_contours, cam_obj);
            detected_markers.push_back(new_marker);
        }
    }
    return detected_markers;
};

Marker::Marker(int idx,
               std::vector<cv::Point3f> points_3d,
               std::vector<std::vector<cv::Point>> contours,
               Camera *cam) {


    id = idx;
    std::vector<cv::Point2f> centers;
    for (int i=0; i<contours.size(); i++){
        cv::Moments M = cv::moments(contours[i]);
        cv::Point2f center(M.m10/M.m00, M.m01/M.m00);
        centers.push_back(center);
    }

    auto solve_method = cv::SolvePnPMethod::SOLVEPNP_AP3P;

    cv::solvePnP(points_3d, centers, cam->camera_matrix_cv, cam->dist_coeffs, r_vec, t_vec, false);

    std::vector<cv::Point2f> projected;
    cv::projectPoints(points_3d, r_vec, t_vec, cam->camera_matrix_cv, cam->dist_coeffs, projected);


    std::vector<cv::Point2f> center_projected;
    std::vector<cv::Point3f> center_3d;
    center_3d.push_back(cv::Point3f(0.0,0.0,0.0));

    cv::projectPoints(center_3d, r_vec, t_vec, cam->camera_matrix_cv, cam->dist_coeffs, center_projected);

    centerpoint_3d = center_3d[0];
    projected_centerpoint = center_projected[0];

    objectpoints_3d = points_3d;
    projected_objectpoints = projected;
}
