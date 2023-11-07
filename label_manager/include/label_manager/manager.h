#ifndef LABEL_MANAGER_H_
#define LABEL_MANAGER_H_

#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "mesh_msgs/msg/mesh_face_cluster_stamped.hpp"
#include "mesh_msgs/srv/get_labeled_clusters.hpp"
#include "label_manager/srv/get_label_groups.hpp"
#include "label_manager/srv/get_labeled_cluster_group.hpp"
#include "label_manager/srv/delete_label.hpp"

#include "rclcpp/rclcpp.hpp"


namespace label_manager
{

class LabelManager : public rclcpp::Node
{
public:
    LabelManager(std::string handle_str = "");

private:
    // Subscriber
    rclcpp::Subscription<mesh_msgs::msg::MeshFaceClusterStamped>::SharedPtr 
        clusterLabelSub;
    
    // Publisher
    rclcpp::Publisher<mesh_msgs::msg::MeshFaceCluster>::SharedPtr 
        newClusterLabelPub;

    // Service (Servers)
    rclcpp::Service<mesh_msgs::srv::GetLabeledClusters>::SharedPtr 
        srv_get_labeled_clusters;
    rclcpp::Service<label_manager::srv::GetLabelGroups>::SharedPtr 
        srv_get_label_groups;
    rclcpp::Service<label_manager::srv::GetLabeledClusterGroup>::SharedPtr 
        srv_get_labeled_cluster_group;
    rclcpp::Service<label_manager::srv::DeleteLabel>::SharedPtr 
        srv_delete_label;
    
    std::string folderPath;

    void clusterLabelCallback(const mesh_msgs::msg::MeshFaceClusterStamped& msg);

    bool service_getLabeledClusters(
        const std::shared_ptr<mesh_msgs::srv::GetLabeledClusters::Request> req,
        std::shared_ptr<mesh_msgs::srv::GetLabeledClusters::Response>      res);
    bool service_getLabelGroups(
        const std::shared_ptr<label_manager::srv::GetLabelGroups::Request> req,
        std::shared_ptr<label_manager::srv::GetLabelGroups::Response>      res);
    bool service_getLabeledClusterGroup(
        const std::shared_ptr<label_manager::srv::GetLabeledClusterGroup::Request> req,
        std::shared_ptr<label_manager::srv::GetLabeledClusterGroup::Response>      res);
    bool service_deleteLabel(
        const std::shared_ptr<label_manager::srv::DeleteLabel::Request> req,
        std::shared_ptr<label_manager::srv::DeleteLabel::Response>      res);

    bool writeIndicesToFile(const std::string& fileName, const std::vector<uint>& indices, const bool append);
    std::vector<uint> readIndicesFromFile(const std::string& fileName);
    std::string getFileName(const std::string& uuid, const std::string& label);
};

}

#endif
