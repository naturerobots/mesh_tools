#ifndef LABEL_MANAGER_H_
#define LABEL_MANAGER_H_

#include <vector>

#include <ros/ros.h>
#include <mesh_msgs/MeshFaceClusterStamped.h>
#include <mesh_msgs/GetLabeledClusters.h>
#include <label_manager/GetLabelGroups.h>
#include <label_manager/GetLabeledClusterGroup.h>
#include <label_manager/DeleteLabel.h>

namespace label_manager
{

class LabelManager
{
public:
    LabelManager(ros::NodeHandle& nodeHandle);

private:
    ros::NodeHandle nh;
    ros::Subscriber clusterLabelSub;
    ros::Publisher newClusterLabelPub;
    ros::ServiceServer srv_get_labeled_clusters;
    ros::ServiceServer srv_get_label_groups;
    ros::ServiceServer srv_get_labeled_cluster_group;
    ros::ServiceServer srv_delete_label;

    std::string folderPath;

    void clusterLabelCallback(const mesh_msgs::MeshFaceClusterStamped::ConstPtr& msg);
    bool service_getLabeledClusters(
        mesh_msgs::GetLabeledClusters::Request& req,
        mesh_msgs::GetLabeledClusters::Response& res);
    bool service_getLabelGroups(
        label_manager::GetLabelGroups::Request& req,
        label_manager::GetLabelGroups::Response& res);
    bool service_getLabeledClusterGroup(
        label_manager::GetLabeledClusterGroup::Request& req,
        label_manager::GetLabeledClusterGroup::Response& res);
    bool service_deleteLabel(
        label_manager::DeleteLabel::Request& req,
        label_manager::DeleteLabel::Response& res);

    bool writeIndicesToFile(const std::string& fileName, const std::vector<uint>& indices, const bool append);
    std::vector<uint> readIndicesFromFile(const std::string& fileName);
    std::string getFileName(const std::string& uuid, const std::string& label);
};

}

#endif
