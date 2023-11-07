#include "label_manager/manager.h"

#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>
#include "mesh_msgs/msg/mesh_face_cluster.h"

using namespace boost::filesystem;

using std::placeholders::_1;
using std::placeholders::_2;

namespace label_manager
{
    LabelManager::LabelManager(std::string handle_str) 
    :rclcpp::Node(handle_str) 
    {
        // TODO: check if this is correct
        this->declare_parameter("folder_path", "/tmp/label_manager/");
        folderPath = this->get_parameter("folder_path").as_string();
        

        path p(folderPath);
        if(!is_directory(p) && !exists(p))
        {
            create_directory(p);
        }

        clusterLabelSub = this->create_subscription<mesh_msgs::msg::MeshFaceClusterStamped>(
            "cluster_label", 10, std::bind(&LabelManager::clusterLabelCallback, this, _1));

        newClusterLabelPub = this->create_publisher<mesh_msgs::msg::MeshFaceCluster>(
            "new_cluster_label", 10);

        srv_get_labeled_clusters = this->create_service<mesh_msgs::srv::GetLabeledClusters>(
            "get_labeled_clusters", std::bind(&LabelManager::service_getLabeledClusters, this, _1, _2));

        srv_get_label_groups = this->create_service<label_manager::srv::GetLabelGroups>(
            "get_label_groups", std::bind(&LabelManager::service_getLabelGroups, this, _1, _2));

        srv_get_labeled_cluster_group = this->create_service<label_manager::srv::GetLabeledClusterGroup>(
            "get_labeled_cluster_group", std::bind(&LabelManager::service_getLabeledClusterGroup, this, _1, _2));

        srv_delete_label = this->create_service<label_manager::srv::DeleteLabel>(
            "delete_label", std::bind(&LabelManager::service_deleteLabel, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Started LabelManager");
    }

    void LabelManager::clusterLabelCallback(const mesh_msgs::msg::MeshFaceClusterStamped& msg)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Got msg for mesh: " << msg.uuid << " with label: " << msg.cluster.label);

        std::vector<uint> indices;
        std::string fileName = getFileName(msg.uuid, msg.cluster.label);

        // if appending (not override), first figure what new indices we have to add
        if (!msg.override)
        {
            std::vector<uint> readIndices = readIndicesFromFile(fileName);

            // if read indices is empty no file was found or could not be read
            if (readIndices.empty())
            {
                indices = msg.cluster.face_indices;
            }
            else
            {
                for (size_t i = 0; i < msg.cluster.face_indices.size(); i++)
                {
                    uint idx = msg.cluster.face_indices[i];

                    // if msg index is not already in file, add it to indices vector
                    if (std::find(readIndices.begin(), readIndices.end(), idx) == readIndices.end())
                    {
                        indices.push_back(idx);
                    }
                }
            }
        }
        else
        {
            indices = msg.cluster.face_indices;
        }

        // publish every new labeled cluster
        newClusterLabelPub->publish(msg.cluster);

        // make sure mesh folder exists before writing
        path p(folderPath + "/" + msg.uuid);
        if (!is_directory(p) || !exists(p))
        {
            create_directory(p);
        }

        writeIndicesToFile(fileName, indices, !msg.override);
    }

    bool LabelManager::service_getLabeledClusters(
        const std::shared_ptr<mesh_msgs::srv::GetLabeledClusters::Request> req,
        std::shared_ptr<mesh_msgs::srv::GetLabeledClusters::Response>      res)
    {

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Service call with uuid: " << req->uuid);

        path p (folderPath + "/" + req->uuid);
        directory_iterator end_itr;

        if (!is_directory(p) || !exists(p))
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "No labeled clusters for uuid '" << req->uuid << "' found");

            return false;
        }

        for (directory_iterator itr(p); itr != end_itr; ++itr)
        {
            // if file is no dir
            if (is_regular_file(itr->path()))
            {
                std::string label = itr->path().filename().string();
                // remove extension from label
                boost::replace_all(label, itr->path().filename().extension().string(), "");

                mesh_msgs::msg::MeshFaceCluster c;
                c.face_indices = readIndicesFromFile(itr->path().string());
                c.label = label;

                res->clusters.push_back(c);
            }
        }

        return true;
    }

    bool LabelManager::service_getLabelGroups(
        const std::shared_ptr<label_manager::srv::GetLabelGroups::Request> req,
        std::shared_ptr<label_manager::srv::GetLabelGroups::Response>      res)
    {
        path p (folderPath + "/" + req->uuid);
        directory_iterator end_itr;

        if (!is_directory(p) || !exists(p))
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "No labeled clusters for uuid '" << req->uuid << "' found");

            return false;
        }

        for (directory_iterator itr(p); itr != end_itr; ++itr)
        {
            // if file is no dir
            if (is_regular_file(itr->path()))
            {
                std::string label = itr->path().filename().string();
                // remove extension from label
                boost::replace_all(label, itr->path().filename().extension().string(), "");

                // assuming the labels will look like this: 'GROUP_SOMETHINGELSE',
                // remove everthing not representing the group
                // TODO make seperator configurable
                label = label.substr(0, label.find_first_of("_", 0));

                // only add label group to response if not already added
                if (std::find(res->labels.begin(), res->labels.end(), label) == res->labels.end())
                {
                    res->labels.push_back(label);
                }
            }
        }


        return true;
    }

    bool LabelManager::service_getLabeledClusterGroup(
        const std::shared_ptr<label_manager::srv::GetLabeledClusterGroup::Request> req,
        std::shared_ptr<label_manager::srv::GetLabeledClusterGroup::Response>      res)
    {
        path p (folderPath + "/" + req->uuid);
        directory_iterator end_itr;

        if (!is_directory(p) || !exists(p))
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "No labeled clusters for uuid '" << req->uuid << "' found");

            return false;
        }

        for (directory_iterator itr(p); itr != end_itr; ++itr)
        {
            // if file is no dir
            if (is_regular_file(itr->path()) && itr->path().filename().string().find(req->label_group) == 0)
            {
                std::string label = itr->path().filename().string();
                // remove extension from label
                boost::replace_all(label, itr->path().filename().extension().string(), "");

                mesh_msgs::msg::MeshFaceCluster c;
                c.face_indices = readIndicesFromFile(itr->path().string());
                c.label = label;

                res->clusters.push_back(c);
            }
        }


        return true;
    }

    bool LabelManager::service_deleteLabel(
        const std::shared_ptr<label_manager::srv::DeleteLabel::Request> req,
        std::shared_ptr<label_manager::srv::DeleteLabel::Response>      res)
    {
        path p(getFileName(req->uuid, req->label));

        if (!is_regular_file(p) || !exists(p))
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Could not delete label '" << req->label << "' of mesh '" << req->uuid << "'.");

            return false;
        }

        res->cluster.face_indices = readIndicesFromFile(p.filename().string());
        res->cluster.label = req->label;

        return remove(p);
    }

    bool LabelManager::writeIndicesToFile(
        const std::string& fileName,
        const std::vector<uint>& indices,
        const bool append
    )
    {
        if (indices.empty())
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Empty indices.");

            return true;
        }

        std::ios_base::openmode mode = append ? (std::ios::out|std::ios::app) : std::ios::out;
        std::ofstream ofs(fileName.c_str(), mode);

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Writing indices to file: " << fileName);

        if (ofs.is_open())
        {
            // if in append mode add , after the old data
            if (append)
            {
                ofs << ",";
            }

            size_t size = indices.size();
            for (size_t i = 0; i < size; i++)
            {
                ofs << indices[i];

                if (i < size - 1)
                {
                    ofs << ",";
                }
            }

            ofs.close();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Successfully written indices to file.");

            return true;
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Could not open file: " << fileName);
        }

        return false;
    }

    std::vector<uint> LabelManager::readIndicesFromFile(const std::string& fileName)
    {
        std::ifstream ifs(fileName.c_str(), std::ios::in);
        std::vector<uint> faceIndices;

        // if file dos not exists, return empty vector
        if (!ifs.good())
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "File " << fileName << " does not exists. Nothing to read...");

            return faceIndices;
        }

        std::string stringNumber;
        while (std::getline(ifs, stringNumber, ','))
        {
            faceIndices.push_back(atoi(stringNumber.c_str()));
        }

        return faceIndices;
    }

    std::string LabelManager::getFileName(const std::string& uuid, const std::string& label)
    {
        return folderPath + "/" +uuid + "/" + label + ".dat";
    }
}
