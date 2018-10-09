#include "label_manager/manager.h"

#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>
#include "mesh_msgs/MeshFaceCluster.h"

using namespace boost::filesystem;

namespace label_manager
{
    LabelManager::LabelManager(ros::NodeHandle& nodeHandle) :
        nh(nodeHandle) {

        if (!nh.getParam("folder_path", folderPath))
        {
            folderPath = "/tmp/label_manager/";
        }

        path p(folderPath);
        if (!is_directory(p) && !exists(p))
        {
            create_directory(p);
        }

        clusterLabelSub = nh.subscribe("cluster_label", 10, &LabelManager::clusterLabelCallback, this);
        newClusterLabelPub = nh.advertise<mesh_msgs::MeshFaceCluster>("new_cluster_label", 1);
        srv_get_labeled_clusters = nh.advertiseService(
            "get_labeled_clusters",
            &LabelManager::service_getLabeledClusters,
            this
        );
        srv_get_label_groups = nh.advertiseService(
            "get_label_groups",
            &LabelManager::service_getLabelGroups,
            this
        );
        srv_get_labeled_cluster_group = nh.advertiseService(
            "get_labeled_cluster_group",
            &LabelManager::service_getLabeledClusterGroup,
            this
        );
        srv_delete_label = nh.advertiseService(
            "delete_label",
            &LabelManager::service_deleteLabel,
            this
        );

        ROS_INFO("Started LabelManager");

        ros::spin();
    }

    void LabelManager::clusterLabelCallback(const mesh_msgs::MeshFaceClusterStamped::ConstPtr& msg)
    {
        ROS_INFO_STREAM("Got msg for mesh: " << msg->uuid << " with label: " << msg->cluster.label);

        std::vector<uint> indices;
        std::string fileName = getFileName(msg->uuid, msg->cluster.label);

        // if appending (not override), first figure what new indices we have to add
        if (!msg->override)
        {
            std::vector<uint> readIndices = readIndicesFromFile(fileName);

            // if read indices is empty no file was found or could not be read
            if (readIndices.empty())
            {
                indices = msg->cluster.face_indices;
            }
            else
            {
                for (size_t i = 0; i < msg->cluster.face_indices.size(); i++)
                {
                    uint idx = msg->cluster.face_indices[i];

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
            indices = msg->cluster.face_indices;
        }

        // publish every new labeled cluster
        newClusterLabelPub.publish(msg->cluster);

        // make sure mesh folder exists before writing
        path p(folderPath + "/" + msg->uuid);
        if (!is_directory(p) || !exists(p))
        {
            create_directory(p);
        }

        writeIndicesToFile(fileName, indices, !msg->override);
    }

    bool LabelManager::service_getLabeledClusters(
        mesh_msgs::GetLabeledClusters::Request& req,
        mesh_msgs::GetLabeledClusters::Response& res
    )
    {
        ROS_DEBUG_STREAM("Service call with uuid: " << req.uuid);

        path p (folderPath + "/" + req.uuid);
        directory_iterator end_itr;

        if (!is_directory(p) || !exists(p))
        {
            ROS_DEBUG_STREAM("No labeled clusters for uuid '" << req.uuid << "' found");

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

                mesh_msgs::MeshFaceCluster c;
                c.face_indices = readIndicesFromFile(itr->path().string());
                c.label = label;

                res.clusters.push_back(c);
            }
        }

        return true;
    }

    bool LabelManager::service_getLabelGroups(
        label_manager::GetLabelGroups::Request& req,
        label_manager::GetLabelGroups::Response& res)
    {
        path p (folderPath + "/" + req.uuid);
        directory_iterator end_itr;

        if (!is_directory(p) || !exists(p))
        {
            ROS_WARN_STREAM("No labeled clusters for uuid '" << req.uuid << "' found");

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
                if (std::find(res.labels.begin(), res.labels.end(), label) == res.labels.end())
                {
                    res.labels.push_back(label);
                }
            }
        }


        return true;
    }

    bool LabelManager::service_deleteLabel(
        label_manager::DeleteLabel::Request& req,
        label_manager::DeleteLabel::Response& res)
    {
        path p(getFileName(req.uuid, req.label));

        if (!is_regular_file(p) || !exists(p))
        {
            ROS_WARN_STREAM("Could not delete label '" << req.label << "' of mesh '" << req.uuid << "'.");

            return false;
        }

        res.cluster.face_indices = readIndicesFromFile(p.filename().string());
        res.cluster.label = req.label;

        return remove(p);
    }

    bool LabelManager::service_getLabeledClusterGroup(
        label_manager::GetLabeledClusterGroup::Request& req,
        label_manager::GetLabeledClusterGroup::Response& res)
    {
        path p (folderPath + "/" + req.uuid);
        directory_iterator end_itr;

        if (!is_directory(p) || !exists(p))
        {
            ROS_WARN_STREAM("No labeled clusters for uuid '" << req.uuid << "' found");

            return false;
        }

        for (directory_iterator itr(p); itr != end_itr; ++itr)
        {
            // if file is no dir
            if (is_regular_file(itr->path()) && itr->path().filename().string().find(req.labelGroup) == 0)
            {
                std::string label = itr->path().filename().string();
                // remove extension from label
                boost::replace_all(label, itr->path().filename().extension().string(), "");

                mesh_msgs::MeshFaceCluster c;
                c.face_indices = readIndicesFromFile(itr->path().string());
                c.label = label;

                res.clusters.push_back(c);
            }
        }


        return true;
    }

    bool LabelManager::writeIndicesToFile(
        const std::string& fileName,
        const std::vector<uint>& indices,
        const bool append
    )
    {
        if (indices.empty())
        {
            ROS_WARN_STREAM("Empty indices.");

            return true;
        }

        std::ios_base::openmode mode = append ? (std::ios::out|std::ios::app) : std::ios::out;
        std::ofstream ofs(fileName.c_str(), mode);

        ROS_DEBUG_STREAM("Writing indices to file: " << fileName);

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
            ROS_DEBUG_STREAM("Successfully written indices to file.");

            return true;
        }
        else
        {
            ROS_ERROR_STREAM("Could not open file: " << fileName);
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
            ROS_DEBUG_STREAM("File " << fileName << " does not exists. Nothing to read...");

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
