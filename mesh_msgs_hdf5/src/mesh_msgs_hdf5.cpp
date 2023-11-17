#include <mesh_msgs_hdf5/mesh_msgs_hdf5.h>
#include <hdf5_map_io/hdf5_map_io.h>
#include <algorithm>

#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace mesh_msgs_hdf5 
{

hdf5_to_msg::hdf5_to_msg()
:rclcpp::Node("mesh_msgs_hdf5")
{
    // TODO: check if this is correct
    this->declare_parameter("inputFile", "/tmp/map.h5");
    inputFile = this->get_parameter("inputFile").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(), "Using input file: " << inputFile);

    srv_get_uuids_ = this->create_service<mesh_msgs::srv::GetUUIDs>(
        "get_uuids", std::bind(&hdf5_to_msg::service_getUUIDs, this, _1, _2));
    srv_get_geometry_ = this->create_service<mesh_msgs::srv::GetGeometry>(
        "get_geometry", std::bind(&hdf5_to_msg::service_getGeometry, this, _1, _2));
    srv_get_geometry_vertices_ = this->create_service<mesh_msgs::srv::GetGeometry>(
        "get_geometry_vertices", std::bind(&hdf5_to_msg::service_getGeometryVertices, this, _1, _2));
    srv_get_geometry_faces_ = this->create_service<mesh_msgs::srv::GetGeometry>(
        "get_geometry_faces", std::bind(&hdf5_to_msg::service_getGeometryFaces, this, _1, _2));
     srv_get_geometry_vertex_normals_ = this->create_service<mesh_msgs::srv::GetGeometry>(
        "get_geometry_vertexnormals", std::bind(&hdf5_to_msg::service_getGeometryVertexNormals, this, _1, _2));
    srv_get_materials_ = this->create_service<mesh_msgs::srv::GetMaterials>(
        "get_materials", std::bind(&hdf5_to_msg::service_getMaterials, this, _1, _2));
    srv_get_texture_ = this->create_service<mesh_msgs::srv::GetTexture>(
        "get_texture", std::bind(&hdf5_to_msg::service_getTexture, this, _1, _2));
    srv_get_vertex_colors_ = this->create_service<mesh_msgs::srv::GetVertexColors>(
        "get_vertex_colors", std::bind(&hdf5_to_msg::service_getVertexColors, this, _1, _2));
    srv_get_vertex_costs_ = this->create_service<mesh_msgs::srv::GetVertexCosts>(
        "get_vertex_costs", std::bind(&hdf5_to_msg::service_getVertexCosts, this, _1, _2));
    srv_get_vertex_cost_layers_ = this->create_service<mesh_msgs::srv::GetVertexCostLayers>(
        "get_vertex_cost_layers", std::bind(&hdf5_to_msg::service_getVertexCostLayers, this, _1, _2));

    pub_geometry_ = this->create_publisher<mesh_msgs::msg::MeshGeometryStamped>(
        "mesh/geometry", 1);
    pub_vertex_colors_ = this->create_publisher<mesh_msgs::msg::MeshVertexColorsStamped>(
        "mesh/vertex_colors", 1);
    pub_vertex_costs_ = this->create_publisher<mesh_msgs::msg::MeshVertexCostsStamped>(
        "mesh/vertex_costs", 1);


    srv_get_labeled_clusters_ = this->create_service<mesh_msgs::srv::GetLabeledClusters>(
        "get_labeled_clusters", std::bind(&hdf5_to_msg::service_getLabeledClusters, this, _1, _2));

    sub_cluster_label_ = this->create_subscription<mesh_msgs::msg::MeshFaceClusterStamped>(
        "cluster_label", 10, std::bind(&hdf5_to_msg::callback_clusterLabel, this, _1));


    loadAndPublishGeometry();
}

void hdf5_to_msg::loadAndPublishGeometry()
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // geometry
    mesh_msgs::msg::MeshGeometryStamped geometryMsg;

    auto vertices = io.getVertices();
    auto faceIds = io.getFaceIds();
    auto vertexNormals = io.getVertexNormals();

    getVertices(vertices, geometryMsg);
    getFaces(faceIds, geometryMsg);
    getVertexNormals(vertexNormals, geometryMsg);

    pub_geometry_->publish(geometryMsg);

    // vertex colors
    mesh_msgs::msg::MeshVertexColorsStamped vertexColorsMsg;

    auto vertexColors = io.getVertexColors();
    getVertexColors(vertexColors, vertexColorsMsg);

    pub_vertex_colors_->publish(vertexColorsMsg);

    // vertex costs
    mesh_msgs::msg::MeshVertexCostsStamped vertexCostsMsg;
    for (std::string costlayer : io.getCostLayers())
    {
        try
        {
            auto costs = io.getVertexCosts(costlayer);
            getVertexCosts(costs, costlayer, vertexCostsMsg);

            pub_vertex_costs_->publish(vertexCostsMsg);
        }
        catch (const hf::DataSpaceException& e)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Could not load costlayer " << costlayer);
        }
    }
}

bool hdf5_to_msg::getVertices(
    std::vector<float>& vertices,
    mesh_msgs::msg::MeshGeometryStamped& geometryMsg)
{
    unsigned int nVertices = vertices.size() / 3;
    RCLCPP_INFO_STREAM(this->get_logger(), "Found " << nVertices << " vertices");
    geometryMsg.mesh_geometry.vertices.resize(nVertices);
    for (unsigned int i = 0; i < nVertices; i++)
    {
        geometryMsg.mesh_geometry.vertices[i].x = vertices[i * 3];
        geometryMsg.mesh_geometry.vertices[i].y = vertices[i * 3 + 1];
        geometryMsg.mesh_geometry.vertices[i].z = vertices[i * 3 + 2];
    }

    // Header
    geometryMsg.uuid = mesh_uuid;
    geometryMsg.header.frame_id = "map";
    geometryMsg.header.stamp = this->get_clock()->now();

    return true;
}

bool hdf5_to_msg::getFaces(
    std::vector<uint32_t>& faceIds,
    mesh_msgs::msg::MeshGeometryStamped& geometryMsg)
{
    unsigned int nFaces = faceIds.size() / 3;
    RCLCPP_INFO_STREAM(this->get_logger(), "Found " << nFaces << " faces");
    geometryMsg.mesh_geometry.faces.resize(nFaces);
    for (unsigned int i = 0; i < nFaces; i++)
    {
        geometryMsg.mesh_geometry.faces[i].vertex_indices[0] = faceIds[i * 3];
        geometryMsg.mesh_geometry.faces[i].vertex_indices[1] = faceIds[i * 3 + 1];
        geometryMsg.mesh_geometry.faces[i].vertex_indices[2] = faceIds[i * 3 + 2];
    }

    // Header
    geometryMsg.uuid = mesh_uuid;
    geometryMsg.header.frame_id = "map";
    geometryMsg.header.stamp = this->get_clock()->now();

    return true;
}

bool hdf5_to_msg::getVertexNormals(
    std::vector<float>& vertexNormals,
    mesh_msgs::msg::MeshGeometryStamped& geometryMsg)
{
    unsigned int nVertexNormals = vertexNormals.size() / 3;
    RCLCPP_INFO_STREAM(this->get_logger(), "Found " << nVertexNormals << " vertex normals");
    geometryMsg.mesh_geometry.vertex_normals.resize(nVertexNormals);
    for (unsigned int i = 0; i < nVertexNormals; i++)
    {
        geometryMsg.mesh_geometry.vertex_normals[i].x = vertexNormals[i * 3];
        geometryMsg.mesh_geometry.vertex_normals[i].y = vertexNormals[i * 3 + 1];
        geometryMsg.mesh_geometry.vertex_normals[i].z = vertexNormals[i * 3 + 2];
    }

    // Header
    geometryMsg.uuid = mesh_uuid;
    geometryMsg.header.frame_id = "map";
    geometryMsg.header.stamp = this->get_clock()->now();

    return true;
}

bool hdf5_to_msg::getVertexColors(
    std::vector<uint8_t>& vertexColors,
    mesh_msgs::msg::MeshVertexColorsStamped& vertexColorsMsg)
{
    unsigned int nVertices = vertexColors.size() / 3;
    RCLCPP_INFO_STREAM(this->get_logger(), "Found " << nVertices << " vertices for vertex colors");
    vertexColorsMsg.mesh_vertex_colors.vertex_colors.resize(nVertices);
    for (unsigned int i = 0; i < nVertices; i++)
    {
        vertexColorsMsg.mesh_vertex_colors
            .vertex_colors[i].r = vertexColors[i * 3] / 255.0f;
        vertexColorsMsg.mesh_vertex_colors
            .vertex_colors[i].g = vertexColors[i * 3 + 1] / 255.0f;
        vertexColorsMsg.mesh_vertex_colors
            .vertex_colors[i].b = vertexColors[i * 3 + 2] / 255.0f;
        vertexColorsMsg.mesh_vertex_colors
            .vertex_colors[i].a = 1;
    }

    // Header
    vertexColorsMsg.uuid = mesh_uuid;
    vertexColorsMsg.header.frame_id = "map";
    vertexColorsMsg.header.stamp = this->get_clock()->now();

    return true;
}

bool hdf5_to_msg::getVertexCosts(
    std::vector<float>& costs, 
    std::string layer,
    mesh_msgs::msg::MeshVertexCostsStamped& vertexCostsMsg)
{
    vertexCostsMsg.mesh_vertex_costs.costs.resize(costs.size());
    for (uint32_t i = 0; i < costs.size(); i++)
    {
        vertexCostsMsg.mesh_vertex_costs.costs[i] = costs[i];
    }

    vertexCostsMsg.uuid = mesh_uuid;
    vertexCostsMsg.type = layer;
    vertexCostsMsg.header.frame_id = "map";
    vertexCostsMsg.header.stamp = this->get_clock()->now();

    return true;
}

bool hdf5_to_msg::service_getUUIDs(
    const std::shared_ptr<mesh_msgs::srv::GetUUIDs::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetUUIDs::Response> res)
{
    res->uuids.push_back(mesh_uuid);
    return true;
}

bool hdf5_to_msg::service_getGeometry(
    const std::shared_ptr<mesh_msgs::srv::GetGeometry::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetGeometry::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Vertices
    auto vertices = io.getVertices();
    getVertices(vertices, res->mesh_geometry_stamped);

    // Faces
    auto faceIds = io.getFaceIds();
    getFaces(faceIds, res->mesh_geometry_stamped);

    // Vertex normals
    auto vertexNormals = io.getVertexNormals();
    getVertexNormals(vertexNormals, res->mesh_geometry_stamped);

    return true;
}

bool hdf5_to_msg::service_getGeometryVertices(
    const std::shared_ptr<mesh_msgs::srv::GetGeometry::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetGeometry::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Vertices
    auto vertices = io.getVertices();
    return getVertices(vertices, res->mesh_geometry_stamped);
}

bool hdf5_to_msg::service_getGeometryFaces(
    const std::shared_ptr<mesh_msgs::srv::GetGeometry::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetGeometry::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Faces
    auto faceIds = io.getFaceIds();
    return getFaces(faceIds, res->mesh_geometry_stamped);
}

bool hdf5_to_msg::service_getGeometryVertexNormals(
    const std::shared_ptr<mesh_msgs::srv::GetGeometry::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetGeometry::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Vertex normals
    auto vertexNormals = io.getVertexNormals();
    return getVertexNormals(vertexNormals, res->mesh_geometry_stamped);
}

bool hdf5_to_msg::service_getMaterials(
    const std::shared_ptr<mesh_msgs::srv::GetMaterials::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetMaterials::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);


    // Materials
    auto materials = io.getMaterials();
    auto materialFaceIndices = io.getMaterialFaceIndices(); // for each face: material index
    unsigned int nMaterials = materials.size();
    unsigned int nFaces = materialFaceIndices.size();
    RCLCPP_INFO_STREAM(this->get_logger(), "Found " << nMaterials << " materials and " << nFaces << " faces");
    res->mesh_materials_stamped.mesh_materials.materials.resize(nMaterials);
    for (uint32_t i = 0; i < nMaterials; i++)
    {
        int texture_index = materials[i].textureIndex;

        // has texture
        res->mesh_materials_stamped
            .mesh_materials
            .materials[i]
            .has_texture = texture_index >= 0;

        // texture index
        res->mesh_materials_stamped.mesh_materials.materials[i]
            .texture_index = static_cast<uint32_t>(texture_index);
        // color
        res->mesh_materials_stamped.mesh_materials.materials[i]
            .color.r = materials[i].r / 255.0f;
        res->mesh_materials_stamped.mesh_materials.materials[i]
            .color.g = materials[i].g / 255.0f;
        res->mesh_materials_stamped.mesh_materials.materials[i]
            .color.b = materials[i].b / 255.0f;
        res->mesh_materials_stamped.mesh_materials.materials[i]
            .color.a = 1;
    }

    // Clusters
    // Map materials to face IDs
    std::map<uint32_t, std::vector<uint32_t>> materialToFaces;
    // Iterate over face <> material index
    for (uint32_t i = 0; i < nFaces; i++)
    {
        // material index for face i
        uint32_t materialIndex = materialFaceIndices[i];

        if (materialToFaces.count(materialIndex) == 0)
        {
            materialToFaces.insert(std::make_pair(materialIndex, std::vector<uint32_t>()));
        }

        materialToFaces[materialIndex].push_back(i);
    }
    // For each material, map contains a list of faces
    res->mesh_materials_stamped.mesh_materials.clusters.resize(nMaterials);
    for (uint32_t i = 0; i < nMaterials; i++)
    {
        for (uint32_t j = 0; j < materialToFaces[i].size(); j++)
        {
            res->mesh_materials_stamped.mesh_materials.clusters[i].face_indices.push_back(materialToFaces[i][j]);
        }
    }
    res->mesh_materials_stamped.mesh_materials.cluster_materials.resize(nMaterials);
    for (uint32_t i = 0; i < nMaterials; i++)
    {
        res->mesh_materials_stamped.mesh_materials.cluster_materials[i] = i;
    }

    // Vertex Tex Coords
    auto vertexTexCoords = io.getVertexTextureCoords();
    unsigned int nVertices = vertexTexCoords.size() / 3;
    res->mesh_materials_stamped.mesh_materials.vertex_tex_coords.resize(nVertices);
    for (uint32_t i = 0; i < nVertices; i++)
    {
        // coords: u/v/w
        // w is always 0
        res->mesh_materials_stamped.mesh_materials.vertex_tex_coords[i].u = vertexTexCoords[3 * i];
        res->mesh_materials_stamped.mesh_materials.vertex_tex_coords[i].v = vertexTexCoords[3 * i + 1];
    }

    // Header
    res->mesh_materials_stamped.uuid = mesh_uuid;
    res->mesh_materials_stamped.header.frame_id = "map";
    res->mesh_materials_stamped.header.stamp = this->get_clock()->now();

    return true;
}

bool hdf5_to_msg::service_getTexture(
    const std::shared_ptr<mesh_msgs::srv::GetTexture::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetTexture::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    for (auto texture : io.getTextures())
    {
        if (std::stoi(texture.name) == req->texture_index)
        {
            res->texture.texture_index = req->texture_index;
            res->texture.uuid = mesh_uuid;
            sensor_msgs::msg::Image image;
            sensor_msgs::fillImage( // TODO: only RGB, breaks when using other color channels
                image,
                "rgb8",
                texture.height,
                texture.width,
                texture.width * 3, // step size
                texture.data.data()
            );

            res->texture.image = image;

            return true;
        }
    }

    return false;
}

bool hdf5_to_msg::service_getVertexColors(
    const std::shared_ptr<mesh_msgs::srv::GetVertexColors::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetVertexColors::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Vertex colors
    auto vertexColors = io.getVertexColors();
    return getVertexColors(vertexColors, res->mesh_vertex_colors_stamped);
}

bool hdf5_to_msg::service_getVertexCosts(
    const std::shared_ptr<mesh_msgs::srv::GetVertexCosts::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetVertexCosts::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);
    
    auto costs = io.getVertexCosts(req->layer);
    return getVertexCosts(costs, req->layer, res->mesh_vertex_costs_stamped);
}

bool hdf5_to_msg::service_getVertexCostLayers(
    const std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    res->layers = io.getCostLayers();
    return true;
}

bool hdf5_to_msg::service_getLabeledClusters(
    const std::shared_ptr<mesh_msgs::srv::GetLabeledClusters::Request> req,
    std::shared_ptr<mesh_msgs::srv::GetLabeledClusters::Response> res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // iterate over groups
    auto groups = io.getLabelGroups();
    for (size_t i = 0; i < groups.size(); i++)
    {
        // iterate over labels in group
        auto labelsInGroup = io.getAllLabelsOfGroup(groups[i]);
        for (size_t j = 0; j < labelsInGroup.size(); j++)
        {
            // copy label
            auto faceIds = io.getFaceIdsOfLabel(groups[i], labelsInGroup[j]);
            mesh_msgs::msg::MeshFaceCluster cluster;
            std::stringstream ss;
            ss << groups[i] << "_" << labelsInGroup[j];
            cluster.label = ss.str();
            cluster.face_indices.resize(faceIds.size());
            for (size_t k = 0; k < faceIds.size(); k++)
            {
                cluster.face_indices[k] = faceIds[k];
            }
            res->clusters.push_back(cluster);
        }
    }

    return true;
}

void hdf5_to_msg::callback_clusterLabel(
    const mesh_msgs::msg::MeshFaceClusterStamped& msg)
{
    if (msg.uuid.compare(mesh_uuid) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid mesh UUID");
        return;
    }

    hdf5_map_io::HDF5MapIO io(inputFile);

    // TODO: implement optional override
    RCLCPP_WARN(this->get_logger(), "Override is enabled by default");

    // split label id into group and name
    std::vector<std::string> split_results;
    boost::split(split_results, msg.cluster.label, [](char c){ return c == '_'; });

    if (split_results.size() != 2)
    {
        RCLCPP_ERROR(this->get_logger(), "Received illegal cluster name");
        return;
    }


    std::string label_group = split_results[0];
    std::string label_name = split_results[1];
    std::vector<uint32_t> indices;
    for (size_t i = 0; i < msg.cluster.face_indices.size(); i++)
    {
        indices.push_back(msg.cluster.face_indices[i]);
    }

    // write to hdf5
    io.addLabel(label_group, label_name, indices);
}

} // namespace mesh_msgs_hdf5

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::ExecutorOptions opts;
    rclcpp::executors::MultiThreadedExecutor executor(opts, 4);
    executor.add_node(std::make_shared<mesh_msgs_hdf5::hdf5_to_msg>());
    executor.spin();
    return 0;
}
