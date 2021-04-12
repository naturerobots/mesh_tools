#include <mesh_msgs_hdf5/mesh_msgs_hdf5.h>
#include <hdf5_map_io/hdf5_map_io.h>

namespace mesh_msgs_hdf5 {

hdf5_to_msg::hdf5_to_msg()
{
    ros::NodeHandle nh("~");

    if (!nh.getParam("inputFile", inputFile))
    {
        inputFile = "/tmp/map.h5";
    }

    ROS_INFO_STREAM("Using input file: " << inputFile);

    srv_get_geometry_ = node_handle.advertiseService(
        "get_geometry", &hdf5_to_msg::service_getGeometry, this);
    srv_get_geometry_vertices_ = node_handle.advertiseService(
        "get_geometry_vertices", &hdf5_to_msg::service_getGeometryVertices, this);
    srv_get_geometry_faces_ = node_handle.advertiseService(
        "get_geometry_faces", &hdf5_to_msg::service_getGeometryFaces, this);
     srv_get_geometry_vertex_normals_ = node_handle.advertiseService(
        "get_geometry_vertexnormals", &hdf5_to_msg::service_getGeometryVertexNormals, this);
    srv_get_materials_ = node_handle.advertiseService(
        "get_materials", &hdf5_to_msg::service_getMaterials, this);
    srv_get_texture_ = node_handle.advertiseService(
        "get_texture", &hdf5_to_msg::service_getTexture, this);
    srv_get_uuids_ = node_handle.advertiseService(
        "get_uuids", &hdf5_to_msg::service_getUUIDs, this);
    srv_get_vertex_colors_ = node_handle.advertiseService(
        "get_vertex_colors", &hdf5_to_msg::service_getVertexColors, this);
    srv_get_vertex_costs_ = node_handle.advertiseService(
        "get_vertex_costs", &hdf5_to_msg::service_getVertexCosts, this);
    srv_get_vertex_cost_layers_ = node_handle.advertiseService(
        "get_vertex_cost_layers", &hdf5_to_msg::service_getVertexCostLayers, this);

    pub_geometry_ = node_handle.advertise<mesh_msgs::MeshGeometryStamped>("mesh/geometry", 1, true);
    pub_vertex_colors_ = node_handle.advertise<mesh_msgs::MeshVertexColorsStamped>("mesh/vertex_colors", 1, true);
    pub_vertex_costs_ = node_handle.advertise<mesh_msgs::MeshVertexCostsStamped>("mesh/vertex_costs", 1);


    srv_get_labeled_clusters_ = node_handle.advertiseService(
        "get_labeled_clusters", &hdf5_to_msg::service_getLabeledClusters, this);

    sub_cluster_label_ = node_handle.subscribe("cluster_label", 10, &hdf5_to_msg::callback_clusterLabel, this);

    loadAndPublishGeometry();
}

void hdf5_to_msg::loadAndPublishGeometry()
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // geometry
    mesh_msgs::MeshGeometryStamped geometryMsg;

    auto vertices = io.getVertices();
    auto faceIds = io.getFaceIds();
    auto vertexNormals = io.getVertexNormals();

    getVertices(vertices, geometryMsg);
    getFaces(faceIds, geometryMsg);
    getVertexNormals(vertexNormals, geometryMsg);

    pub_geometry_.publish(geometryMsg);

    // vertex colors
    mesh_msgs::MeshVertexColorsStamped vertexColorsMsg;

    auto vertexColors = io.getVertexColors();
    getVertexColors(vertexColors, vertexColorsMsg);

    pub_vertex_colors_.publish(vertexColorsMsg);

    // vertex costs
    mesh_msgs::MeshVertexCostsStamped vertexCostsMsg;
    for (std::string costlayer : io.getCostLayers())
    {
        try
        {
            auto costs = io.getVertexCosts(costlayer);
            getVertexCosts(costs, costlayer, vertexCostsMsg);

            pub_vertex_costs_.publish(vertexCostsMsg);
        }
        catch (const hf::DataSpaceException& e)
        {
            ROS_WARN_STREAM("Could not load costlayer " << costlayer);
        }
    }
}

bool hdf5_to_msg::getVertices(std::vector<float>& vertices, mesh_msgs::MeshGeometryStamped& geometryMsg)
{
    unsigned int nVertices = vertices.size() / 3;
    ROS_INFO_STREAM("Found " << nVertices << " vertices");
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
    geometryMsg.header.stamp = ros::Time::now();

    return true;
}

bool hdf5_to_msg::getFaces(std::vector<uint32_t>& faceIds, mesh_msgs::MeshGeometryStamped& geometryMsg)
{
    unsigned int nFaces = faceIds.size() / 3;
    ROS_INFO_STREAM("Found " << nFaces << " faces");
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
    geometryMsg.header.stamp = ros::Time::now();

    return true;
}

bool hdf5_to_msg::getVertexNormals(std::vector<float>& vertexNormals, mesh_msgs::MeshGeometryStamped& geometryMsg)
{
    unsigned int nVertexNormals = vertexNormals.size() / 3;
    ROS_INFO_STREAM("Found " << nVertexNormals << " vertex normals");
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
    geometryMsg.header.stamp = ros::Time::now();

    return true;
}

bool hdf5_to_msg::getVertexColors(std::vector<uint8_t>& vertexColors, mesh_msgs::MeshVertexColorsStamped& vertexColorsMsg)
{
    unsigned int nVertices = vertexColors.size() / 3;
    ROS_INFO_STREAM("Found " << nVertices << " vertices for vertex colors");
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
    vertexColorsMsg.header.stamp = ros::Time::now();

    return true;
}

bool hdf5_to_msg::getVertexCosts(std::vector<float>& costs, std::string layer, mesh_msgs::MeshVertexCostsStamped& vertexCostsMsg)
{
    vertexCostsMsg.mesh_vertex_costs.costs.resize(costs.size());
    for (uint32_t i = 0; i < costs.size(); i++)
    {
        vertexCostsMsg.mesh_vertex_costs.costs[i] = costs[i];
    }

    vertexCostsMsg.uuid = mesh_uuid;
    vertexCostsMsg.type = layer;
    vertexCostsMsg.header.frame_id = "map";
    vertexCostsMsg.header.stamp = ros::Time::now();

    return true;
}

bool hdf5_to_msg::service_getUUIDs(
    mesh_msgs::GetUUIDs::Request& req,
    mesh_msgs::GetUUIDs::Response& res)
{
    res.uuids.push_back(mesh_uuid);
    return true;
}

bool hdf5_to_msg::service_getGeometry(
    mesh_msgs::GetGeometry::Request& req,
    mesh_msgs::GetGeometry::Response& res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Vertices
    auto vertices = io.getVertices();
    getVertices(vertices, res.mesh_geometry_stamped);

    // Faces
    auto faceIds = io.getFaceIds();
    getFaces(faceIds, res.mesh_geometry_stamped);

    // Vertex normals
    auto vertexNormals = io.getVertexNormals();
    getVertexNormals(vertexNormals, res.mesh_geometry_stamped);

    return true;
}

bool hdf5_to_msg::service_getGeometryVertices(
    mesh_msgs::GetGeometry::Request& req,
    mesh_msgs::GetGeometry::Response& res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Vertices
    auto vertices = io.getVertices();
    return getVertices(vertices, res.mesh_geometry_stamped);
}

bool hdf5_to_msg::service_getGeometryFaces(
    mesh_msgs::GetGeometry::Request& req,
    mesh_msgs::GetGeometry::Response& res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Faces
    auto faceIds = io.getFaceIds();
    return getFaces(faceIds, res.mesh_geometry_stamped);
}

bool hdf5_to_msg::service_getGeometryVertexNormals(
    mesh_msgs::GetGeometry::Request& req,
    mesh_msgs::GetGeometry::Response& res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Vertex normals
    auto vertexNormals = io.getVertexNormals();
    return getVertexNormals(vertexNormals, res.mesh_geometry_stamped);
}

bool hdf5_to_msg::service_getMaterials(
    mesh_msgs::GetMaterials::Request& req,
    mesh_msgs::GetMaterials::Response& res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);


    // Materials
    auto materials = io.getMaterials();
    auto materialFaceIndices = io.getMaterialFaceIndices(); // for each face: material index
    unsigned int nMaterials = materials.size();
    unsigned int nFaces = materialFaceIndices.size();
    ROS_INFO_STREAM("Found " << nMaterials << " materials and " << nFaces << " faces");
    res.mesh_materials_stamped.mesh_materials.materials.resize(nMaterials);
    for (uint32_t i = 0; i < nMaterials; i++)
    {
        int texture_index = materials[i].textureIndex;

        // has texture
        res.mesh_materials_stamped
            .mesh_materials
            .materials[i]
            .has_texture = texture_index >= 0;

        // texture index
        res.mesh_materials_stamped.mesh_materials.materials[i]
            .texture_index = static_cast<uint32_t>(texture_index);
        // color
        res.mesh_materials_stamped.mesh_materials.materials[i]
            .color.r = materials[i].r / 255.0f;
        res.mesh_materials_stamped.mesh_materials.materials[i]
            .color.g = materials[i].g / 255.0f;
        res.mesh_materials_stamped.mesh_materials.materials[i]
            .color.b = materials[i].b / 255.0f;
        res.mesh_materials_stamped.mesh_materials.materials[i]
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
    res.mesh_materials_stamped.mesh_materials.clusters.resize(nMaterials);
    for (uint32_t i = 0; i < nMaterials; i++)
    {
        for (uint32_t j = 0; j < materialToFaces[i].size(); j++)
        {
            res.mesh_materials_stamped.mesh_materials.clusters[i].face_indices.push_back(materialToFaces[i][j]);
        }
    }
    res.mesh_materials_stamped.mesh_materials.cluster_materials.resize(nMaterials);
    for (uint32_t i = 0; i < nMaterials; i++)
    {
        res.mesh_materials_stamped.mesh_materials.cluster_materials[i] = i;
    }

    // Vertex Tex Coords
    auto vertexTexCoords = io.getVertexTextureCoords();
    unsigned int nVertices = vertexTexCoords.size() / 3;
    res.mesh_materials_stamped.mesh_materials.vertex_tex_coords.resize(nVertices);
    for (uint32_t i = 0; i < nVertices; i++)
    {
        // coords: u/v/w
        // w is always 0
        res.mesh_materials_stamped.mesh_materials.vertex_tex_coords[i].u = vertexTexCoords[3 * i];
        res.mesh_materials_stamped.mesh_materials.vertex_tex_coords[i].v = vertexTexCoords[3 * i + 1];
    }

    // Header
    res.mesh_materials_stamped.uuid = mesh_uuid;
    res.mesh_materials_stamped.header.frame_id = "map";
    res.mesh_materials_stamped.header.stamp = ros::Time::now();

    return true;
}

bool hdf5_to_msg::service_getTexture(
    mesh_msgs::GetTexture::Request& req,
    mesh_msgs::GetTexture::Response& res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    for (auto texture : io.getTextures())
    {
        if (std::stoi(texture.name) == req.texture_index)
        {
            res.texture.texture_index = req.texture_index;
            res.texture.uuid = mesh_uuid;
            sensor_msgs::Image image;
            sensor_msgs::fillImage( // TODO: only RGB, breaks when using other color channels
                image,
                "rgb8",
                texture.height,
                texture.width,
                texture.width * 3, // step size
                texture.data.data()
            );

            res.texture.image = image;

            return true;
        }
    }

    return false;
}

bool hdf5_to_msg::service_getVertexColors(
    mesh_msgs::GetVertexColors::Request& req,
    mesh_msgs::GetVertexColors::Response& res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    // Vertex colors
    auto vertexColors = io.getVertexColors();
    return getVertexColors(vertexColors, res.mesh_vertex_colors_stamped);
}

bool hdf5_to_msg::service_getVertexCosts(
    mesh_msgs::GetVertexCosts::Request& req,
    mesh_msgs::GetVertexCosts::Response& res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);
    
    auto costs = io.getVertexCosts(req.layer);
    return getVertexCosts(costs, req.layer, res.mesh_vertex_costs_stamped);
}

bool hdf5_to_msg::service_getVertexCostLayers(
    mesh_msgs::GetVertexCostLayers::Request &req,
    mesh_msgs::GetVertexCostLayers::Response &res)
{
    hdf5_map_io::HDF5MapIO io(inputFile);

    res.layers = io.getCostLayers();
    return true;
}

bool hdf5_to_msg::service_getLabeledClusters(
    mesh_msgs::GetLabeledClusters::Request& req,
    mesh_msgs::GetLabeledClusters::Response& res)
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
            mesh_msgs::MeshFaceCluster cluster;
            std::stringstream ss;
            ss << groups[i] << "_" << labelsInGroup[j];
            cluster.label = ss.str();
            cluster.face_indices.resize(faceIds.size());
            for (size_t k = 0; k < faceIds.size(); k++)
            {
                cluster.face_indices[k] = faceIds[k];
            }
            res.clusters.push_back(cluster);
        }
    }

    return true;
}

void hdf5_to_msg::callback_clusterLabel(const mesh_msgs::MeshFaceClusterStamped::ConstPtr& msg)
{
    if (msg->uuid.compare(mesh_uuid) != 0)
    {
        ROS_ERROR("Invalid mesh UUID");
        return;
    }

    hdf5_map_io::HDF5MapIO io(inputFile);

    // TODO: implement optional override
    ROS_WARN("Override is enabled by default");

    // split label id into group and name
    std::vector<std::string> split_results;
    boost::split(split_results, msg->cluster.label, [](char c){ return c == '_'; });

    if (split_results.size() != 2)
    {
        ROS_ERROR("Received illegal cluster name");
        return;
    }


    std::string label_group = split_results[0];
    std::string label_name = split_results[1];
    std::vector<uint32_t> indices;
    for (size_t i = 0; i < msg->cluster.face_indices.size(); i++)
    {
        indices.push_back(msg->cluster.face_indices[i]);
    }

    // write to hdf5
    io.addLabel(label_group, label_name, indices);
}

} // namespace mesh_msgs_hdf5

int main(int argc, char **args)
{
    ros::init(argc, args, "mesh_msgs_hdf5");
    mesh_msgs_hdf5::hdf5_to_msg hdf5_to_msg;
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
