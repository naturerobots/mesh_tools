#include "hdf5_map_io/hdf5_map_io.h"
#include <hdf5_hl.h>


namespace hdf5_map_io
{

HDF5MapIO::HDF5MapIO(std::string filename)
    : m_file(filename, hf::File::ReadWrite)
{
    if (!m_file.exist(GEOMETRY_GROUP) ||
        !m_file.exist(ATTRIBUTES_GROUP) ||
        !m_file.exist(CLUSTERSETS_GROUP) ||
        !m_file.exist(TEXTURES_GROUP) ||
        !m_file.exist(LABELS_GROUP))
    {
        throw "No valid map h5 file";
    }

    m_geometryGroup = m_file.getGroup(GEOMETRY_GROUP);
    m_attributesGroup = m_file.getGroup(ATTRIBUTES_GROUP);
    m_clusterSetsGroup = m_file.getGroup(CLUSTERSETS_GROUP);
    m_texturesGroup = m_file.getGroup(TEXTURES_GROUP);
    m_labelsGroup = m_file.getGroup(LABELS_GROUP);
}

HDF5MapIO::HDF5MapIO(
    std::string filename,
    const std::vector<float>& vertices,
    const std::vector<uint32_t>& face_ids
)
    : m_file(filename, hf::File::ReadWrite | hf::File::Create | hf::File::Truncate)
{

    if (!m_file.isValid())
    {
        throw "Could not open file.";
    }

    // Create top level groups
    m_geometryGroup = m_file.createGroup(GEOMETRY_GROUP);
    m_attributesGroup = m_file.createGroup(ATTRIBUTES_GROUP);
    m_clusterSetsGroup = m_file.createGroup(CLUSTERSETS_GROUP);
    m_texturesGroup = m_file.createGroup(TEXTURES_GROUP);
    m_labelsGroup = m_file.createGroup(LABELS_GROUP);

    // Create geometry data sets
    m_geometryGroup
        .createDataSet<float>("vertices", hf::DataSpace::From(vertices))
        .write(vertices);
    m_geometryGroup
        .createDataSet<uint32_t>("faces", hf::DataSpace::From(face_ids))
        .write(face_ids);
}

HDF5MapIO::~HDF5MapIO()
{
    if (!m_file.isValid())
    {
        // do nothing if file is not valid, i.e. already closed
        return;
    }

    H5Gclose(m_geometryGroup.getId());
    H5Gclose(m_attributesGroup.getId());
    H5Gclose(m_clusterSetsGroup.getId());
    H5Gclose(m_texturesGroup.getId());
    H5Gclose(m_labelsGroup.getId());

    H5Fclose(m_file.getId());
}


std::vector<float> HDF5MapIO::getVertices()
{
    std::vector<float> vertices;

    if (!m_geometryGroup.exist("vertices"))
    {
        return vertices;
    }

    m_geometryGroup.getDataSet("vertices")
        .read(vertices);

    return vertices;
}

std::vector<uint32_t> HDF5MapIO::getFaceIds()
{
    std::vector<uint32_t> faceIds;

    if (!m_geometryGroup.exist("faces"))
    {
        return faceIds;
    }

    m_geometryGroup.getDataSet("faces")
        .read(faceIds);

    return faceIds;
}

std::vector<float> HDF5MapIO::getVertexNormals()
{
    std::vector<float> normals;

    if (!m_attributesGroup.exist("normals"))
    {
        return normals;
    }

    m_attributesGroup.getDataSet("normals")
        .read(normals);

    return normals;
}

std::vector<uint8_t> HDF5MapIO::getVertexColors()
{
    std::vector<uint8_t> rgbColors;

    if (!m_attributesGroup.exist("rgb_colors"))
    {
        return rgbColors;
    }

    m_attributesGroup.getDataSet("rgb_colors")
        .read(rgbColors);

    return rgbColors;
}

std::vector<MapImage> HDF5MapIO::getTextures()
{
    std::vector<MapImage> textures;

    if (!m_texturesGroup.exist("images"))
    {
        return textures;
    }

    const hf::Group& imagesGroup = m_texturesGroup.getGroup("images");
    for (auto setName: imagesGroup.listObjectNames())
    {
        textures.push_back(getImage(imagesGroup, setName));
    }

    return textures;
}

std::unordered_map<MapVertex, std::vector<float>> HDF5MapIO::getFeatures()
{
    std::unordered_map<MapVertex, std::vector<float>> features;

    if (!m_attributesGroup.exist("texture_features"))
    {
        return features;
    }

    const auto& featuresGroup = m_attributesGroup.getGroup("texture_features");
    features.reserve(featuresGroup.getNumberObjects());

    for (auto name : featuresGroup.listObjectNames())
    {
        // fill vector with descriptor
        std::vector<float> descriptor;
        auto dataset = featuresGroup.getDataSet(name);
        dataset.read(descriptor);

        // read vector attribute with xyz coords
        MapVertex v;
        std::vector<float> xyz(3);
        auto vector_attr = dataset.getAttribute("vector");
        vector_attr.read(xyz);

        v.x = xyz[0];
        v.y = xyz[1];
        v.z = xyz[2];

        features.insert({v, descriptor});
    }

    return features;
}

std::vector<MapMaterial> HDF5MapIO::getMaterials()
{
    std::vector<MapMaterial> materials;

    if (!m_texturesGroup.exist("materials"))
    {
        return materials;
    }

    m_texturesGroup.getDataSet("materials")
        .read(materials);

    return materials;
}

std::vector<uint32_t> HDF5MapIO::getMaterialFaceIndices()
{
    std::vector<uint32_t> matFaceIndices;

    if (!m_texturesGroup.exist("mat_face_indices"))
    {
        return matFaceIndices;
    }

    m_texturesGroup.getDataSet("mat_face_indices")
        .read(matFaceIndices);

    return matFaceIndices;
}

std::vector<float> HDF5MapIO::getVertexTextureCoords()
{
    std::vector<float> coords;

    if (!m_texturesGroup.exist("coords"))
    {
        return coords;
    }

    m_texturesGroup.getDataSet("coords")
        .read(coords);

    return coords;
}

std::vector<std::string> HDF5MapIO::getLabelGroups()
{
    return m_labelsGroup.listObjectNames();
}

std::vector<std::string> HDF5MapIO::getAllLabelsOfGroup(std::string groupName)
{
    if (!m_labelsGroup.exist(groupName))
    {
        return std::vector<std::string>();
    }

    return m_labelsGroup.getGroup(groupName).listObjectNames();
}

std::vector<uint32_t> HDF5MapIO::getFaceIdsOfLabel(std::string groupName, std::string labelName)
{
    std::vector<uint32_t> faceIds;

    if (!m_labelsGroup.exist(groupName))
    {
        return faceIds;
    }

    auto lg = m_labelsGroup.getGroup(groupName);

    if (!lg.exist(labelName))
    {
        return faceIds;
    }

    lg.getDataSet(labelName).read(faceIds);

    return faceIds;
}

std::vector<float> HDF5MapIO::getRoughness()
{
    std::vector<float> roughness;

    if (!m_attributesGroup.exist("roughness"))
    {
        return roughness;
    }

    m_attributesGroup.getDataSet("roughness")
        .read(roughness);

    return roughness;
}

std::vector<float> HDF5MapIO::getHeightDifference()
{
    std::vector<float> diff;

    if (!m_attributesGroup.exist("height_difference"))
    {
        return diff;
    }

    m_attributesGroup.getDataSet("height_difference")
        .read(diff);

    return diff;
}

MapImage HDF5MapIO::getImage(hf::Group group, std::string name)
{
    MapImage t;

    if (!group.exist(name))
    {
        return t;
    }

    hsize_t width;
    hsize_t height;
    hsize_t pixel_size;
    char interlace[20];
    hssize_t npals;

    H5IMget_image_info(group.getId(), name.c_str(), &width, &height, &pixel_size, interlace, &npals);

    auto bufSize = width * height * pixel_size;
    std::vector<unsigned char> buf;
    buf.resize(bufSize);
    H5IMread_image(group.getId(), name.c_str(), buf.data());

    t.name = name;
    t.width = width;
    t.height = height;
    t.channels = pixel_size;
    t.data = buf;

    return t;
}

hf::DataSet HDF5MapIO::addVertexNormals(std::vector<float>& normals)
{
    // TODO make more versatile to add and/or overwrite normals in file
    auto dataSet = m_attributesGroup.createDataSet<float>("normals", hf::DataSpace::From(normals));
    dataSet.write(normals);

    return dataSet;
}

hf::DataSet HDF5MapIO::addVertexColors(std::vector<uint8_t>& colors)
{
    auto dataSet = m_attributesGroup.createDataSet<uint8_t>("rgb_colors", hf::DataSpace::From(colors));
    dataSet.write(colors);

    return dataSet;
}

void HDF5MapIO::addTexture(int index, uint32_t width, uint32_t height, uint8_t *data)
{
    if (!m_texturesGroup.exist("images"))
    {
        m_texturesGroup.createGroup("images");
    }

    auto imagesGroup = m_texturesGroup.getGroup("images");
    const std::string& name = std::to_string(index);

    if (imagesGroup.exist(name))
    {
        return;
    }

    addImage(imagesGroup, name, width, height, data);
}

void HDF5MapIO::addMaterials(std::vector<MapMaterial>& materials, std::vector<uint32_t>& matFaceIndices)
{
    m_texturesGroup
        .createDataSet<MapMaterial>("materials", hf::DataSpace::From(materials))
        .write(materials);

    m_texturesGroup
        .createDataSet<uint32_t>("mat_face_indices", hf::DataSpace::From(matFaceIndices))
        .write(matFaceIndices);
}

void HDF5MapIO::addVertexTextureCoords(std::vector<float>& coords)
{
    m_texturesGroup
        .createDataSet<float>("coords", hf::DataSpace::From(coords))
        .write(coords);
}

void HDF5MapIO::addLabel(std::string groupName, std::string labelName, std::vector<uint32_t>& faceIds)
{
    if (!m_labelsGroup.exist(groupName))
    {
        m_labelsGroup.createGroup(groupName);
    }

    m_labelsGroup.getGroup(groupName)
        .createDataSet<uint32_t>(labelName, hf::DataSpace::From(faceIds))
        .write(faceIds);
}

void HDF5MapIO::addTextureKeypointsMap(std::unordered_map<MapVertex, std::vector<float>>& keypoints_map)
{
    if (!m_attributesGroup.exist("texture_features"))
    {
        m_attributesGroup.createGroup("texture_features");
    }

    auto tf = m_attributesGroup.getGroup("texture_features");

    size_t i = 0;
    for (const auto& keypoint_features : keypoints_map)
    {
        auto dataset = tf.createDataSet<float>(std::to_string(i), hf::DataSpace::From(keypoint_features.second));
        dataset.write(keypoint_features.second);

        std::vector<float> v = {keypoint_features.first.x, keypoint_features.first.y, keypoint_features.first.z};
        dataset.template createAttribute<float>("vector", hf::DataSpace::From(v))
            .write(v);

        i++;
    }
}

void HDF5MapIO::addRoughness(std::vector<float>& roughness)
{
    m_attributesGroup.createDataSet<float>("roughness", hf::DataSpace::From(roughness))
        .write(roughness);
}

void HDF5MapIO::addHeightDifference(std::vector<float>& diff)
{
    m_attributesGroup.createDataSet<float>("height_difference", hf::DataSpace::From(diff))
        .write(diff);
}

void HDF5MapIO::addImage(hf::Group group, std::string name, const uint32_t width, const uint32_t height,
                                 const uint8_t *pixelBuffer)
{
    H5IMmake_image_24bit(group.getId(), name.c_str(), width, height, "INTERLACE_PIXEL", pixelBuffer);
}

bool HDF5MapIO::removeAllLabels()
{
    bool result = true;
    for (std::string name : m_labelsGroup.listObjectNames())
    {
        std::string fullPath = std::string(LABELS_GROUP) + "/" + name;
        result = H5Ldelete(m_file.getId(), fullPath.data(), H5P_DEFAULT) > 0;
    }

    return result;
}

void HDF5MapIO::flush()
{
    m_file.flush();
}

} // namespace hdf5_map_io

