#ifndef HDF5_MAP_IO__H_
#define HDF5_MAP_IO__H_

#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>

#include <H5Tpublic.h>
#include <highfive/H5File.hpp>

namespace hf = HighFive;

namespace hdf5_map_io
{

/**
 * Helper struct to save vertices to the map.
 */
struct MapVertex {
    float x;
    float y;
    float z;
};


/**
 * Helper struct to save textures / images to the map.
 */
struct MapImage {
    std::string name;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    std::vector<uint8_t> data;
};

/**
 * Helper struct for saving material data to the map.
 *
 * This struct is defined as an HDF compound data type.
 */
struct MapMaterial {
    int32_t textureIndex;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

/**
 * This class if responsible for the map format. It tries to abstract most if not all calls to the
 * underlying HDF5 API and the HighFive wrapper. Furthermore it ensures the defined map format is always
 * in place and not tinkered with.
 *
 * NOTE: the map file is held open for the whole live time of this object. Thus it is possible that some data is
 * only written to disc if the destructor is called or the program has ended. Also make sure the map file is not opened
 * in any other way. (i.e. with the HDF5 Viewer). This will always lead to errors trying to access the file.
 */
class HDF5MapIO
{
public:
    /**
     * @brief Opens a map file for reading and writing.
     */
    HDF5MapIO(std::string filename);

    /**
     * @brief Creates a map file (or truncates if the file already exists).
     */
    HDF5MapIO(
        std::string filename,
        const std::vector<float>& vertices,
        const std::vector<uint32_t>& face_ids
    );

    /**
     * @brief Closes main groups and makes sure all buffers are flushed to the file on disc.
     */
    ~HDF5MapIO();

    /**
     * @brief Returns vertices vector
     */
    std::vector<float> getVertices();

    /**
     * @brief Returns face ids vector
     */
    std::vector<uint32_t> getFaceIds();

    /**
     * @brief Returns vertex normals vector
     */
    std::vector<float> getVertexNormals();

    /**
     * @brief Returns vertex colors vector
     */
    std::vector<uint8_t> getVertexColors();

    /**
     * @brief Returns textures vector
     */
    std::vector<MapImage> getTextures();

    /**
     * @brief Returns an map which keys are representing the features point in space and the values
     * are an vector of floats representing the keypoints.
     */
    std::unordered_map<MapVertex, std::vector<float>> getFeatures();

    /**
     * @brief Returns materials as MapMaterial
     */
    std::vector<MapMaterial> getMaterials();

    /**
     * @brief Returns material <-> face indices
     */
    std::vector<uint32_t> getMaterialFaceIndices();

    /**
     * @brief Returns vertex texture coordinates
     */
    std::vector<float> getVertexTextureCoords();

    /**
     * @brief Returns all available label groups
     */
    std::vector<std::string> getLabelGroups();

    /**
     * @brief  Returns all labels inside the given group
     */
    std::vector<std::string> getAllLabelsOfGroup(std::string groupName);

    /**
     * @brief Returns face ids for the given label inside the group.
     * E.g: label=tree_1 -> groupName=tree; labelName=1
     */
    std::vector<uint32_t> getFaceIdsOfLabel(std::string groupName, std::string labelName);

    /**
     * @brief Returns the roughness as float vector.
     */
    std::vector<float> getRoughness();

    /**
     * @brief Returns the height difference as float vector.
     */
    std::vector<float> getHeightDifference();

    /**
     * @brief Returns one costlayer as float vector.
     */
    std::vector<float> getVertexCosts(std::string costlayer);

    /**
     * @brief returns the names of all available costlayers
     */
    std::vector<std::string> getCostLayers();

    /**
     * @brief Returns the image in the group, if it exists. If not an empty struct is returned
     */
    MapImage getImage(hf::Group group, std::string name);

    /**
     * @brief Add normals to the attributes group.
     */
    hf::DataSet addVertexNormals(std::vector<float>& normals);

    /**
     * @brief Add vertex colors to the attributes group.
     */
    hf::DataSet addVertexColors(std::vector<uint8_t>& colors);

    /**
     * Add texture img with given index to the textures group. Texture CAN NOT be overridden
     */
    void addTexture(int index, uint32_t width, uint32_t height, uint8_t* data);

    /**
     * @brief Add materials as MapMaterial and the corresponding material <-> face indices
     */
    void addMaterials(std::vector<MapMaterial>& materials, std::vector<uint32_t>& matFaceIndices);

    /**
     * @brief Add vertex texture coordinates to the textures group.
     */
    void addVertexTextureCoords(std::vector<float>& coords);

    /**
     * @brief Adds the label (labelName) to the label group with the given faces.
     * E.g.: tree_1 -> groupName=tree; labelName=1; separated by the '_'
     */
    void addLabel(std::string groupName, std::string labelName, std::vector<uint32_t>& faceIds);

    /**
     * @brief Adds or updates the label (labelName) to the label group with the given faces.
     * E.g.: tree_1 -> groupName=tree; labelName=1; separated by the '_'
     */
    void addOrUpdateLabel(std::string groupName, std::string labelName, std::vector<uint32_t>& faceIds);

    /**
     * @brief Adds the keypoints with their corresponding positions to the attributes_group. The position
     * is saved to the entry via an attribute called 'vector'.
     */
    void addTextureKeypointsMap(std::unordered_map<MapVertex, std::vector<float>>& keypoints_map);

    /**
     * @brief Adds the roughness to the attributes group.
     */
    void addRoughness(std::vector<float>& roughness);

    /**
     * @brief Adds the height difference to the attributes group.
     */
    void addHeightDifference(std::vector<float>& diff);

    /**
     * @brief Adds an image with given data set name to the given group
     */
    void addImage(hf::Group group,
                  std::string name,
                  const uint32_t width,
                  const uint32_t height,
                  const uint8_t* pixelBuffer
    );

    /**
     * Removes all labels from the file.
     * <br>
     * Be careful, this does not clear up the space of the labels. Use the cli tool 'h5repack' manually to clear up
     * all wasted space if this method was used multiple times.
     *
     * @return true if removing all labels successfully.
     */
    bool removeAllLabels();

    /**
     * @brief Flushes the file. All opened buffers are saved to disc.
     */
    void flush();

private:
    hf::File m_file;

    void creatOrGetGroups();

    size_t getSize(hf::DataSet& data_set);
    // group names
    static constexpr const char* CHANNELS_GROUP = "/mesh/channels";
    static constexpr const char* CLUSTERSETS_GROUP = "/mesh/clustersets";
    static constexpr const char* TEXTURES_GROUP = "/mesh/textures";
    static constexpr const char* LABELS_GROUP = "/mesh/labels";

    // main groups for reference
    hf::Group m_channelsGroup;
    hf::Group m_clusterSetsGroup;
    hf::Group m_texturesGroup;
    hf::Group m_labelsGroup;
};
} // namespace hdf5_map_io


namespace HighFive {

/**
 * Define the MapMaterial as an HDF5 compound data type.
 */
template <>
inline AtomicType<hdf5_map_io::MapMaterial>::AtomicType()
{
    hid_t materialHid = H5Tcreate(H5T_COMPOUND, sizeof(hdf5_map_io::MapMaterial));

    H5Tinsert(materialHid, "textureIndex", offsetof(hdf5_map_io::MapMaterial, textureIndex), H5T_NATIVE_INT);
    H5Tinsert(materialHid, "r", offsetof(hdf5_map_io::MapMaterial, r), H5T_NATIVE_UCHAR);
    H5Tinsert(materialHid, "g", offsetof(hdf5_map_io::MapMaterial, g), H5T_NATIVE_UCHAR);
    H5Tinsert(materialHid, "b", offsetof(hdf5_map_io::MapMaterial, b), H5T_NATIVE_UCHAR);

    _hid = H5Tcopy(materialHid);
}

}

namespace std
{
template <>
struct hash<hdf5_map_io::MapVertex>
{
  size_t operator()(const hdf5_map_io::MapVertex& k) const
  {
      size_t h1 = std::hash<double>()(k.x);
      size_t h2 = std::hash<double>()(k.y);
      size_t h3 = std::hash<double>()(k.z);
      return (h1 ^ (h2 << 1)) ^ h3;
  }
};

template <>
struct equal_to<hdf5_map_io::MapVertex>
{
  bool operator()( const hdf5_map_io::MapVertex& lhs, const hdf5_map_io::MapVertex& rhs ) const{
      return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z);
  }
};

}

#endif // HDF5_MAP_IO__H_
