/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  MeshVisual.hpp
 *
 *
 *  authors:
 *
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Henning Deeken <hdeeken@uni-osnabrueck.de>
 *    Marcel Mrozinski
 *    Nils Oesting
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 *    Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 */

#ifndef MESH_VISUAL_HPP
#define MESH_VISUAL_HPP

#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs/MeshGeometry.h>
#include <mesh_msgs/MeshVertexColorsStamped.h>
#include <mesh_msgs/MeshVertexColors.h>
#include <mesh_msgs/MeshVertexCostsStamped.h>
#include <mesh_msgs/MeshVertexCosts.h>
#include <mesh_msgs/MeshMaterialsStamped.h>
#include <mesh_msgs/MeshMaterials.h>
#include <mesh_msgs/MeshMaterial.h>
#include <mesh_msgs/MeshTexture.h>

#include <sensor_msgs/Image.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreColourValue.h>

#include <Types.hpp>
#include <vector>

namespace Ogre
{
// Forward declaration
class Vector3;
class Quaternion;
class SceneNode;
class Entity;

}  // End namespace Ogre

namespace rviz_map_plugin
{
/**
 * @brief Class to display mesh data in the main panel of rviz.
 */
class MeshVisual
{
public:
  /**
   * @brief Constructor.
   *
   * @param context The context that contains the display information.
   * @param displayID The display id
   * @param meshID The mesh id
   * @param randomID random number that will be used as part of the meshes UID
   */
  MeshVisual(rviz::DisplayContext* context, size_t displayID, size_t meshID, size_t randomID);

  /**
   * @brief Destructor.
   */
  virtual ~MeshVisual();

  /**
   * @brief Clears whole stored data.
   */
  void reset();

  /**
   * @brief Extracts data from the ros-messages and creates meshes.
   *
   * @param geometry Geometry containing the mesh
   */
  bool setGeometry(const Geometry& geometry);

  /**
   * @brief Passes the normal data to the mesh visual
   *
   * @param normals Vector containing the normal data
   */
  bool setNormals(const std::vector<Normal>& normals);

  /**
   * @brief Extracts data from the ros-messages and creates a colored mesh.
   *
   * @param vertexColors Vector containing the vertex color information
   */
  bool setVertexColors(const std::vector<Color>& vertexColors);

  /**
   * @brief Extracts data from the ros-messages and creates a colored mesh with colors calculated from vertex costs.
   *
   * @param vertexCosts Vector containing the vertex cost information
   */
  bool setVertexCosts(const std::vector<float>& vertexCosts);

  /**
   * @brief Extracts data from the ros-messages and creates a colored mesh with colors calculated from vertex costs.
   *
   * @param vertexCosts Vector containing the vertex cost information
   * @param costColorType colorization method (0 = rainbow; 1 = red-green)
   */
  bool setVertexCosts(const std::vector<float>& vertexCosts, int costColorType);

  /**
   * @brief Extracts data from the ros-messages and creates a colored mesh with colors calculated from vertex costs.
   *
   * @param vertexCosts Vector containing the vertex cost information
   * @param costColorType colorization method (0 = rainbow; 1 = red-green)
   * @param minCost minimum value for colorization
   * @param maxCost maximum value for colorization
   */
  bool setVertexCosts(const std::vector<float>& vertexCosts, int costColorType, float minCost, float maxCost);

  /**
   * @brief Extracts data from the ros-messages and creates a textured mesh.
   *
   * @param materials Vector containing all materials
   * @param texCoords Vector containing all texture coordinates
   */
  bool setMaterials(const vector<Material>& materials, const vector<TexCoords>& texCoords);

  /**
   * @brief Extracts data from the ros-messages and adds textures to the textured mesh.
   *
   * @param texture       Texture containing the texture information and data
   * @param textureIndex  Index of the texture
   */
  bool addTexture(Texture& texture, uint32_t textureIndex);

  /**
   * @brief Sets the pose of the coordinate frame the message refers to.
   *
   * @param position The pose of the coordinate frame
   */
  void setFramePosition(const Ogre::Vector3& position);

  /**
   * @brief Sets the orientation of the coordinate frame the message refers to.
   *
   * @param orientation The orientation of the coordinate frame
   */
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  /**
   * @brief Updates the visible parts of the mesh depending on input from the rviz display.
   *
   * @param showFaces             When TRUE faces are visible
   * @param facesColor            The color of the faces
   * @param facesAlpha            The transparency of the faces
   * @param useVertexColors       When TRUE vertex colors are used
   * @param showVertexCosts       When TRUE vertex costs are visible
   * @param showTextures          When TRUE textures are visible
   * @param showTexturedFacesOnly When TRUE only textured faces are visible
   */
  void updateMaterial(bool showFaces, Ogre::ColourValue facesColor, float facesAlpha, bool useVertexColors,
                      bool showVertexCosts, bool showTextures, bool showTexturedFacesOnly);

  /**
   * @brief Updates the visible parts of the mesh depending on input from the rviz display.
   *
   * @param showWireframe         When TRUE wireframe is visible
   * @param wireframeColor        The color of the wireframe
   * @param wireframeAlpha        The transparency of the wireframe
   * @param showFaces             When TRUE faces are visible
   * @param facesColor            The color of the faces
   * @param facesAlpha            The transparency of the faces
   * @param useVertexColors       When TRUE vertex colors are used
   * @param showVertexCosts       When TRUE vertex costs are visible
   * @param showTextures          When TRUE textures are visible
   * @param showTexturedFacesOnly When TRUE only textured faces are visible
   * @param showNormals           When TRUE normals are visible
   * @param normalsColor          The color of the normals
   * @param normalsAlpha          The transparency of the normals
   * @param normalsScallingFactor The size of the normals
   */
  void updateMaterial(bool showWireframe, Ogre::ColourValue wireframeColor, float wireframeAlpha, bool showFaces,
                      Ogre::ColourValue facesColor, float facesAlpha, bool useVertexColors, bool showVertexCosts,
                      bool showTextures, bool showTexturedFacesOnly, bool showNormals, Ogre::ColourValue normalsColor,
                      float normalsAlpha, float normalsScallingFactor);

  /**
   * @brief Updates the size of the normals dynamically.
   *
   * @param scallingFactor The factor the normals have to be scaled with
   */
  void updateNormals(float scallingFactor);

  /**
   * @brief Updates the normals dynamically.
   *
   * @param showNormals       When TRUE normals are visible
   * @param normalsColor      The color of the normals
   * @param normalsAlpha      The transparency of the normals
   */
  void updateNormals(bool showNormals, Ogre::ColourValue normalsColor, float normalsAlpha);

  /**
   * @brief Updates the normals dynamically.
   *
   * @param showNormals       When TRUE normals are visible
   * @param normalsColor      The color of the normals
   * @param normalsAlpha      The transparency of the normals
   * @param scalingFactor     The factor the normals have to be scaled with
   */
  void updateNormals(bool showNormals, Ogre::ColourValue normalsColor, float normalsAlpha, float scalingFactor);

  /**
   * @brief Updates the wireframe dynamically.
   *
   * @param showWireframe     When TRUE wireframe is visible
   * @param wireframeColor    The color of the wireframe
   * @param wireframeAlpha    The transparency of the wireframe
   */
  void updateWireframe(bool showWireframe, Ogre::ColourValue wireframeColor, float wireframeAlpha);

private:
  /**
   * @brief Enables the wireframe
   * @param pass Ogre Pass
   * @param wireframeColor The color of the wireframe
   * @param wireframeAlpha Transparency of the wireframe
   */
  void showWireframe(Ogre::Pass* pass, Ogre::ColourValue wireframeColor, float wireframeAlpha);

  /**
   * @brief
   */
  void showFaces(Ogre::Pass* pass, Ogre::ColourValue facesColor, float facesAlpha, bool useVertexColors);

  void showNormals(Ogre::Pass* pass, Ogre::ColourValue normalsColor, float normalsAlpha);

  void showTextures(Ogre::Pass* pass);

  void enteringGeneralTriangleMesh(const Geometry& mesh);

  void enteringColoredTriangleMesh(const Geometry& mesh, const vector<Color>& vertexColors);

  void enteringTriangleMeshWithVertexCosts(const Geometry& mesh, const vector<float>& vertexCosts, int costColorType);
  void enteringTriangleMeshWithVertexCosts(const Geometry& mesh, const vector<float>& vertexCosts, int costColorType,
                                           float minCost, float maxCost);

  void enteringTexturedTriangleMesh(const Geometry& mesh, const vector<Material>& meshMaterials,
                                    const vector<TexCoords>& texCoords);

  void enteringNormals(const Geometry& mesh, const vector<Normal>& normals);

  Ogre::PixelFormat getOgrePixelFormatFromRosString(std::string encoding);

  void loadImageIntoTextureMaterial(size_t textureIndex);

  /**
   *
   * @brief Calculates a color for a given cost value using a spectrum from red to green.
   *
   * @param cost The cost value (should be within the range 0 - 1)
   *
   * @return calculated color
   */
  Ogre::ColourValue calculateColorFromCost(float cost, int costColorType);

  bool m_vertex_normals_enabled;
  bool m_vertex_colors_enabled;
  bool m_vertex_costs_enabled;
  bool m_materials_enabled;
  bool m_texture_coords_enabled;
  bool m_textures_enabled;

  /// Ogre Scenenode
  Ogre::SceneNode* m_sceneNode;

  /// The context that contains the display information.
  rviz::DisplayContext* m_displayContext;

  /// First ID of the created mesh
  size_t m_prefix;

  /// Second ID of the created mesh
  size_t m_postfix;

  /// Random ID of the created mesh
  size_t m_random;

  /// The mesh-object to display
  Ogre::ManualObject* m_mesh;

  /// The manual object to display normals
  Ogre::ManualObject* m_normals;

  /// The manual object to display the mesh with vertex costs
  Ogre::ManualObject* m_vertexCostsMesh;

  /// The manual object to display the textured mesh
  Ogre::ManualObject* m_texturedMesh;

  /// The manual object to display the not textured parts of the textured mesh
  Ogre::ManualObject* m_noTexCluMesh;

  // The textures for the mesh
  std::vector<Ogre::Image> m_images;

  // The material for the textured mesh
  Ogre::MaterialPtr m_texturedMeshMaterial;

  /// The material for the general mesh
  Ogre::MaterialPtr m_meshGeneralMaterial;

  /// The material for the textured triangle mesh
  Ogre::MaterialPtr m_meshTexturedTrianglesMaterial;

  /// The material of the normals
  Ogre::MaterialPtr m_normalMaterial;

  /// The materials of the not textured clusters
  Ogre::MaterialPtr m_noTexCluMaterial;

  /// The material of the mesh with vertex costs
  Ogre::MaterialPtr m_vertexCostMaterial;

  /// The materials of the textures
  std::vector<Ogre::MaterialPtr> m_textureMaterials;

  /// Factor the normal-size is multiplied with.
  float m_normalsScalingFactor;

  /// raw Triangle Mesh
  Geometry m_geometry;

  /// raw normals
  std::vector<Normal> m_geometryNormals;
};
}  // End namespace rviz_map_plugin

#endif
