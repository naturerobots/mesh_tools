#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreRenderOperation.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreHardwarePixelBuffer.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>

#include <stdint.h>

#include "trianglemesh_visual.h"


namespace rviz_mesh_plugin
{

  TriangleMeshVisual::TriangleMeshVisual(
    rviz::DisplayContext* context,
      size_t displayID,
      size_t meshID,
      size_t randomID)
    : m_displayContext(context),
    m_prefix(displayID),
    m_postfix(meshID),
    m_random(randomID),
    m_vertex_normals_enabled(false),
    m_vertex_colors_enabled(false),
    m_triangle_colors_enabled(false),
    m_texture_coords_enabled(false),
    m_textureCounter(0),
    m_normalsScalingFactor(1)
  {

    ROS_INFO("Creating TriangleMeshVisual %lu_TriangleMesh_%lu_%lu",m_prefix, m_postfix, m_random);

    // get or create the scene node
    Ogre::SceneManager* sceneManager = m_displayContext->getSceneManager();
    Ogre::SceneNode* rootNode = sceneManager->getRootSceneNode();

    std::stringstream strstream;
    strstream << "TriangleMeshScene" << m_random;
    std::string sceneId = strstream.str();
    if (sceneManager->hasSceneNode(sceneId)){
      //ROS_INFO("Attaching to scene: %s", sceneId);
      m_sceneNode = (Ogre::SceneNode*)(rootNode->getChild(sceneId));
    }else{
      //ROS_INFO("Creating new scene: %s", sceneId);
      m_sceneNode = rootNode->createChildSceneNode(sceneId);
    }

    // create manual object and attach it to the scene node
    std::stringstream sstm;
    sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random;
    m_mesh = sceneManager->createManualObject(sstm.str());
    m_mesh->setDynamic(false);
    m_sceneNode->attachObject(m_mesh);
  }

  TriangleMeshVisual::~TriangleMeshVisual()
  {
    ROS_INFO("Destroying TriangleMeshVisual %lu_TriangleMesh_%lu_%lu",m_prefix, m_postfix, m_random);

    std::stringstream sstm;
    reset();
    sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random;
    m_displayContext->getSceneManager()->destroyManualObject(sstm.str());
    m_displayContext->getSceneManager()->destroySceneNode(m_sceneNode);
    sstm.str("");
    sstm.flush();
  }

  void TriangleMeshVisual::reset()
  {

    ROS_INFO("Resetting TriangleMeshVisual %lu_TriangleMesh_%lu_%lu",m_prefix, m_postfix, m_random);


    std::stringstream sstm;

    for (size_t i = 0; i < m_textureCounter; i++)
    {
      sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random << "Texture_" << i;
      Ogre::TextureManager::getSingleton().unload(sstm.str());
      Ogre::TextureManager::getSingleton().remove(sstm.str());
      sstm.str("");
      sstm.clear();
    }

    sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random << "Material_" << 0;
    Ogre::MaterialManager::getSingleton().unload(sstm.str());
    Ogre::MaterialManager::getSingleton().remove(sstm.str());
    sstm.str("");
    sstm.clear();

    sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random << "Material_" << 1;
    Ogre::MaterialManager::getSingleton().unload(sstm.str());
    Ogre::MaterialManager::getSingleton().remove(sstm.str());
    sstm.str("");
    sstm.clear();

    sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random << "NormalMaterial";
    Ogre::MaterialManager::getSingleton().unload(sstm.str());
    Ogre::MaterialManager::getSingleton().remove(sstm.str());
    sstm.str("");
    sstm.clear();

    m_textureCounter = 0;
    m_fakeTextures.clear();
    m_hasTextures = false;
    m_mesh->clear();
    sstm.str("");
    sstm.flush();
    m_meshGeneralMaterial.setNull();
    m_meshColoredTrianglesMaterial.setNull();
    m_normalMaterial.setNull();
  }

  void TriangleMeshVisual::showWireframe(
    Ogre::Pass* pass,
    Ogre::ColourValue wireframeColor,
    float wireframeAlpha
  ){
        pass->setAmbient(
          Ogre::ColourValue(
            wireframeColor.r,
            wireframeColor.g,
            wireframeColor.b,
            wireframeAlpha
          )
        );
        pass->setDiffuse(
          Ogre::ColourValue(
            wireframeColor.r,
            wireframeColor.g,
            wireframeColor.b,
            wireframeAlpha
          )
        );

        if (wireframeAlpha < 1.0){
          pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
          pass->setDepthWriteEnabled(false);
        }
        pass->setPolygonMode(Ogre::PM_WIREFRAME);
        pass->setCullingMode(Ogre::CULL_NONE);
  }

  void TriangleMeshVisual::showFaces(
    Ogre::Pass* pass,
    Ogre::ColourValue facesColor,
    float facesAlpha,
    bool useVertexColors
  ){

    pass->setDiffuse(
      Ogre::ColourValue(
        facesColor.r,
        facesColor.g,
        facesColor.b,
        facesAlpha
      )
    );
    pass->setSelfIllumination(facesColor.r, facesColor.g, facesColor.b);


    if(useVertexColors){
      pass->setLightingEnabled(false);
    }

    if (facesAlpha < 1.0)
    {
      pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      pass->setDepthWriteEnabled(false);
    }
    pass->setPolygonMode(Ogre::PM_SOLID);
    pass->setCullingMode(Ogre::CULL_NONE);
  }

  void TriangleMeshVisual::showNormals(
    Ogre::Pass* pass,
    Ogre::ColourValue normalsColor,
    float normalsAlpha
  ){

    pass->setSelfIllumination(normalsColor.r, normalsColor.g, normalsColor.b);
    pass->setDiffuse(
      Ogre::ColourValue(
        normalsColor.r,
        normalsColor.g,
        normalsColor.b,
        normalsAlpha
      )
    );
    if (normalsAlpha < 1.0)
    {
      pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      pass->setDepthWriteEnabled(false);
    }
    pass->setPolygonMode(Ogre::PM_SOLID);
    pass->setCullingMode(Ogre::CULL_NONE);
  }

  void TriangleMeshVisual::showTextures(
    Ogre::Pass* pass){
        //TODO
  }

    void TriangleMeshVisual::updateMaterial(
        bool showWireframe,
        Ogre::ColourValue wireframeColor,
        float wireframeAlpha,
        bool showFaces,
        Ogre::ColourValue facesColor,
        float facesAlpha,
        bool useVertexColors,
        bool useTriangleColors,
        bool showTextures,
        bool showNormals,
        Ogre::ColourValue normalsColor,
        float normalsAlpha,
        float normalsScalingFactor)
  {
    // remove all passes
    if (!m_meshGeneralMaterial.isNull()){
      m_meshGeneralMaterial->getTechnique(0)->removeAllPasses();
    }

    if(!m_meshColoredTrianglesMaterial.isNull()){
      m_meshColoredTrianglesMaterial->getTechnique(0)->removeAllPasses();
    }

    if (!m_normalMaterial.isNull()){
      m_normalMaterial->getTechnique(0)->removeAllPasses();
    }

    bool triangleColorsEnabled = m_triangle_colors_enabled && useTriangleColors;

    // if the material exists and the triangle colors are not enabled
    // because, if the triangle colors are are enabled we have to use the
    // mesh with the "vertex for each triangle" structure and
    // the m_meshColoredTrianglesMaterial otherwise we can use this
    // general mesh with the m_meshGeneralMaterial
    if (!m_meshGeneralMaterial.isNull() && !triangleColorsEnabled){
      Ogre::Technique* tech = m_meshGeneralMaterial->getTechnique(0);

      if (showFaces){
        this->showFaces(tech->createPass(), facesColor, facesAlpha, useVertexColors);
      }

      if (showWireframe){
        this->showWireframe(tech->createPass(), wireframeColor, wireframeAlpha);
      }

      if (showTextures && m_hasTextures){
        this->showTextures(tech->createPass());
      }
    }

    if (!m_meshColoredTrianglesMaterial.isNull() && triangleColorsEnabled){
      Ogre::Technique* tech = m_meshColoredTrianglesMaterial->getTechnique(0);

      if (showFaces){
        this->showFaces(tech->createPass(), facesColor, facesAlpha, true);
      }

      if (showWireframe){
        this->showWireframe(tech->createPass(), wireframeColor, wireframeAlpha);
      }

      if (showTextures && m_hasTextures){
        this->showTextures(tech->createPass());
      }
    }

    if(!m_normalMaterial.isNull()){
      if (showNormals){
        Ogre::Technique* tech = m_normalMaterial->getTechnique(0);
        this->showNormals(tech->createPass(), normalsColor, normalsAlpha);
        updateNormals(normalsScalingFactor);
      }
    }
  }

void TriangleMeshVisual::updateNormals(float ScalingFactor)
{
  Ogre::VertexData* vertexData;
  const Ogre::VertexElement* vertexElement;
  Ogre::HardwareVertexBufferSharedPtr vertexBuffer;
  unsigned char* vertexChar;
  float* vertexFloat;

  vertexData = m_mesh->getSection(m_mesh->getNumSections() - 1)->getRenderOperation()->vertexData;
  vertexElement = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
  vertexBuffer = vertexData->vertexBufferBinding->getBuffer(vertexElement->getSource());
  vertexChar = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

  size_t halfVertexCount = vertexData->vertexCount/2;
  Ogre::Vector3* vertices = new Ogre::Vector3[halfVertexCount];
  Ogre::Vector3* normals = new Ogre::Vector3[halfVertexCount];

  for(size_t i = 0, vIndex = 0, nIndex = 0; i < vertexData->vertexCount; i++, vertexChar += vertexBuffer->getVertexSize())
  {
    vertexElement->baseVertexPointerToElement(vertexChar, &vertexFloat);
    Ogre::Vector3 tempVector(vertexFloat[0], vertexFloat[1], vertexFloat[2]);

    if (i % 2 == 0)
    {
      vertices[vIndex] = tempVector;
      vIndex++;
    }

    else
    {
      normals[nIndex] = (tempVector - vertices[nIndex]) / m_normalsScalingFactor;
      nIndex++;
    }
  }
  vertexBuffer->unlock();

  m_mesh->beginUpdate(m_mesh->getNumSections() - 1);
  for (size_t i = 0; i < halfVertexCount; i++)
  {
    m_mesh->position(vertices[i].x, vertices[i].y, vertices[i].z);
    m_mesh->position(vertices[i].x + ScalingFactor * normals[i].x,
        vertices[i].y + ScalingFactor * normals[i].y,
        vertices[i].z + ScalingFactor * normals[i].z);
  }
  m_mesh->end();
  delete [] vertices;
  delete [] normals;
  m_normalsScalingFactor = ScalingFactor;
}



void TriangleMeshVisual::enteringColoredTriangleMesh(const mesh_msgs::TriangleMesh& mesh){

  if(!m_triangle_colors_enabled){
    return;
  }

  std::stringstream sstm;
  sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random << "Material_" << 1;

  m_meshColoredTrianglesMaterial =
    Ogre::MaterialManager::getSingleton().create(
        sstm.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true
        );

  // start entering data
  m_mesh->begin(sstm.str(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // write vertices for each triangle
  // to enable a coloring for each triangle

  // write triangle colors as vertex colours
  // write vertex normals(if enabled)

  size_t triangle_vertex_cnt = 0;
  for(size_t i = 0; i < mesh.triangles.size(); i++){
    // write three triangle vertices
    for(size_t j = 0; j < 3; j++){
      int vertex_index = mesh.triangles[i].vertex_indices[j];
      // write vertex positions
      m_mesh->position(
          mesh.vertices[vertex_index].x,
          mesh.vertices[vertex_index].y,
          mesh.vertices[vertex_index].z
          );
      // write triangle colors
      m_mesh->colour(
          mesh.triangle_colors[i].r,
          mesh.triangle_colors[i].g,
          mesh.triangle_colors[i].b,
          mesh.triangle_colors[i].a
          );
      // write vertex normals, if enabled
      if(m_vertex_normals_enabled){
        m_mesh->normal(
            mesh.vertex_normals[vertex_index].x,
            mesh.vertex_normals[vertex_index].y,
            mesh.vertex_normals[vertex_index].z
            );
      }
    }
    // write the three trianlge vertex indices
    m_mesh->triangle(
        triangle_vertex_cnt,
        triangle_vertex_cnt+1,
        triangle_vertex_cnt+2
        );
    triangle_vertex_cnt += 3;
  }

  // finish entering data
  m_mesh->end();
}

void TriangleMeshVisual::enteringGeneralTriangleMesh(const mesh_msgs::TriangleMesh& mesh){

  std::stringstream sstm;

  sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random << "Material_" << 0;

  m_meshGeneralMaterial =
    Ogre::MaterialManager::getSingleton().create(
        sstm.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true
        );

  // start entering data
  m_mesh->begin(sstm.str(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // write vertices
  // write color(if enabled)
  // write exture coords(if enabled)
  // write vertex normals(if enabled)
  for (size_t i = 0; i < mesh.vertices.size(); i++)
  {
    // write vertex
    m_mesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);

    // write vertex color, if enabled
    if(m_vertex_colors_enabled){
      m_mesh->colour(
          mesh.vertex_colors[i].r,
          mesh.vertex_colors[i].g,
          mesh.vertex_colors[i].b,
          mesh.vertex_colors[i].a
          );
    }

    // write texture coord, if enabled
    if(m_texture_coords_enabled){
      m_mesh->textureCoord(
          mesh.vertex_texture_coords[i].x,
          mesh.vertex_texture_coords[i].y,
          mesh.vertex_texture_coords[i].z
          );
    }

    // write vertex normals, if enabled
    if(m_vertex_normals_enabled){
      m_mesh->normal(
          mesh.vertex_normals[i].x,
          mesh.vertex_normals[i].y,
          mesh.vertex_normals[i].z
          );
    }
  }

  // write triangles
  for (size_t i = 0; i < mesh.triangles.size(); i++)
  {
    m_mesh->triangle(
        mesh.triangles[i].vertex_indices[0],
        mesh.triangles[i].vertex_indices[1],
        mesh.triangles[i].vertex_indices[2]
        );
  }

  // finish entering data
  m_mesh->end();

}

void TriangleMeshVisual::enteringNormals(const mesh_msgs::TriangleMesh& mesh){

  if(!m_vertex_normals_enabled){
    return;
  }

  std::stringstream sstm;
  sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random << "NormalMaterial";
  m_normalMaterial = Ogre::MaterialManager::getSingleton().create(sstm.str(),
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

  // Create pointNormals
  m_mesh->begin(sstm.str(), Ogre::RenderOperation::OT_LINE_LIST);

  // Vertices
  for (size_t i = 0; i < mesh.vertex_normals.size(); i++)
  {
    m_mesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
    m_mesh->position(
        mesh.vertices[i].x + m_normalsScalingFactor * mesh.vertex_normals[i].x,
        mesh.vertices[i].y + m_normalsScalingFactor * mesh.vertex_normals[i].y,
        mesh.vertices[i].z + m_normalsScalingFactor * mesh.vertex_normals[i].z);
    // add line to index buffer
    m_mesh->index(2*i);
    m_mesh->index(2*i+1);
  }
  m_mesh->end();

}

void TriangleMeshVisual::setMessage(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg)
{
  reset();

  // for a better legibility of the code
  const mesh_msgs::TriangleMesh& mesh = mesh_msg->mesh;

  // default: vertex colors are optional and therefore disabled
  m_vertex_colors_enabled = false;

  // default: triangle colors are optional and therefore disabled
  m_triangle_colors_enabled = false;

  // default: textures and texture_coords are optional and therefore disabled
  m_texture_coords_enabled = false;

  // default: vertex normals are optional and therefore disabled
  m_vertex_normals_enabled = false;

  // check if there are enough vertices given
  if (mesh.vertices.size() < 3){
    ROS_WARN("Received not enough vertices, can't create mesh!");
    return;
  }

  // defines the buffer sizes
  int vertex_count = mesh.vertices.size();
  int index_count = mesh.triangles.size() * 3;


  // vertex colors
  // check if there are vertex colors for each vertex
  if(mesh.vertex_colors.size() == mesh.vertices.size()){
    ROS_INFO("Received %lu vertex colors.", mesh.vertex_colors.size());
    m_vertex_colors_enabled = true;
  }
  else if(mesh.vertex_colors.size() > 0){
    ROS_WARN("Received not as much vertex colors as vertices, ignoring the vertex colors!");
  }

  // triangle colors
  // check if there are vertex colors for each vertex
  if(mesh.triangle_colors.size() == mesh.triangles.size()){
    ROS_INFO("Received %lu triangle colors.", mesh.triangle_colors.size());
    m_triangle_colors_enabled = true;

    // space for the extra mesh
    vertex_count += 3 * mesh.triangles.size();
    index_count += 3 * mesh.triangles.size();
  }
  else if(mesh.triangle_colors.size() > 0){
    ROS_WARN("Received not as much triangle colors as triangles, ignoring the triangle colors!");
  }

  // texture coords
  // check if there are texture coords for each vertex
  if(mesh.vertex_texture_coords.size() == mesh.vertices.size()){
    ROS_INFO("Received %lu texture coords.", mesh.vertex_texture_coords.size());
    m_texture_coords_enabled = true; // enable texture coords
  }
  else if(mesh.vertex_texture_coords.size() > 0){
    ROS_WARN("Received not as much texture coords as vertices, ignoring texture coords!");
  }

  // vertex normals
  // check if there are vertex normals for each vertex
  if(mesh.vertex_normals.size() == mesh.vertices.size()){
    ROS_INFO("Received %lu vertex normals.", mesh.vertex_normals.size());
    m_vertex_normals_enabled = true;
    // space for the normals in the buffer
    vertex_count += mesh.vertices.size();
    index_count += mesh.vertices.size() * 2;
  }
  else if(mesh.vertex_normals.size() > 0){
    ROS_WARN("Received not as much vertex normals as vertices, ignoring vertex normals!");
  }

  // avoid memory reallocation
  m_mesh->estimateVertexCount(vertex_count);
  m_mesh->estimateIndexCount(index_count);

  // entering a general triangle mesh into the internal buffer
  enteringGeneralTriangleMesh(mesh);

  // entering a special mesh for colored triangles into the internal buffer
  if(m_triangle_colors_enabled){
    enteringColoredTriangleMesh(mesh);
  }

  // entering the normals into the internal buffer
  if(m_vertex_normals_enabled){
    enteringNormals(mesh);
  }
}

void TriangleMeshVisual::setFramePosition(const Ogre::Vector3& position)
{
  m_sceneNode->setPosition(position);
}

void TriangleMeshVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  m_sceneNode->setOrientation(orientation);
}

} // end namespace rviz_mesh_plugin
