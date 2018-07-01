#include <TexturedMeshVisual.hpp>

#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreRenderOperation.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgrePixelFormat.h>

#include <stdint.h>



namespace rviz_map_plugin
{

TexturedMeshVisual::TexturedMeshVisual(
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
    m_materials_enabled(false),
    m_texture_coords_enabled(false),
    m_normalsScalingFactor(1)
{

    ROS_DEBUG("Creating TexturedMeshVisual %lu_TexturedMesh_%lu_%lu",m_prefix, m_postfix, m_random);

    // get or create the scene node
    Ogre::SceneManager* sceneManager = m_displayContext->getSceneManager();
    Ogre::SceneNode* rootNode = sceneManager->getRootSceneNode();

    std::stringstream strstream;
    strstream << "TexturedMeshScene" << m_random;
    std::string sceneId = strstream.str();
    if (sceneManager->hasSceneNode(sceneId))
    {
        m_sceneNode = (Ogre::SceneNode*)(rootNode->getChild(sceneId));
    }
    else
    {
        m_sceneNode = rootNode->createChildSceneNode(sceneId);
    }

    // create manual objects and attach them to the scene node
    std::stringstream sstm;
    sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random;
    m_mesh = sceneManager->createManualObject(sstm.str());
    m_mesh->setDynamic(false);
    m_sceneNode->attachObject(m_mesh);

    std::stringstream sstmNormals;
    sstmNormals << m_prefix << "_Normals_" << m_postfix << "_" << m_random;
    m_normalMesh = sceneManager->createManualObject(sstmNormals.str());
    m_normalMesh->setDynamic(false);
    m_sceneNode->attachObject(m_normalMesh);

    std::stringstream sstmTexturedMesh;
    sstmTexturedMesh << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random;
    m_texturedMesh = sceneManager->createManualObject(sstmTexturedMesh.str());
    m_texturedMesh->setDynamic(false);
    m_sceneNode->attachObject(m_texturedMesh);

    std::stringstream sstmNoTexCluMesh;
    sstmNoTexCluMesh << m_prefix << "_NoTexCluMesh_" << m_postfix << "_" << m_random;
    m_noTexCluMesh = sceneManager->createManualObject(sstmNoTexCluMesh.str());
    m_noTexCluMesh->setDynamic(false);
    m_sceneNode->attachObject(m_noTexCluMesh);

    std::stringstream sstmVertexCostsMesh;
    sstmVertexCostsMesh << m_prefix << "_VertexCostsMesh_" << m_postfix << "_" << m_random;
    m_vertexCostsMesh = sceneManager->createManualObject(sstmVertexCostsMesh.str());
    m_vertexCostsMesh->setDynamic(false);
    m_sceneNode->attachObject(m_vertexCostsMesh);
}

TexturedMeshVisual::~TexturedMeshVisual()
{
    ROS_DEBUG("Destroying TexturedMeshVisual %lu_TexturedMesh_%lu_%lu",m_prefix, m_postfix, m_random);

    reset();

    std::stringstream sstm;
    sstm << m_prefix << "_TriangleMesh_" << m_postfix << "_" << m_random;
    m_displayContext->getSceneManager()->destroyManualObject(sstm.str());

    std::stringstream sstmNormals;
    sstmNormals << m_prefix << "_Normals_" << m_postfix << "_" << m_random;
    m_displayContext->getSceneManager()->destroyManualObject(sstmNormals.str());

    std::stringstream sstmTexturedMesh;
    sstmTexturedMesh << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random;
    m_displayContext->getSceneManager()->destroyManualObject(sstmTexturedMesh.str());

    std::stringstream sstmNoTexCluMesh;
    sstmNoTexCluMesh << m_prefix << "_NoTexCluMesh_" << m_postfix << "_" << m_random;
    m_displayContext->getSceneManager()->destroyManualObject(sstmNoTexCluMesh.str());

    std::stringstream sstmVertexCostsMesh;
    sstmVertexCostsMesh << m_prefix << "_VertexCostsMesh_" << m_postfix << "_" << m_random;
    m_displayContext->getSceneManager()->destroyManualObject(sstmVertexCostsMesh.str());

    m_displayContext->getSceneManager()->destroySceneNode(m_sceneNode);
    sstm.str("");
    sstm.flush();
}

void TexturedMeshVisual::reset()
{

    ROS_DEBUG("Resetting TexturedMeshVisual %lu_TexturedMesh_%lu_%lu",m_prefix, m_postfix, m_random);


    std::stringstream sstm;

    sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "GeneralMaterial_";
    Ogre::MaterialManager::getSingleton().unload(sstm.str());
    Ogre::MaterialManager::getSingleton().remove(sstm.str());
    sstm.str("");
    sstm.clear();

    if (m_vertex_colors_enabled)
    {
        sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "Material_" << 1;
        Ogre::MaterialManager::getSingleton().unload(sstm.str());
        Ogre::MaterialManager::getSingleton().remove(sstm.str());
        sstm.str("");
        sstm.clear();
    }

    sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "NormalMaterial";
    Ogre::MaterialManager::getSingleton().unload(sstm.str());
    Ogre::MaterialManager::getSingleton().remove(sstm.str());
    sstm.str("");
    sstm.clear();

    for (Ogre::MaterialPtr textureMaterial : m_textureMaterials)
    {
        Ogre::MaterialManager::getSingleton().unload(textureMaterial->getName());
        Ogre::MaterialManager::getSingleton().remove(textureMaterial->getName());
    }

    if (!m_noTexCluMaterial.isNull())
    {
        Ogre::MaterialManager::getSingleton().unload(m_noTexCluMaterial->getName());
        Ogre::MaterialManager::getSingleton().remove(m_noTexCluMaterial->getName());
    }

    if (!m_vertexCostMaterial.isNull())
    {
        Ogre::MaterialManager::getSingleton().unload(m_vertexCostMaterial->getName());
        Ogre::MaterialManager::getSingleton().remove(m_vertexCostMaterial->getName());
    }

    m_mesh->clear();
    m_normalMesh->clear();
    m_texturedMesh->clear();
    m_noTexCluMesh->clear();
    m_vertexCostsMesh->clear();
    sstm.str("");
    sstm.flush();

    m_meshGeneralMaterial.setNull();
    m_normalMaterial.setNull();
    m_noTexCluMaterial.setNull();
    m_textureMaterials.clear();
    m_vertexCostMaterial.setNull();

    m_images.clear();

    m_vertex_colors_enabled = false;
    m_materials_enabled = false;
    m_texture_coords_enabled = false;
    m_textures_enabled = false;
    m_vertex_costs_enabled = false;
}

void TexturedMeshVisual::showWireframe(
    Ogre::Pass* pass,
    Ogre::ColourValue wireframeColor,
    float wireframeAlpha
)
{
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

    if (wireframeAlpha < 1.0)
    {
        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->setDepthWriteEnabled(false);
    }
    pass->setPolygonMode(Ogre::PM_WIREFRAME);
    pass->setCullingMode(Ogre::CULL_NONE);
}

void TexturedMeshVisual::showFaces(
    Ogre::Pass* pass,
    Ogre::ColourValue facesColor,
    float facesAlpha,
    bool useVertexColors
)
{

    pass->setDiffuse(
        Ogre::ColourValue(
            facesColor.r,
            facesColor.g,
            facesColor.b,
            facesAlpha
        )
    );
    pass->setSelfIllumination(facesColor.r, facesColor.g, facesColor.b);


    if (useVertexColors)
    {
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

void TexturedMeshVisual::showNormals(
    Ogre::Pass* pass,
    Ogre::ColourValue normalsColor,
    float normalsAlpha
)
{

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

void TexturedMeshVisual::updateMaterial(
    bool showFaces,
    Ogre::ColourValue facesColor,
    float facesAlpha,
    bool useVertexColors,
    bool showVertexCosts,
    bool showTextures,
    bool showTexturedFacesOnly
)
{
    // remove the faces pass
    if (!m_meshGeneralMaterial.isNull())
    {
        Ogre::Technique* tech = m_meshGeneralMaterial->getTechnique(0);
        if (tech->getPass("faces") != 0)
        {
            tech->removePass(tech->getPass("faces")->getIndex());
        }
    }

    m_texturedMesh->setVisible(false);
    m_noTexCluMesh->setVisible(false);
    m_vertexCostsMesh->setVisible(false);

    // if the material exists and the textures are not enabled
    // we can use the general mesh with the m_meshGeneralMaterial
    if (!m_meshGeneralMaterial.isNull() && !showTextures && !showVertexCosts)
    {
        Ogre::Technique* tech = m_meshGeneralMaterial->getTechnique(0);

        if (showFaces)
        {
            Ogre::Pass* pass = tech->createPass();
            pass->setName("faces");
            this->showFaces(pass, facesColor, facesAlpha, useVertexColors);
        }
    }

    // if there are vertex costs and the vertex cost are enabled
    // the mesh with the colors calculated from vertex costs is made visible
    if (m_vertex_costs_enabled && showVertexCosts)
    {
        m_vertexCostsMesh->setVisible(true);
    }

    // if there are materials or textures the mesh with texture coordinates that
    // uses the material and texture materials is made visible
    if ((m_materials_enabled || m_textures_enabled) && showTextures)
    {
        m_texturedMesh->setVisible(true);
        m_noTexCluMesh->setVisible(!showTexturedFacesOnly);
    }
}

void TexturedMeshVisual::updateNormals(
    bool showNormals,
    Ogre::ColourValue normalsColor,
    float normalsAlpha,
    float scalingFactor
)
{
    if (!m_normalMaterial.isNull())
    {
        m_normalMaterial->getTechnique(0)->removeAllPasses();
    }

    if (!m_normalMaterial.isNull())
    {
        if (showNormals)
        {
            Ogre::Technique* tech = m_normalMaterial->getTechnique(0);
            this->showNormals(tech->createPass(), normalsColor, normalsAlpha);

            Ogre::VertexData* vertexData;
            const Ogre::VertexElement* vertexElement;
            Ogre::HardwareVertexBufferSharedPtr vertexBuffer;
            unsigned char* vertexChar;
            float* vertexFloat;

            vertexData = m_normalMesh->getSection(0)->getRenderOperation()->vertexData;
            vertexElement = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
            vertexBuffer = vertexData->vertexBufferBinding->getBuffer(vertexElement->getSource());
            vertexChar = static_cast<unsigned char*>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            size_t halfVertexCount = vertexData->vertexCount / 2;
            Ogre::Vector3* vertices = new Ogre::Vector3[halfVertexCount];
            Ogre::Vector3* normals = new Ogre::Vector3[halfVertexCount];

            for (
                size_t i = 0, vIndex = 0, nIndex = 0;
                i < vertexData->vertexCount;
                i++, vertexChar += vertexBuffer->getVertexSize()
            )
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

            m_normalMesh->beginUpdate(0);
            for (size_t i = 0; i < halfVertexCount; i++)
            {
                m_normalMesh->position(vertices[i].x, vertices[i].y, vertices[i].z);
                m_normalMesh->position(
                    vertices[i].x + scalingFactor * normals[i].x,
                    vertices[i].y + scalingFactor * normals[i].y,
                    vertices[i].z + scalingFactor * normals[i].z
                );
            }
            m_normalMesh->end();
            delete [] vertices;
            delete [] normals;
            m_normalsScalingFactor = scalingFactor;
        }
    }
}

void TexturedMeshVisual::updateWireframe(
    bool showWireframe,
    Ogre::ColourValue wireframeColor,
    float wireframeAlpha
)
{
    if (!m_meshGeneralMaterial.isNull())
    {
        Ogre::Technique* tech = m_meshGeneralMaterial->getTechnique(0);

        if (tech->getPass("wireframe") != 0)
        {
            tech->removePass(tech->getPass("wireframe")->getIndex());
        }

        if (showWireframe)
        {
            Ogre::Pass* pass = tech->createPass();
            pass->setName("wireframe");
            this->showWireframe(pass, wireframeColor, wireframeAlpha);
        }
    }

}

void TexturedMeshVisual::enteringGeneralTriangleMesh(const Geometry& mesh)
{

    std::stringstream sstm;

    sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "GeneralMaterial_";

    m_meshGeneralMaterial =
        Ogre::MaterialManager::getSingleton().create(
            sstm.str(),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            true
        );

    m_meshGeneralMaterial->getTechnique(0)->removeAllPasses();

    // start entering data
    m_mesh->begin(sstm.str(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // write vertices
    for (size_t i = 0; i < mesh.vertices.size(); i++)
    {
        // write vertices
        m_mesh->position(
            mesh.vertices[i].x,
            mesh.vertices[i].y,
            mesh.vertices[i].z
        );
    }

    // write triangles
    for (size_t i = 0; i < mesh.faces.size(); i++)
    {
        m_mesh->triangle(
            mesh.faces[i].vertexIndices[0],
            mesh.faces[i].vertexIndices[1],
            mesh.faces[i].vertexIndices[2]
        );
    }

    // finish entering data
    m_mesh->end();

}

void TexturedMeshVisual::enteringColoredTriangleMesh(
    const Geometry& mesh,
    const vector<Color>& vertexColors)
{

    if (m_meshGeneralMaterial.isNull())
    {
        std::stringstream sstm;
        sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "GeneralMaterial_";

        m_meshGeneralMaterial =
            Ogre::MaterialManager::getSingleton().create(
                sstm.str(),
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                true
            );

        m_meshGeneralMaterial->getTechnique(0)->removeAllPasses();
    }

    // start entering data
    m_mesh->begin(m_meshGeneralMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // write vertices
    // write vertex colors
    for (size_t i = 0; i < mesh.vertices.size(); i++)
    {
        // write vertices
        m_mesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);

        // write vertex colors
        m_mesh->colour(
            vertexColors[i].r,
            vertexColors[i].g,
            vertexColors[i].b,
            vertexColors[i].a
        );
    }

    // write triangles
    for (size_t i = 0; i < mesh.faces.size(); i++)
    {
        m_mesh->triangle(
            mesh.faces[i].vertexIndices[0],
            mesh.faces[i].vertexIndices[1],
            mesh.faces[i].vertexIndices[2]
        );
    }

    // finish entering data
    m_mesh->end();

}

void TexturedMeshVisual::enteringTriangleMeshWithVertexCosts(
    const Geometry& mesh,
    const vector<float>& vertexCosts)
{

    // Calculate maximum value for vertex costs
    float maxCost = 0.0f;
    for (float cost : vertexCosts)
    {
        maxCost = cost > maxCost ? cost : maxCost;
    }


    if (m_vertexCostMaterial.isNull())
    {
        std::stringstream sstm;
        sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "VertexCostMaterial_";

        m_vertexCostMaterial =
            Ogre::MaterialManager::getSingleton().create(
                sstm.str(),
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                true
            );

        Ogre::Pass* pass = m_vertexCostMaterial->getTechnique(0)->getPass(0);
        pass->setCullingMode(Ogre::CULL_NONE);
        pass->setLightingEnabled(false);
    }
    m_vertexCostsMesh->setVisible(false);

    // start entering data
    m_vertexCostsMesh->begin(m_vertexCostMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // write vertices
    // write vertex colors
    for (size_t i = 0; i < mesh.vertices.size(); i++)
    {
        // write vertices
        m_vertexCostsMesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);

        // write vertex colors that are calculated from the cost values
        m_vertexCostsMesh->colour(calculateColorFromCost(vertexCosts[i] / maxCost));
    }

    // write triangles
    for (size_t i = 0; i < mesh.faces.size(); i++)
    {
        m_vertexCostsMesh->triangle(
            mesh.faces[i].vertexIndices[0],
            mesh.faces[i].vertexIndices[1],
            mesh.faces[i].vertexIndices[2]
        );
    }

    // finish entering data
    m_vertexCostsMesh->end();

}

void TexturedMeshVisual::enteringTexturedTriangleMesh(
    const Geometry& mesh,
    const vector<Material>& materials,
    const vector<TexCoords>& texCoords
)
{
    std::stringstream sstm;
    sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "NoTexCluMaterial_";
    m_noTexCluMaterial = Ogre::MaterialManager::getSingleton().create(
        sstm.str(),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        true
    );

    m_texturedMesh->setVisible(false);
    m_noTexCluMesh->setVisible(false);

    Ogre::Pass* pass = m_noTexCluMaterial->getTechnique(0)->getPass(0);
    pass->setCullingMode(Ogre::CULL_NONE);
    pass->setLightingEnabled(false);


    m_noTexCluMesh->begin(m_noTexCluMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

    size_t noTexCluVertexCount = 0;

    size_t materialIndex = 0;

    for (auto& material : materials)
    {

        bool hasTexture = material.textureIndex? true : false;

        // if the material has a texture, create an ogre texture and load the image
        if (hasTexture)
        {
            uint32_t textureIndex = *(material.textureIndex);
            std::stringstream sstm;
            sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "TextureMaterial_" << textureIndex;
            m_textureMaterials.push_back(
                Ogre::MaterialManager::getSingleton().create(
                    sstm.str(),
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    true
                )
            );

            // set some rendering options for textured clusters
            Ogre::Pass* pass = m_textureMaterials[textureIndex]->getTechnique(0)->getPass(0);
            // pass->setTextureFiltering(Ogre::TFO_NONE);
            pass->setCullingMode(Ogre::CULL_NONE);
            pass->setLightingEnabled(false);

            // check if image was already loaded
            // this is the case if the vector of images doesn't contain this element yet or
            // if the image was only default constructed, in which case its width will be 0
            if (m_images.size() < textureIndex + 1 || m_images[textureIndex].getWidth() == 0)
            {
                ROS_DEBUG("Texture with index %u not loaded yet", textureIndex);
            }
            else
            {
                loadImageIntoTextureMaterial(textureIndex);
            }
        }


        if (hasTexture)
        {
            uint32_t textureIndex = *(material.textureIndex);
            // start entering data
            m_texturedMesh->begin(m_textureMaterials[textureIndex]->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

            // write vertices for each triangle
            // write texture coordinates

            size_t triangleVertexCount = 0;
            for (size_t i = 0; i < material.faceIndices.size(); i++)
            {
                uint32_t faceIndex = material.faceIndices[i];
                // write three triangle vertices
                for (size_t j = 0; j < 3; j++)
                {
                    uint32_t vertexIndex = mesh.faces[faceIndex].vertexIndices[j];
                    // write vertex positions
                    m_texturedMesh->position(
                        mesh.vertices[vertexIndex].x,
                        mesh.vertices[vertexIndex].y,
                        mesh.vertices[vertexIndex].z
                    );
                    // write texture coordinates
                    m_texturedMesh->textureCoord(
                        texCoords[vertexIndex].u,
                        1 - texCoords[vertexIndex].v
                    );
                }
                // write the three triangle vertex indices
                m_texturedMesh->triangle(
                    triangleVertexCount,
                    triangleVertexCount + 1,
                    triangleVertexCount + 2
                    );
                triangleVertexCount += 3;
            }

            // finish entering data
            m_texturedMesh->end();

        }
        else
        {
            // write vertices for each triangle to enable a coloring for each triangle
            // write triangle colors as vertex colours

            size_t triangleVertexCount = 0;
            for (size_t i = 0; i < material.faceIndices.size(); i++)
            {
                uint32_t faceIndex = material.faceIndices[i];
                // write three triangle vertices
                for (size_t j = 0; j < 3; j++)
                {
                    int vertexIndex = mesh.faces[faceIndex].vertexIndices[j];
                    // write vertex positions
                    m_noTexCluMesh->position(
                        mesh.vertices[vertexIndex].x,
                        mesh.vertices[vertexIndex].y,
                        mesh.vertices[vertexIndex].z
                    );

                    // write triangle colors
                    m_noTexCluMesh->colour(
                        material.color.r,
                        material.color.g,
                        material.color.b,
                        material.color.a
                    );
                }
                // write the three triangle vertex indices
                m_noTexCluMesh->triangle(
                    noTexCluVertexCount,
                    noTexCluVertexCount + 1,
                    noTexCluVertexCount + 2
                );
                noTexCluVertexCount += 3;
            }

        }
    }

    m_noTexCluMesh->end();
}

void TexturedMeshVisual::enteringNormals(const Geometry& mesh, const vector<Normal>& normals)
{

    if (!m_vertex_normals_enabled)
    {
        return;
    }

    std::stringstream sstm;
    sstm << m_prefix << "_TexturedMesh_" << m_postfix << "_" << m_random << "NormalMaterial";
    m_normalMaterial = Ogre::MaterialManager::getSingleton().create(
        sstm.str(),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        true
    );
    m_normalMaterial->getTechnique(0)->removeAllPasses();

    // Create pointNormals
    m_normalMesh->begin(sstm.str(), Ogre::RenderOperation::OT_LINE_LIST);

    // Vertices
    for (size_t i = 0; i < mesh.vertices.size(); i++)
    {
        m_normalMesh->position(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
        m_normalMesh->position(
            mesh.vertices[i].x + m_normalsScalingFactor * normals[i].x,
            mesh.vertices[i].y + m_normalsScalingFactor * normals[i].y,
            mesh.vertices[i].z + m_normalsScalingFactor * normals[i].z
        );
        // add line to index buffer
        m_normalMesh->index(2 * i);
        m_normalMesh->index(2 * i + 1);
    }
    m_normalMesh->end();

}

bool TexturedMeshVisual::setGeometry(const Geometry& mesh)
{
    reset();

    m_geometry = mesh;

    // default: vertex colors are optional and therefore disabled
    m_vertex_colors_enabled = false;

    // default: textures and texture_coords are optional and therefore disabled
    m_textures_enabled = false;
    m_texture_coords_enabled = false;

    // default: vertex normals are optional and therefore disabled
    m_vertex_normals_enabled = false;

    // default: vertex costs are optional and therefore disabled
    m_vertex_costs_enabled = false;

    // check if there are enough vertices given
    if (mesh.vertices.size() < 3)
    {
        ROS_WARN("Received not enough vertices, can't create mesh!");
        return false;
    }

    // defines the buffer sizes
    int vertex_count = mesh.vertices.size();
    int index_count = mesh.faces.size() * 3;

    // avoid memory reallocation
    m_mesh->estimateVertexCount(vertex_count);
    m_mesh->estimateIndexCount(index_count);

    // entering a general triangle mesh into the internal buffer
    enteringGeneralTriangleMesh(mesh);

    return true;
}

bool TexturedMeshVisual::setNormals(const vector<Normal>& normals)
{
    // vertex normals
    // check if there are vertex normals for each vertex
    if (normals.size() == m_geometry.vertices.size())
    {
        ROS_INFO("Received %lu vertex normals.", normals.size());
        m_vertex_normals_enabled = true;
    }
    else if (normals.size() > 0)
    {
        ROS_WARN("Received not as much vertex normals as vertices, ignoring vertex normals!");
        return false;
    }

    // avoid memory reallocation
    m_normalMesh->estimateVertexCount(m_geometry.vertices.size() * 2);
    m_normalMesh->estimateIndexCount(m_geometry.vertices.size() * 2);

    // entering the normals into the internal buffer
    if (m_vertex_normals_enabled)
    {
        enteringNormals(m_geometry, normals);
    }

    return true;
}

bool TexturedMeshVisual::setVertexColors(const vector<Color>& vertexColors)
{
    // check if there are vertex colors for each vertex
    if (vertexColors.size() == m_geometry.vertices.size())
    {
        ROS_INFO("Received %lu vertex colors.", vertexColors.size());
        m_vertex_colors_enabled = true;
    }
    else
    {
        ROS_WARN("Received not as much vertex colors as vertices, ignoring the vertex colors!");
        return false;
    }

    enteringColoredTriangleMesh(m_geometry, vertexColors);

    return true;
}

bool TexturedMeshVisual::setVertexCosts(const vector<float>& vertexCosts)
{
    // check if there are vertex costs for each vertex
    if (vertexCosts.size() == m_geometry.vertices.size())
    {
        ROS_DEBUG("Received %lu vertex costs.", vertexCosts.size());
        m_vertex_costs_enabled = true;
    }
    else
    {
        ROS_WARN("Received not as much vertex costs as vertices, ignoring the vertex costs!");
        return false;
    }

    enteringTriangleMeshWithVertexCosts(m_geometry, vertexCosts);

    return true;
}

bool TexturedMeshVisual::setMaterials(
    const vector<Material>& materials,
    const vector<TexCoords>& texCoords
)
{
    // check if there is a material index for each cluster
    if (materials.size() >= 0)
    {
        ROS_INFO("Received %lu materials.", materials.size());
        m_materials_enabled = true; // enable materials
    }
    else
    {
        ROS_WARN("Received zero materials, ignoring materials!");
        return false;
    }

    // texture coords
    // check if there are texture coords for each vertex
    if (texCoords.size() == m_geometry.vertices.size())
    {
        ROS_INFO("Received %lu texture coords.", texCoords.size());
        m_texture_coords_enabled = true; // enable texture coords
        m_textures_enabled = true; // enable textures
    }
    else if (texCoords.size() > 0)
    {
        ROS_WARN("Received not as much texture coords as vertices, ignoring texture coords!");
    }

    enteringTexturedTriangleMesh(m_geometry, materials, texCoords);

    return true;
}

bool TexturedMeshVisual::addTexture(Texture& texture, uint32_t textureIndex)
{
    uint32_t width = texture.width;
    uint32_t height = texture.height;
    uint32_t step = texture.channels;

    uint32_t dataSize = width * height * step;

    Ogre::PixelFormat pixelFormat = getOgrePixelFormatFromRosString(texture.pixelFormat);

    Ogre::Image image = Ogre::Image();
    image.loadDynamicImage(texture.data.data(), width, height, 1, pixelFormat, false);
    m_images.insert(m_images.begin() + textureIndex, image);

    if (m_textureMaterials.size() >= textureIndex + 1)
    {
        loadImageIntoTextureMaterial(textureIndex);
        return true;
    }
    else
    {
        ROS_WARN("Can't load image into texture material, material does not exist!");
        return false;
    }

}

Ogre::PixelFormat TexturedMeshVisual::getOgrePixelFormatFromRosString(std::string encoding)
{
    if (encoding == "rgba8")
    {
        return Ogre::PF_BYTE_RGBA;
    }
    else if (encoding == "rgb8")
    {
        return Ogre::PF_BYTE_RGB;
    }

    ROS_WARN("Unknown texture encoding! Using Ogre::PF_UNKNOWN");
    return Ogre::PF_UNKNOWN;
}

void TexturedMeshVisual::loadImageIntoTextureMaterial(size_t textureIndex)
{
    std::stringstream textureNameStream;
    textureNameStream << m_prefix << "_Texture" << textureIndex << "_" << m_postfix << "_" << m_random;

    Ogre::TexturePtr texturePtr = Ogre::TextureManager::getSingleton().createManual(
        textureNameStream.str(),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        m_images[textureIndex].getWidth(),
        m_images[textureIndex].getHeight(),
        0,
        m_images[textureIndex].getFormat()
    );

    texturePtr->loadImage(m_images[textureIndex]);

    Ogre::Pass* pass = m_textureMaterials[textureIndex]->getTechnique(0)->getPass(0);
    pass->removeAllTextureUnitStates();
    pass->createTextureUnitState()->addFrameTextureName(textureNameStream.str());
}

Ogre::ColourValue TexturedMeshVisual::calculateColorFromCost(float cost)
{
    Ogre::ColourValue color;

    // calculate a color that is green for 0, yellow for 0.5 and red for 1
    color.r = cost * 2;
    color.r = color.r > 1.0f ? 1.0f : color.r;
    color.g = (1.0f - cost) * 2;
    color.g = color.g > 1.0f ? 1.0f : color.g;
    color.b = 0.0f;
    color.a = 1.0;

    return color;
}

} // end namespace rviz_map_plugin
