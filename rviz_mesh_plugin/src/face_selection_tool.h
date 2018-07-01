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
 *  face_selection_tool.h
 *
 *	authors:  Henning Deeken <hdeeken@uos.de>
 *            Sebastian Pütz <spuetz@uos.de>
 *            Tristan Igelbrink <tigelbri@uos.de>
 *            Johannes Heitmann <joheitma@uos.de>
 *            Marcel Mrozinski  <mmronzs@uos.de
 *
 */

#ifndef FACE_SELECTION_TOOL_H
#define FACE_SELECTION_TOOL_H

#include <vector>
#include <map>
#include <boost/lexical_cast.hpp>

#include <QMessageBox>
#include <QApplication>
#include <QIcon>

#include <ros/console.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/display.h>

#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/display_group.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <std_msgs/Int32.h>

#ifndef Q_MOC_RUN
#include <rviz/mesh_loader.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreStringConverter.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneQuery.h>

#endif

namespace rviz{
  class RosTopicProperty;
}

/**
 *
 *@class FaceSelectionTool
 *
 *@brief Implements a rviz tool for marking single faces in a mesh
 *
 * with this rviz tool the user can mark single faces in a displayed
 * OGRE mesh. The marking can be done by left click or with a selection
 * box
 *
 */

// OGRE stuff
namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz_mesh_plugin
{
class FaceSelectionTool: public rviz::Tool
{
Q_OBJECT
public:

  FaceSelectionTool();
  ~FaceSelectionTool();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);
  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  static const float BOX_SIZE_TOLERANCE;
  static const size_t MAXIMUM_PICKED_FACES;

  void clearSelection();

  bool areFacesSelected();
  void getSelectedFaces(size_t goalSection, std::string regionLabel, mesh_msgs::TriangleMesh &meshMsg);


private Q_SLOTS:
    void updateTopic();

private:

    void initNode();
    void initOgre();

    void updateSelectionMesh();

    void meshCb(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh);

    void setTransform(const mesh_msgs::TriangleMeshStamped &mesh);
    void setReferenceMesh( mesh_msgs::TriangleMesh mesh);
    void getSegmentMesh( mesh_msgs::TriangleMesh& mesh);

    void selectSingleFace(rviz::ViewportMouseEvent& event);
    void deselectSingleFace(rviz::ViewportMouseEvent& event);

    bool singleRayQuery(rviz::ViewportMouseEvent& event, int num_results, Ogre::Ray& ray);

    void getIdentityOfSingleFace(Ogre::ManualObject* mesh,
                                 Ogre::Ray &ray,
                                 size_t &goalSection,
                                 size_t &goalIndex,
                                 Ogre::Real& dist);

    void getRawManualObjectData(Ogre::ManualObject *mesh,
                                size_t sectionNumber,
                                size_t &vertexCount,
                                Ogre::Vector3* &vertices,
                                size_t &indexCount,
                                unsigned long* &indices);

    Ogre::SceneManager* scene_manager;
    Ogre::ManualObject* reference_mesh;
    Ogre::MaterialPtr reference_mesh_material;
    Ogre::ManualObject* segment_mesh;
    Ogre::MaterialPtr segment_mesh_material;
    Ogre::SceneNode* scene_node;

    rviz::RosTopicProperty* mesh_topic;

    std::map<size_t, std::vector<size_t> > m_goalFaces;

    bool m_singleSelect;
    bool m_singleDeselect;

    ros::NodeHandle n;
    ros::Subscriber mesh_sub;
    ros::Publisher mesh_pub;
    ros::Publisher id_pub;
    ros::Publisher goal_pub;
    int num_results;
    bool has_mesh;

    int reference_color_r = 0;
    int reference_color_g = 155;
    int reference_color_b = 155;
    float reference_color_a = 0.5;
  
    int segment_color_r = 0;
    int segment_color_g = 0;
    int segment_color_b = 255;
    float segment_color_a = 0.75;

};
}
#endif
