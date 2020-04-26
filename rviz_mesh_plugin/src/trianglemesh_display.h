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
 *  trianglemesh_display.h
 *
 *
 *  authors:
 *
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Henning Deeken <hdeeken@uni-osnabrueck.de>
 *    Marcel Mrozinski
 *    Nils Oesting
 */

#ifndef TRIANGLEMESH_DISPLAY_H
#define TRIANGLEMESH_DISPLAY_H

#include <rviz/display.h>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>
#include <tf2_ros/message_filter.h>
#include <boost/circular_buffer.hpp>
#include "trianglemesh_visual.h"
#endif

namespace rviz
{

// Forward declaration
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;
class EnumProperty;

} // End namespace rviz


namespace rviz_mesh_plugin
{

// Forward declaration
class TriangleMeshVisual;

/**
 * @brief TriangleMeshDisplay
 * @brief Class to show options in rviz window.
 */
class TriangleMeshDisplay : public rviz::Display
{
Q_OBJECT
public:

    /// Counter for the number of displays
    static size_t displayCounter;

    /**
     * @brief Constructor.
     */
    TriangleMeshDisplay();

    /**
     * @brief Destructor.
     */
    ~TriangleMeshDisplay();

protected:

    /**
     * @brief Initialises all nessessary things to get started.
     */
    void onInitialize();

    /**
     * @brief Clears whole stored data.
     */
    void reset();

    /**
     * @brief Set the topics to subscribe.
     */
    void subscribe();

    /**
     * @brief Unsubscribes all topics.
     */
    void unsubscribe();

    /**
     * @brief Calls subscribe() if display is enabled
     */
    void onEnable();

    /**
     * @brief Calls unsubscribe() and reset() if display is disabled.
     */
    void onDisable();

    /**
     * @brief Sets the fixed frame.
     */
    void fixedFrameChanged();

    /**
     * @brief Tests if messages are valid, calls processMessage().
     * @param meshMsg Message containing geometry information
     */
    void incomingMessage(const mesh_msgs::TriangleMeshStamped::ConstPtr& meshMsg);


private Q_SLOTS:

    /**
     * @brief Updates material for each mesh displayed by trianglemesh_visual.
     */
    void updateMesh();

    /**
     * @brief Sets capacity of trianglemesh_visual.
     */
    void updateMeshBufferSize();

    /**
     * @brief Updates the subscribed topic.
     */
    void updateTopic();

    /**
     * @brief Updates the topic synchronizer
     */
    void updateSynchronizer();

private:

    /**
     * @brief Sets data for trianglemesh_visual and updates the mesh.
     * @param meshMsg Message containing geometry information
     */
    void processMessage(const mesh_msgs::TriangleMeshStamped::ConstPtr& meshMsg);

    /// Subscriber for meshMsg
    message_filters::Subscriber<mesh_msgs::TriangleMeshStamped> m_meshSubscriber;

    /// Messagefilter for meshMsg
    tf2_ros::MessageFilter<mesh_msgs::TriangleMeshStamped>* m_tfMeshFilter;

    /// Synchronizer for meshMsgs
    message_filters::Cache<mesh_msgs::TriangleMeshStamped>* m_synchronizer;

    /// Counter for the received messages
    uint32_t m_messagesReceived;

    /// Shared pointer to store the created objects of trianglemesh_visual
    boost::circular_buffer<boost::shared_ptr<TriangleMeshVisual> > m_meshVisuals;

    /// Counter for the meshes
    size_t m_meshCounter;

    /// DisplayID
    size_t m_displayID;

    /// Property to handle topic for meshMsg
    rviz::RosTopicProperty* m_meshTopic;

    /// Property to set meshBufferSize
    rviz::IntProperty* m_meshBufferSize;

    /// Property to set wireframe color
    rviz::ColorProperty* m_wireframeColor;

    /// Property to set wireframe transparency
    rviz::FloatProperty* m_wireframeAlpha;

    /// Property to set faces color
    rviz::ColorProperty* m_facesColor;

    /// Property to set faces transparency
    rviz::FloatProperty* m_facesAlpha;

    /// Property to use the vertex colors
    rviz::BoolProperty* m_facesVertexColors;

    /// Property to use the triangle colors
    rviz::BoolProperty* m_facesTriangleColors;

    /// Property to set the size of the normals
    rviz::FloatProperty* m_scalingFactor;

    /// Property to set the color of the normals
    rviz::ColorProperty* m_normalsColor;

    /// Property to set the transparency of the normals
    rviz::FloatProperty* m_normalsAlpha;

    /// Property to select the display type
    rviz::EnumProperty* m_displayType;

    /// Property to select the wireframe
    rviz::BoolProperty* m_showWireframe;

    /// Property to select the normals
    rviz::BoolProperty* m_showNormals;

};
} // End namespace rviz_mesh_plugin

#endif
