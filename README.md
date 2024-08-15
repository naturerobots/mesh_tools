This is the active ROS2 branch of this repository, which targets [ROS2 humble](https://docs.ros.org/en/humble/index.html).
Our CI runs on Ubuntu Jammy (22.04), but other distros might work as well.

If your are looking for the old ROS1 version, checkout the [noetic branch](https://github.com/naturerobots/mesh_tools/tree/noetic).

# Mesh Tools

We introduce a set of tools to make 3D environment mesh representations
accessible and manageable in ROS. We provide RViz tools to visualize and 
annotate huge meshes in combination with generated textures and different 
cost layers, which are based on the geometric analyses of the environment, 
or which represent different sensor readings, e.g. RGB image or even 
hyper-spectral image textures.
 
## Introduction

Over the last years, the Knowledge Based Systems Group has developed
an extensive set of tools to automatically generate triangle meshes
from 3D laser scans. These tools are implemented in the
freely available Las Vegas Surface Reconstruction Toolkit
(LVR) and have been successfully applied in different
robotic contexts.

With terrestrial and mobile laser scanning it is possible to create 3D
point clouds of large environments in short time. However, the amount
of collected data is too large to be processed on a mobile robot. To
make this data available on mobile robots, we suggest to compute 3D
triangle meshes from point cloud data as corresponding map
representations. Such maps have several advantages
over raw point cloud data and voxel representations. They overcome the
discretization of voxel maps and deliver topologically connected
surface representations, which ease segmentation, annotation and enable intuitive visualization.

However, to make the maps accessible in ROS, corresponding message
definitions and tools for storing, handling and editing such maps are
required.

### Citation

Please reference the following papers when using mesh_tools in your scientific work.

```latex
@inproceedings{putz2019ecmr,
  title={{Tools for visualizing, annotating and storing triangle meshes in ROS and RViz}},
  author={P{\"u}tz, Sebastian and Wiemann, Thomas and Hertzberg, Joachim},
  booktitle={2019 European Conference on Mobile Robots (ECMR)},
  pages={1--6},
  year={2019},
  organization={IEEE}
}
```

```latex
@article{putz2021ras,
  title = {{The Mesh Tools Package – Introducing Annotated 3D Triangle Maps in ROS}},
  journal = {Robotics and Autonomous Systems},
  volume = {138},
  pages = {103688},
  year = {2021},
  doi = {https://doi.org/10.1016/j.robot.2020.103688},
  author = {P{\"u}tz, Sebastian and Wiemann, Thomas and Hertzberg, Joachim},
}
```

## Message Definitions

We divided the mesh structure in its geometry, textures and cost layers. These different components are then associated using a UUID. This structure enables passing the geometry at first to RViz and other nodes. Theses nodes may analyze the geometry, e.g the roughness or height difference and compute corresponding mesh cost layers which are published as individual message with a corresponding mesh UUID. Thereby other nodes can associate these layers with a certain mesh object. In RViz a certain layer can then be selected by the user and visualized coloring the corresponding costs or displaying the textures. In the following gives an overview on most important mesh messages:

+ **MeshGeometry(Stamped)**:  Geometric structure of a triangle mesh using a buffer vertices, normals and faces
+ **MeshVertexColors(Stamped)**:  Vertex color information of a corresponding mesh
+ **MeshVertexCosts(Stamped)**:  Vertex cost information of a corresponding mesh coloring the mesh using a rainbow color scheme.
+ **ClusterLabel**:  Grouping a set of triangles / faces of a corresponding mesh using the face indices, optional cluster label
+ **Texture**: Texture using an Image and an ID
+ **Material**: Color and optional texture ID
+ **VertexTexCoords**: Texture coordinate
+ **MeshMaterials(Stamped)**: Combining the materials with texture coordinates and clusters of a corresponding mesh
+ **Feature**: A feature at a specific location and its feature descriptor
+ **MeshFeatures**: List of features for a corresponding mesh

## RViz Plugins

We developed several RViz plugins to display 3D meshes together
with different information layers (RGB textures, semantic labels,
trafficability, etc.). It is possible to render the received meshes in
different modes (lighting, materials and wireframe). The supported
modes are selectable based on availability in the tree view. For path
planning, we also implemented an interactive tool to set navigation
goal poses on the mesh surface. To generate semantic maps, we implemented an
interactive tool that can be used to select connected clusters of
triangles and assign semantic labels to them. Some screenshots and videos demonstrate these plugins (see Supplementary Material).

### RViz Mesh Plugin
The *TexturedMesh* displays a mesh with optional a fixed color, textures, vertex colors or vertex costs. Additionally, it can display the wire-frame as well as the vertex normals of the mesh. The *MeshGoal Tool* provides the possibility to select a *geometry_msgs/PoseStamped* on the surface of the mesh. 

### RViz Map Plugin
The *Mesh Display* displays labeled faces. The *ClusterLabel Display* displays labeled clusters. The *ClusterLabel Tool* enables the labeling of certain faces using different selection and de-selection methods as shown in a video (see Supplementary Material. The *ClusterLabel Panel* allows to manage your clusters and label names as well as the corresponding colors.

# Videos

## Mesh Cost Layer
This Video shows different cost layers which are used for path planning in outdoor terrain. It shows one partly reconstructed scan in the botanical garden of Osnabrück

[![Mesh Cost Layer](http://img.youtube.com/vi/Ac1YLn88QGk/0.jpg)](http://www.youtube.com/watch?v=Ac1YLn88QGk)

## Mesh Textures
This Video shows a textured dataset of the botanical garden of Osnabrück including footpaths

[![Mesh Textures](http://img.youtube.com/vi/CF-WdXwx_zo/0.jpg)](http://www.youtube.com/watch?v=CF-WdXwx_zo)

## Label Tool
This video shows the labeling tool, which is used to cluster faces to an object or a union, (in the example a door) using a mesh inside a building

[![Label Tool](http://img.youtube.com/vi/3IV2yo0D_CU/0.jpg)](http://www.youtube.com/watch?v=3IV2yo0D_CU)

## Build Status

| ROS Distro  | GitHub CI | Develop | Documentation | Source Deb | Binary Deb |
|-------------|-----------|---------|---------------|------------|------------|
| **Humble**  | [![Humble CI](https://github.com/naturerobots/mesh_tools/actions/workflows/humble.yaml/badge.svg)](https://github.com/naturerobots/mesh_tools/actions/workflows/humble.yaml) | N/A | N/A | N/A | N/A |
| **Noetic**  | [![Noetic CI](https://github.com/naturerobots/mesh_tools/actions/workflows/noetic.yml/badge.svg)](https://github.com/naturerobots/mesh_tools/actions/workflows/noetic.yml) | [![Build Dev Status](http://build.ros.org/buildStatus/icon?job=Ndev__mesh_tools__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__mesh_tools__ubuntu_focal_amd64) | [![Build Doc Status](http://build.ros.org/buildStatus/icon?job=Ndoc__mesh_tools__ubuntu_focal_amd64)](http://build.ros.org/job/Ndoc__mesh_tools__ubuntu_focal_amd64) | [![Build Src Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mesh_tools__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mesh_tools__ubuntu_focal__source) | [![Build Bin Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mesh_tools__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mesh_tools__ubuntu_focal_amd64__binary) | 