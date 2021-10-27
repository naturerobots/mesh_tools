^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_map_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2021-10-27)
------------------
* fixed GetUUIDs service call in MeshDisplay
* added option to visualize vertex costs using the rviz Map Visuslization
* removed unused messages
* added support for multiple meshes to be visualized at the same time
* added cleaning of generalmesh before adding new data
* removed old plugin and displays
* fixed thin wireframe
* fixed normals shading
* improved normals performance
* fixed rainbow-redgreen cost color type change
* fixed duplicate vertex cost types
* fixed map3d and fixed loading Vertex colors when updating the VertexColors topic
* MeshDisplay: removed visual access from topic and service callbacks
* added option to stop MeshDisplay listening to MeshMsgs
* MeshDisplay now woring with MeshMap
* added properties to MeshDisplay
* added Map3D file dialog property
* added textured mesh display for mesh msgs visualization to rviz_map_plugin
* combined mesh-plugin and map-plugin texture mesh visuals
* fixed mpi error
* resolved catkin lint problems

1.0.1 (2020-11-11)
------------------

1.0.0 (2020-04-26)
------------------
* release version 1.0.0
