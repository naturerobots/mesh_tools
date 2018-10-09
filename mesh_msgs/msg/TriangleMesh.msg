## Definition of a triangle mesh

TriangleIndices[] triangles # list of triangles; the index values refer to positions in vertices (and vertex_normals, if given)
geometry_msgs/Point[] vertices # the actual vertices that make up the mesh
#optional:
geometry_msgs/Point[] vertex_normals
std_msgs/ColorRGBA[] vertex_colors
std_msgs/ColorRGBA[] triangle_colors
geometry_msgs/Point[] vertex_texture_coords
mesh_msgs/MeshMaterial[] face_materials
sensor_msgs/Image[] textures
mesh_msgs/MeshFaceCluster[] clusters
