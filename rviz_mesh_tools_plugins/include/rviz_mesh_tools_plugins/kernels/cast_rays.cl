//#pragma OPENCL EXTENSION cl_intel_printf : enable

#define EPSILON 0.0000001
#define PI 3.14159265

typedef struct {
    bool intersects;
    float intersection;
} intersection_result;

typedef struct {
    float3 vertex0;
    float3 vertex1;
    float3 vertex2;
} triangle_t;

/**
 * Credits go to: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
 */
intersection_result ray_intersects_triangle(
    float3 ray,
    float3 ray_origin,
    triangle_t triangle
)
{
    float3 edge1, edge2, h, s, q;
    float a, f, u, v;
    intersection_result out;

    edge1 = triangle.vertex1 - triangle.vertex0;
    edge2 = triangle.vertex2 - triangle.vertex0;
    h = cross(ray, edge2);
    a = dot(edge1, h);

    if (a > -EPSILON && a < EPSILON)
    {
        out.intersects = false;
        return out;
    }

    f = 1/a;
    s = ray_origin - triangle.vertex0;
    u = f * dot(s, h);

    if (u < 0.0 || u > 1.0)
    {
        out.intersects = false;
        return out;
    }

    q = cross(s, edge1);
    v = f * dot(ray, q);

    if (v < 0.0 || u + v > 1.0)
    {
        out.intersects = false;
        return out;
    }

    float t = f * dot(edge2, q);
    if (t > EPSILON)
    {
        out.intersects = true;
        out.intersection = t;
        return out;
    }

    out.intersects = false;
    return out;
}


__kernel void cast_rays(
    __global float* vertices,
    __global float* ray,
    __global float* result
)
{
    int id = get_global_id(0);

    float3 rayOrigin = (float3)(ray[0], ray[1], ray[2]);
    float3 rayDirection = (float3)(ray[3], ray[4], ray[5]);

    // initialize result memory
    result[id] = -1;

    triangle_t triangle;
    triangle.vertex0 = (float3)(vertices[id * 9 + 0], vertices[id * 9 + 1], vertices[id * 9 + 2]);
    triangle.vertex1 = (float3)(vertices[id * 9 + 3], vertices[id * 9 + 4], vertices[id * 9 + 5]);
    triangle.vertex2 = (float3)(vertices[id * 9 + 6], vertices[id * 9 + 7], vertices[id * 9 + 8]);

    intersection_result i_result = ray_intersects_triangle(rayDirection, rayOrigin, triangle);
    if (i_result.intersects)
    {
        result[id] = i_result.intersection;
    }
}

__kernel void cast_sphere(
    __global float* vertices,
    __global float* sphere,
    __global float* result,
    float dist
)
{
    int id = get_global_id(0);

    float3 center = (float3)(sphere[0], sphere[1], sphere[2]);

    // initialize result memory
    result[id] = -1;

    // store each input vertex in a float4 to simplify access
    float3 vertex0 = (float3)(vertices[id * 9 + 0], vertices[id * 9 + 1], vertices[id * 9 + 2]);
    float3 vertex1 = (float3)(vertices[id * 9 + 3], vertices[id * 9 + 4], vertices[id * 9 + 5]);
    float3 vertex2 = (float3)(vertices[id * 9 + 6], vertices[id * 9 + 7], vertices[id * 9 + 8]);

    // calculate the distance to the center point for each vertex
    float dist0 = distance(center, vertex0);
    float dist1 = distance(center, vertex1);
    float dist2 = distance(center, vertex2);

    // if one of the vertices is closer to the center than the threshold, the triangle is hit
    if (dist0 <= dist || dist1 <= dist || dist2 <= dist)
    {
        result[id] = 1;
    }

}

__kernel void cast_box(
    __global float* vertices,
    __global float4* box,
    __global float* result
)
{
    int id = get_global_id(0);

    // store each input vertex in a float4 to simplify access
    float4 vertex0 = (float4)(vertices[id * 9 + 0], vertices[id * 9 + 1], vertices[id * 9 + 2], 1.0);
    float4 vertex1 = (float4)(vertices[id * 9 + 3], vertices[id * 9 + 4], vertices[id * 9 + 5], 1.0);
    float4 vertex2 = (float4)(vertices[id * 9 + 6], vertices[id * 9 + 7], vertices[id * 9 + 8], 1.0);

    // check for each vertex if it lies within the volume spanned by the box planes
    bool vertexInVolume0 = true;
    bool vertexInVolume1 = true;
    bool vertexInVolume2 = true;
    for (int planeId = 0; planeId < 6; planeId++)
    {
        float4 plane = box[planeId];
        vertexInVolume0 = vertexInVolume0 && dot(plane, vertex0) > 0;
        vertexInVolume1 = vertexInVolume1 && dot(plane, vertex1) > 0;
        vertexInVolume2 = vertexInVolume2 && dot(plane, vertex2) > 0;
    }

    // if one of the vertices was in the volume, set this triangle's result to hit
    if (vertexInVolume0 || vertexInVolume1 || vertexInVolume2)
    {
        result[id] = 1;
    }
    else
    {
        result[id] = -1;
    }

}

