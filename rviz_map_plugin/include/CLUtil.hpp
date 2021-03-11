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
 *  CLUtil.hpp
 *
 *
 *  authors:
 *
 *    Kristin Schmidt <krschmidt@uni-osnabrueck.de>
 */

#ifndef CL_UTIL_HPP
#define CL_UTIL_HPP

#include <CL/cl2.hpp>

namespace rviz_map_plugin
{
/**
 * @class CLUtil
 * @brief Utility class for getting human-readable messages from OpenCL error codes
 */
class CLUtil
{
public:
  /**
   * @brief Returns the error string to a given OpenCL error code
   * @param error The error code
   * @return The name of the error
   */
  static inline const char* getErrorString(cl_int error)
  {
    switch (error)
    {
      // run-time and JIT compiler errors
      case 0:
        return "CL_SUCCESS";
      case -1:
        return "CL_DEVICE_NOT_FOUND";
      case -2:
        return "CL_DEVICE_NOT_AVAILABLE";
      case -3:
        return "CL_COMPILER_NOT_AVAILABLE";
      case -4:
        return "CL_MEM_OBJECT_ALLOCATION_FAILURE";
      case -5:
        return "CL_OUT_OF_RESOURCES";
      case -6:
        return "CL_OUT_OF_HOST_MEMORY";
      case -7:
        return "CL_PROFILING_INFO_NOT_AVAILABLE";
      case -8:
        return "CL_MEM_COPY_OVERLAP";
      case -9:
        return "CL_IMAGE_FORMAT_MISMATCH";
      case -10:
        return "CL_IMAGE_FORMAT_NOT_SUPPORTED";
      case -11:
        return "CL_BUILD_PROGRAM_FAILURE";
      case -12:
        return "CL_MAP_FAILURE";
      case -13:
        return "CL_MISALIGNED_SUB_BUFFER_OFFSET";
      case -14:
        return "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST";
      case -15:
        return "CL_COMPILE_PROGRAM_FAILURE";
      case -16:
        return "CL_LINKER_NOT_AVAILABLE";
      case -17:
        return "CL_LINK_PROGRAM_FAILURE";
      case -18:
        return "CL_DEVICE_PARTITION_FAILED";
      case -19:
        return "CL_KERNEL_ARG_INFO_NOT_AVAILABLE";

      // compile-time errors
      case -30:
        return "CL_INVALID_VALUE";
      case -31:
        return "CL_INVALID_DEVICE_TYPE";
      case -32:
        return "CL_INVALID_PLATFORM";
      case -33:
        return "CL_INVALID_DEVICE";
      case -34:
        return "CL_INVALID_CONTEXT";
      case -35:
        return "CL_INVALID_QUEUE_PROPERTIES";
      case -36:
        return "CL_INVALID_COMMAND_QUEUE";
      case -37:
        return "CL_INVALID_HOST_PTR";
      case -38:
        return "CL_INVALID_MEM_OBJECT";
      case -39:
        return "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR";
      case -40:
        return "CL_INVALID_IMAGE_SIZE";
      case -41:
        return "CL_INVALID_SAMPLER";
      case -42:
        return "CL_INVALID_BINARY";
      case -43:
        return "CL_INVALID_BUILD_OPTIONS";
      case -44:
        return "CL_INVALID_PROGRAM";
      case -45:
        return "CL_INVALID_PROGRAM_EXECUTABLE";
      case -46:
        return "CL_INVALID_KERNEL_NAME";
      case -47:
        return "CL_INVALID_KERNEL_DEFINITION";
      case -48:
        return "CL_INVALID_KERNEL";
      case -49:
        return "CL_INVALID_ARG_INDEX";
      case -50:
        return "CL_INVALID_ARG_VALUE";
      case -51:
        return "CL_INVALID_ARG_SIZE";
      case -52:
        return "CL_INVALID_KERNEL_ARGS";
      case -53:
        return "CL_INVALID_WORK_DIMENSION";
      case -54:
        return "CL_INVALID_WORK_GROUP_SIZE";
      case -55:
        return "CL_INVALID_WORK_ITEM_SIZE";
      case -56:
        return "CL_INVALID_GLOBAL_OFFSET";
      case -57:
        return "CL_INVALID_EVENT_WAIT_LIST";
      case -58:
        return "CL_INVALID_EVENT";
      case -59:
        return "CL_INVALID_OPERATION";
      case -60:
        return "CL_INVALID_GL_OBJECT";
      case -61:
        return "CL_INVALID_BUFFER_SIZE";
      case -62:
        return "CL_INVALID_MIP_LEVEL";
      case -63:
        return "CL_INVALID_GLOBAL_WORK_SIZE";
      case -64:
        return "CL_INVALID_PROPERTY";
      case -65:
        return "CL_INVALID_IMAGE_DESCRIPTOR";
      case -66:
        return "CL_INVALID_COMPILER_OPTIONS";
      case -67:
        return "CL_INVALID_LINKER_OPTIONS";
      case -68:
        return "CL_INVALID_DEVICE_PARTITION_COUNT";

      // extension errors
      case -1000:
        return "CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR";
      case -1001:
        return "CL_PLATFORM_NOT_FOUND_KHR";
      case -1002:
        return "CL_INVALID_D3D10_DEVICE_KHR";
      case -1003:
        return "CL_INVALID_D3D10_RESOURCE_KHR";
      case -1004:
        return "CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR";
      case -1005:
        return "CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR";
      default:
        return "Unknown OpenCL error";
    }
  };

  /**
   * @brief Returns a description to a given OpenCL error code
   * @param error The error code
   * @return A description of the error
   */
  static inline const char* getErrorDescription(cl_int error)
  {
    switch (error)
    {
      // run-time and JIT compiler errors
      case 0:
        return "Indicates that the function executed successfully.";
      case -1:
        return "Returned by clGetDeviceIDs and clCreateContextFromType if no OpenCL devices that match "
               "the specified devices were found.";
      case -2:
        return "Returned by clCreateContext and clCreateContextFromType if the specified device is not "
               "currently available.";
      case -3:
        return "Returned by clBuildProgram if the parameter program is created with "
               "clCreateProgramWithSource and a compiler is not available. For example "
               "CL_DEVICE_COMPILER_AVAILABLE is set to CL_FALSE.";
      case -4:
        return "Returned by the functions listed below if there is a failure to allocate memory for data "
               "store associated with image or buffer objects specified as arguments to kernel. The exact "
               "condition that generates this error depends on the calling function. Refer to the "
               "function for more information. \n"
               "Returned by the following functions: clCreateBuffer, clEnqueueReadBuffer, "
               "clEnqueueWriteBuffer, clEnqueueCopyBuffer, clCreateImage2D, clCreateImage3D, "
               "clEnqueueReadImage, clEnqueueWriteImage, clEnqueueCopyImage, clEnqueueCopyImageToBuffer, "
               "clEnqueueCopyBufferToImage, clEnqueueMapBuffer, clEnqueueMapImage, "
               "clEnqueueNDRangeKernel, clEnqueueTask, and clEnqueueNativeKernel ";
      case -5:
        return "Returned by clEnqueueNDRangeKernel, clEnqueueTask, and clEnqueueNativeKernel in the event "
               "of a failure to queue the execution instance of kernel on the command-queue because of "
               "insufficient resources needed to execute the kernel.";
      case -6:
        return "Returned by the functions listed below in the event of a failure to allocate resources "
               "required by the OpenCL implementation on the host. The exact condition that generates "
               "this error depends on the calling function. Refer to the function for more information. ";
      case -7:
        return "Returned by clGetEventProfilingInfo if the CL_QUEUE_PROFILING_ENABLE flag is not set for "
               "the command-queue and the profiling information is currently not available (because the "
               "command identified by event has not completed).";
      case -8:
        return "Returned by clEnqueueCopyBuffer and clEnqueueCopyImage if the source and destination "
               "images are the same image (or the source and destination buffers are the same buffer), "
               "and the source and destination regions overlap.";
      case -9:
        return "Returned by clEnqueueCopyImage if the specified source and destination images are not "
               "valid image objects.";
      case -10:
        return "Returned by clCreateImage2D and clCreateImage3D if the specified image format is not "
               "supported.";
      case -11:
        return "Returned by clBuildProgram if there is a failure to build the program executable. This "
               "error will be returned if clBuildProgram does not return until the build has completed.";
      case -12:
        return "Returned by clEnqueueMapBuffer and clEnqueueMapImage if there is a failure to map the "
               "requested region into the host address space. This error cannot occur for buffer objects "
               "created with CL_MEM_USE_HOST_PTR or CL_MEM_ALLOC_HOST_PTR.";
      case -13:
        return "CL_MISALIGNED_SUB_BUFFER_OFFSET";
      case -14:
        return "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST";
      case -15:
        return "CL_COMPILE_PROGRAM_FAILURE";
      case -16:
        return "CL_LINKER_NOT_AVAILABLE";
      case -17:
        return "CL_LINK_PROGRAM_FAILURE";
      case -18:
        return "CL_DEVICE_PARTITION_FAILED";
      case -19:
        return "CL_KERNEL_ARG_INFO_NOT_AVAILABLE";

      // compile-time errors
      case -30:
        return "Returned by the functions listed below if a parameter is not an expected value. The "
               "exact condition that generates this error depends on the calling function. Refer to the "
               "function for more information.";
      case -31:
        return "Returned by clGetDeviceIDs and clCreateContextFromType if device type specified is not "
               "valid.";
      case -32:
        return "Returned by clGetPlatformInfo and clGetDeviceIDs if the specified platform is not a "
               "valid platform.\n"
               "Returned by clCreateContext and clCreateContextFromType if properties is NULL and no "
               "platform could be selected, or if platform value specified in properties is not a valid "
               "platform.";
      case -33:
        return "Returned by the functions listed below if the device or devices specified are not valid. "
               "The exact condition that generates this error depends on the calling function. Refer to "
               "the function for more information.";
      case -34:
        return "Returned by the functions listed below if the specified context is not a valid OpenCL "
               "context, or the context associated with certain parameters are not the same. The exact "
               "condition that generates this error depends on the calling function. Refer to the "
               "function for more information.";
      case -35:
        return "Returned by clCreateCommandQueue and clSetCommandQueueProperty if specified properties "
               "are valid but are not supported by the device.";
      case -36:
        return "Returned by the functions listed below if the specified command-queue is not a valid "
               "command-queue. The exact condition that generates this error depends on the calling "
               "function. Refer to the function for more information.";
      case -37:
        return "Returned by the functions listed below if host_ptr is NULL and CL_MEM_USE_HOST_PTR or "
               "CL_MEM_COPY_HOST_PTR are set in flags or if host_ptr is not NULL but "
               "CL_MEM_COPY_HOST_PTR or CL_MEM_USE_HOST_PTR are not set in flags. The exact condition "
               "that generates this error depends on the calling function. Refer to the function for "
               "more information.\n"
               "Returned by the functions clCreateBuffer, clCreateImage2D, and clCreateImage3D. ";
      case -38:
        return "Returned by the functions listed below if a parameter is not a valid memory, image, or "
               "buffer object. The exact condition that generates this error depends on the calling "
               "function. Refer to the function for more information.";
      case -39:
        return "Returned by clCreateImage2D and clCreateImage3D if the image format specified is not "
               "valid or is NULL.\n"
               "Returned byclCreateFromGLTexture2D and clCreateFromGLTexture3D\n"
               "Returned by clCreateFromGLRenderbuffer if the OpenGL renderbuffer internal format does "
               "not map to a supported OpenCL image format. ";
      case -40:
        return "Returned by clCreateImage2D if the specified image width or height are 0 or if they "
               "exceed values specified in CL_DEVICE_IMAGE2D_MAX_WIDTH or CL_DEVICE_IMAGE2D_MAX_HEIGHT "
               "respectively for all devices in context, or if the specified image row pitch does not "
               "follow rules described for clCreateImage2D.\n"
               "Returned by clCreateImage3D if the specified image width or height are 0 or if the image "
               "depth is <= 1, or if they exceed values specified in CL_DEVICE_IMAGE3D_MAX_WIDTH, "
               "CL_DEVICE_IMAGE3D_MAX_HEIGHT or CL_DEVICE_IMAGE3D_MAX_DEPTH respectively for all devices "
               "in context, or if the image row pitch and image slice pitch do not follow rules "
               "described for clCreateImage3D. ";
      case -41:
        return "Returned by clRetainSampler, clReleaseSampler, and clGetSamplerInfo if the specified "
               "sampler is not a valid sampler object.\n"
               "Returned by clSetKernelArg for an argument declared to be of type sampler_t when the "
               "specified arg_value is not a valid sampler object. ";
      case -42:
        return "Returned by clBuildProgram and clCreateProgramWithBinary if the program binary is not a "
               "valid binary for the specified device.";
      case -43:
        return "Returned by clBuildProgram if the specified build options are invalid.";
      case -44:
        return "Returned by clCreateKernel if there is no successfully built executable for program, and "
               "returned by clCreateKernelsInProgram if there is no device in program.\n"
               "Returned by clEnqueueNDRangeKernel and clEnqueueTask if there is no successfully built "
               "program executable available for device associated with command_queue. ";
      case -45:
        return "Returned by clCreateKernel if there is no successfully built executable for program, and "
               "returned by clCreateKernelsInProgram if there is no device in program.\n"
               "Returned by clEnqueueNDRangeKernel and clEnqueueTask if there is no successfully built "
               "program executable available for device associated with command_queue. ";
      case -46:
        return "Returned by clCreateKernel if the specified kernel name is not found in program.";
      case -47:
        return "Returned by clCreateKernel if the function definition for __kernel function given by "
               "kernel_name such as the number of arguments, the argument types are not the same for all "
               "devices for which the program executable has been built.";
      case -48:
        return "Returned by the functions listed below if the specified kernel is not a valid kernel "
               "object. The exact condition that generates this error depends on the calling function. "
               "Refer to the function for more information.";
      case -49:
        return "Returned by clSetKernelArg if an invalid argument index is specified.";
      case -50:
        return "Returned by clSetKernelArg if the argument value specified is NULL for an argument that "
               "is not declared with the __local qualifier or vice-versa.";
      case -51:
        return "Returned by clSetKernelArg if argument size specified (arg_size) does not match the size "
               "of the data type for an argument that is not a memory object, or if the argument is a "
               "memory object and arg_size != sizeof(cl_mem) or if arg_size is zero and the argument is "
               "declared with the __local qualifier or if the argument is a sampler and arg_size != "
               "sizeof(cl_sampler).";
      case -52:
        return "Returned by clEnqueueNDRangeKernel and clEnqueueTask if the kernel argument values have "
               "not been specified.";
      case -53:
        return "Returned by clEnqueueNDRangeKernel if work_dim is not a valid value.";
      case -54:
        return "Returned by clEnqueueNDRangeKernel and clEnqueueTask if local_work_size is specified and "
               "number of workitems specified by global_work_size is not evenly divisible by size of "
               "work-group given by local_work_size or does not match the work-group size specified for "
               "kernel using the __attribute__((reqd_work_group_size(X, Y, Z))) qualifier in program "
               "source.";
      case -55:
        return "Returned by clEnqueueNDRangeKernel if the number of work-items specified in any of "
               "local_work_size... [0]... local_work_size[work_dim - 1] is greater than the "
               "corresponding values specified by CL_DEVICE_MAX_WORK_ITEM_SIZES[0],... "
               "CL_DEVICE_MAX_WORK_ITEM_SIZES[work_dim -1].";
      case -56:
        return "Returned by clEnqueueNDRangeKernel if global_work_offset is not NULL.";
      case -57:
        return "Returned by the functions listed below if event_wait_list is NULL and "
               "num_events_in_wait_list > 0, or event_wait_list_list is not NULL and "
               "num_events_in_wait_list is 0, or specified event objects are not valid events. The exact "
               "condition that generates this error depends on the calling function. Refer to the "
               "function for more information.";
      case -58:
        return "Returned by the functions listed below if the event objects specified are not valid. The "
               "exact condition that generates this error depends on the calling function. Refer to the "
               "function for more information.";
      case -59:
        return "Returned by clCreateImage2D, clCreateImage3D, and clCreateSampler if there are no "
               "devices in context that support images.\n"
               "Returned by clBuildProgram if the build of a program executable for any of the devices "
               "specified by a previous call to clBuildProgram for program has not completed, or if "
               "there are kernel objects attached to program.\n"
               "Returned by clEnqueueNativeKernel if the specified device cannot execute the native "
               "kernel.\n"
               "Returned by clCreateFromGLTexture2D if the miplevel is less than 0.\n"
               "Returned by clCreateFromGLTexture3D if 3D images are not supported by the OpenCL "
               "embedded profile. ";
      case -60:
        return "Returned by clCreateFromGLBuffer if bufobj is not a GL buffer object or is a GL buffer "
               "object but does not have an existing data store.\n"
               "Returned by clCreateFromGLRenderbuffer if renderbuffer is not a GL renderbuffer object "
               "or if the width or height of renderbuffer is zero.\n"
               "Returned by clCreateFromGLTexture2D and clCreateFromGLTexture3D if texture is not a GL "
               "texture object whose type matches texture_target, if the specified miplevel of texture "
               "is not defined, or if the width, height or depth of the specified miplevel is zero.\n"
               "Returned by clGetGLObjectInfo and clGetGLTextureInfo if there is no GL object or texture "
               "associated with memobj.\n"
               "Returned by clEnqueueAcquireGLObjects and clEnqueueReleaseGLObjects if memory objects "
               "in mem_objects have not been created from a GL object(s). The exact condition that "
               "generates this error depends on the calling function. ";
      case -61:
        return "Returned by clCreateBuffer if the value of the parameter size is 0 or is greater than "
               "CL_DEVICE_MAX_MEM_ALLOC_SIZE for all devices specified in the parameter context.";
      case -62:
        return "CL_INVALID_MIP_LEVEL";
      case -63:
        return "CL_INVALID_GLOBAL_WORK_SIZE";
      case -64:
        return "CL_INVALID_PROPERTY";
      case -65:
        return "CL_INVALID_IMAGE_DESCRIPTOR";
      case -66:
        return "CL_INVALID_COMPILER_OPTIONS";
      case -67:
        return "CL_INVALID_LINKER_OPTIONS";
      case -68:
        return "CL_INVALID_DEVICE_PARTITION_COUNT";

      // extension errors
      case -1000:
        return "CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR";
      case -1001:
        return "CL_PLATFORM_NOT_FOUND_KHR";
      case -1002:
        return "CL_INVALID_D3D10_DEVICE_KHR";
      case -1003:
        return "CL_INVALID_D3D10_RESOURCE_KHR";
      case -1004:
        return "CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR";
      case -1005:
        return "CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR";
      default:
        return "Unknown OpenCL error";
    }
  };
};

}  // end namespace rviz_map_plugin

#endif