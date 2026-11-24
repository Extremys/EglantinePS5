// SPDX-License-Identifier: GPL-3.0-or-later
// The Vulkan entry points MihawkGen's sources call by name, as function pointers (as volk gives
// them in MihawkGen's own build): the console's RADV is linked without its vk* prototypes, so
// Eden reaches the driver through its dispatch tables, and so does this. Force-included into the
// MihawkGen units (headless/vulkan.cmake); fg_load_vulkan fills the pointers from Eden's device.
#pragma once
#ifndef VK_NO_PROTOTYPES
#define VK_NO_PROTOTYPES
#endif
#include <vulkan/vulkan.h>

#define FG_VULKAN_FUNCTIONS(X)                                                                     \
    X(vkAllocateDescriptorSets) X(vkAllocateMemory) X(vkBindBufferMemory) X(vkBindImageMemory)    \
    X(vkCmdBindDescriptorSets) X(vkCmdBindPipeline) X(vkCmdClearColorImage) X(vkCmdDispatch)      \
    X(vkCmdPipelineBarrier) X(vkCmdPushConstants) X(vkCmdWriteTimestamp) X(vkCreateBuffer)        \
    X(vkCreateComputePipelines) X(vkCreateDescriptorPool) X(vkCreateDescriptorSetLayout)         \
    X(vkCreateImage) X(vkCreateImageView) X(vkCreatePipelineLayout) X(vkCreateSampler)            \
    X(vkCreateShaderModule) X(vkDestroyBuffer) X(vkDestroyDescriptorPool)                         \
    X(vkDestroyDescriptorSetLayout) X(vkDestroyImage) X(vkDestroyImageView) X(vkDestroyPipeline)  \
    X(vkDestroyPipelineLayout) X(vkDestroySampler) X(vkDestroyShaderModule) X(vkFreeMemory)       \
    X(vkGetBufferMemoryRequirements) X(vkGetImageMemoryRequirements) X(vkMapMemory)               \
    X(vkUnmapMemory) X(vkUpdateDescriptorSets)

#define FG_VULKAN_DECLARE(name) extern PFN_##name name;
FG_VULKAN_FUNCTIONS(FG_VULKAN_DECLARE)
extern PFN_vkGetPhysicalDeviceMemoryProperties vkGetPhysicalDeviceMemoryProperties;
#undef FG_VULKAN_DECLARE

/* Fills the pointers above: device functions through gdpa(device), the one instance function
 * through gipa(instance). False when one is missing (then MihawkGen must not be used). */
bool fg_load_vulkan(PFN_vkGetInstanceProcAddr gipa, VkInstance instance,
                    PFN_vkGetDeviceProcAddr gdpa, VkDevice device);
