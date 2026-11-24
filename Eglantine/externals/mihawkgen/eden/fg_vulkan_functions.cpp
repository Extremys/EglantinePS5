// SPDX-License-Identifier: GPL-3.0-or-later
#include "fg_vulkan_functions.h"

#define FG_VULKAN_DEFINE(name) PFN_##name name = nullptr;
FG_VULKAN_FUNCTIONS(FG_VULKAN_DEFINE)
PFN_vkGetPhysicalDeviceMemoryProperties vkGetPhysicalDeviceMemoryProperties = nullptr;
#undef FG_VULKAN_DEFINE

bool fg_load_vulkan(PFN_vkGetInstanceProcAddr gipa, VkInstance instance,
                    PFN_vkGetDeviceProcAddr gdpa, VkDevice device) {
    bool complete = true;
#define FG_VULKAN_LOAD(name)                                                                       \
    name = reinterpret_cast<PFN_##name>(gdpa(device, #name));                                      \
    complete = complete && name != nullptr;
    FG_VULKAN_FUNCTIONS(FG_VULKAN_LOAD)
#undef FG_VULKAN_LOAD
    vkGetPhysicalDeviceMemoryProperties = reinterpret_cast<PFN_vkGetPhysicalDeviceMemoryProperties>(
        gipa(instance, "vkGetPhysicalDeviceMemoryProperties"));
    return complete && vkGetPhysicalDeviceMemoryProperties != nullptr;
}
