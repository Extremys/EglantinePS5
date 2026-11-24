// SPDX-License-Identifier: GPL-3.0-or-later

#include "video_core/renderer_vulkan/present/mihawk_frame_gen.h"

#include <cstddef>
#include <string>

#include "common/logging.h"
#include "video_core/renderer_vulkan/vk_scheduler.h"
#include "video_core/vulkan_common/vulkan_device.h"

// MihawkGen's interface (Eglantine/externals/mihawkgen/include/framegen.h) includes the Vulkan
// headers itself; Eden's are already in, without prototypes, which is what it needs here.
#include "framegen.h"

bool fg_load_vulkan(PFN_vkGetInstanceProcAddr gipa, VkInstance instance,
                    PFN_vkGetDeviceProcAddr gdpa, VkDevice device);
extern "C" const unsigned char* const fg_network_blob;
extern "C" const std::size_t fg_network_blob_size;

namespace Vulkan {
namespace {

VkImageMemoryBarrier Barrier(VkImage image, VkAccessFlags src_access, VkAccessFlags dst_access,
                             VkImageLayout old_layout, VkImageLayout new_layout) {
    return VkImageMemoryBarrier{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .pNext = nullptr,
        .srcAccessMask = src_access,
        .dstAccessMask = dst_access,
        .oldLayout = old_layout,
        .newLayout = new_layout,
        .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .image = image,
        .subresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
    };
}

} // Anonymous namespace

MihawkFrameGen::MihawkFrameGen(const Device& device, MemoryAllocator& memory_allocator,
                               VkExtent2D extent_, VkFormat format)
    : extent{extent_} {
    const auto& dld = device.GetDispatchLoader();
    if (!fg_load_vulkan(dld.vkGetInstanceProcAddr, device.GetInstance(), dld.vkGetDeviceProcAddr,
                        *device.GetLogical())) {
        LOG_ERROR(Render_Vulkan, "MihawkGen: a Vulkan entry point it needs is missing");
        return;
    }
    // The two latest presented frames, in the presented format, read by MihawkGen's shaders.
    for (size_t i = 0; i < history.size(); ++i) {
        history[i] = memory_allocator.CreateImage(VkImageCreateInfo{
            .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
            .pNext = nullptr,
            .flags = 0,
            .imageType = VK_IMAGE_TYPE_2D,
            .format = format,
            .extent = {extent.width, extent.height, 1},
            .mipLevels = 1,
            .arrayLayers = 1,
            .samples = VK_SAMPLE_COUNT_1_BIT,
            .tiling = VK_IMAGE_TILING_OPTIMAL,
            .usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
            .queueFamilyIndexCount = 0,
            .pQueueFamilyIndices = nullptr,
            .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
        });
        history_views[i] = device.GetLogical().CreateImageView(VkImageViewCreateInfo{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .pNext = nullptr,
            .flags = 0,
            .image = *history[i],
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format = format,
            .components{},
            .subresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
        });
    }
    // The shaders write rgba8; the destination's storage view has the presented format, whose
    // descriptor carries its own component order (as Fsr3FrameGen's output does).
    const fg_create_info info{
        .device = *device.GetLogical(),
        .physical_device = static_cast<VkPhysicalDevice>(device.GetPhysical()),
        .width = extent.width,
        .height = extent.height,
        .slots = SLOTS,
        .output_format = VK_FORMAT_R8G8B8A8_UNORM,
        .source_width = 0,
        .source_height = 0,
    };
    char error[256]{};
    fg_context* created = nullptr;
    if (fg_context_create(&info, &created, error, sizeof(error)) != VK_SUCCESS) {
        LOG_ERROR(Render_Vulkan, "MihawkGen: {}", error);
        return;
    }
    if (fg_context_load_network(created, fg_network_blob, fg_network_blob_size, error,
                                sizeof(error)) != VK_SUCCESS ||
        !fg_method_available(created, FG_METHOD_NEURAL)) {
        LOG_ERROR(Render_Vulkan, "MihawkGen: network: {}", error);
        fg_context_destroy(created);
        return;
    }
    context = created;
    LOG_INFO(Render_Vulkan, "MihawkGen: {}x{}, {} slots, network {} bytes", extent.width,
             extent.height, SLOTS, fg_network_blob_size);
}

MihawkFrameGen::~MihawkFrameGen() {
    // The owner (FrameGen) waits for the GPU before it lets this go.
    if (context) {
        fg_context_destroy(context);
    }
}

void MihawkFrameGen::Process(Scheduler& scheduler, VkImage current_frame, u64 frame_count) {
    if (!context) {
        return;
    }
    frames = frame_count + 1;
    const VkImage newest = *history[frame_count & 1];
    const bool first_use = frame_count < history.size();
    scheduler.Record([current_frame, newest, first_use, extent = extent](vk::CommandBuffer cmdbuf) {
        const VkImageCopy region{
            .srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
            .srcOffset = {0, 0, 0},
            .dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
            .dstOffset = {0, 0, 0},
            .extent = {extent.width, extent.height, 1},
        };
        const std::array before{
            Barrier(current_frame,
                    VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                        VK_ACCESS_TRANSFER_WRITE_BIT,
                    VK_ACCESS_TRANSFER_READ_BIT, VK_IMAGE_LAYOUT_GENERAL,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL),
            Barrier(newest, VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT,
                    first_use ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL),
        };
        cmdbuf.PipelineBarrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
                                   VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                                   VK_PIPELINE_STAGE_TRANSFER_BIT,
                               VK_PIPELINE_STAGE_TRANSFER_BIT, 0, {}, {}, before);
        cmdbuf.CopyImage(current_frame, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, newest,
                         VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, region);
        const std::array after{
            Barrier(current_frame, VK_ACCESS_TRANSFER_READ_BIT,
                    VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL),
            Barrier(newest, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL),
        };
        cmdbuf.PipelineBarrier(VK_PIPELINE_STAGE_TRANSFER_BIT,
                               VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
                                   VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                               0, {}, {}, after);
    });
}

void MihawkFrameGen::Generate(Scheduler& scheduler, VkImage destination,
                              VkImageView destination_storage_view, float t) {
    if (!context || frames < history.size()) {
        return;
    }
    const u32 slot = next_slot;
    next_slot = (next_slot + 1) % SLOTS;
    if (slot_ticks[slot] != 0 && !scheduler.IsFree(slot_ticks[slot])) {
        scheduler.Wait(slot_ticks[slot]);
    }
    slot_ticks[slot] = scheduler.CurrentTick();
    const fg_frame_pair pair{
        .prev_color = *history_views[(frames - 2) & 1],
        .next_color = *history_views[(frames - 1) & 1],
        .t = t,
    };
    scheduler.Record([this, pair, slot, destination,
                      destination_storage_view](vk::CommandBuffer cmdbuf) {
        const auto discard = Barrier(destination, 0, VK_ACCESS_SHADER_WRITE_BIT,
                                     VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);
        cmdbuf.PipelineBarrier(VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                               VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, {}, {}, discard);
        fg_record_generate(context, *cmdbuf, slot, FG_METHOD_NEURAL, &pair,
                           destination_storage_view);
        const auto done = Barrier(destination, VK_ACCESS_SHADER_WRITE_BIT,
                                  VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_TRANSFER_READ_BIT |
                                      VK_ACCESS_COLOR_ATTACHMENT_READ_BIT,
                                  VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL);
        cmdbuf.PipelineBarrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                               VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, {}, {}, done);
    });
}

} // namespace Vulkan
