// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>

#include "common/common_types.h"
#include "video_core/vulkan_common/vulkan_memory_allocator.h"
#include "video_core/vulkan_common/vulkan_wrapper.h"

struct fg_context;

namespace Vulkan {

class Device;
class Scheduler;

/// MihawkGen (Eglantine/externals/mihawkgen): a trained network that makes the picture between two
/// presented frames from their colour alone (no motion vectors, no depth). The two latest frames
/// are kept as copies; Generate records MihawkGen's compute passes, which write the frame at t
/// straight into the destination's storage view, as Fsr3FrameGen does.
class MihawkFrameGen {
public:
    explicit MihawkFrameGen(const Device& device, MemoryAllocator& memory_allocator,
                            VkExtent2D extent, VkFormat format);
    ~MihawkFrameGen();

    MihawkFrameGen(const MihawkFrameGen&) = delete;
    MihawkFrameGen& operator=(const MihawkFrameGen&) = delete;

    /// Whether the network and its passes were made (false: MihawkGen is unavailable here).
    [[nodiscard]] bool IsValid() const {
        return context != nullptr;
    }

    /// Keeps the frame just presented as the newest of the two.
    void Process(Scheduler& scheduler, VkImage current_frame, u64 frame_count);

    /// Writes the frame at t (0 < t < 1) between the two kept frames into the destination.
    void Generate(Scheduler& scheduler, VkImage destination, VkImageView destination_storage_view,
                  float t);

private:
    // Descriptor sets and the network's images are kept per slot: a slot is not reused before the
    // submission that last used it has completed.
    static constexpr u32 SLOTS = 4;

    VkExtent2D extent;
    std::array<vk::Image, 2> history;
    std::array<vk::ImageView, 2> history_views;
    std::array<u64, SLOTS> slot_ticks{};
    u32 next_slot = 0;
    u64 frames = 0;
    fg_context* context = nullptr;
};

} // namespace Vulkan
