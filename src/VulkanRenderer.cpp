#include <fstream>

template<typename F>
struct extract_vulkan_type;
template<typename R, typename... Args>
struct extract_vulkan_type<R(VKAPI_CALL*)(Args...)> {
    using type = std::tuple_element_t<sizeof...(Args)-1, std::tuple<Args...>>;
};

template<typename F, typename... Args>
auto vkGetThing(F func, Args&&... args) {
    using T = std::remove_pointer_t<typename extract_vulkan_type<F>::type>;
    T result;
    func(std::forward<Args>(args)..., &result);
    return result;
}

template<typename F, typename... Args>
auto vkGetThings(F func, Args&&... args) {
    std::uint32_t count{};
    func(std::forward<Args>(args)..., &count, nullptr);
    using T = std::remove_pointer_t<typename extract_vulkan_type<F>::type>;
    std::vector<T> result(count);
    func(std::forward<Args>(args)..., &count, result.data());
    return result;
}

struct Renderer {
    bool display_quadtree = false;
    float zoom = 1.0f;
    glm::vec2 center = {0.0, 0.0};

    RGFW_window* window = nullptr;
    VkInstance instance = nullptr;
    VkSurfaceKHR surface = nullptr;
    VkPhysicalDevice physical_device = nullptr;
    VkDevice device = nullptr;

    std::uint32_t queue_family_index = 0;
    VkQueue graphics_queue = nullptr;

    VkFormat swapchain_format = VK_FORMAT_B8G8R8A8_SRGB;
    VkExtent2D swapchain_extent = {1200, 1200};
    VkSwapchainKHR swapchain = nullptr;
    std::vector<VkImage> swapchain_images;
    std::vector<VkImageView> swapchain_image_views;

    VkPipeline pipeline = nullptr;

    VkCommandPool command_pool = nullptr;
    std::uint32_t frame_index = 0;
    std::vector<VkCommandBuffer> command_buffers;
    std::vector<VkSemaphore> present_complete_semaphores;
    std::vector<VkSemaphore> render_finished_semaphores;
    std::vector<VkFence> in_flight_fences;

    PFN_vkCmdPushDataEXT vkCmdPushDataEXT;
    PFN_vkWriteResourceDescriptorsEXT vkWriteResourceDescriptorsEXT;
    PFN_vkWriteSamplerDescriptorsEXT vkWriteSamplerDescriptorsEXT;
    PFN_vkCmdBindResourceHeapEXT vkCmdBindResourceHeapEXT;
    PFN_vkCmdBindSamplerHeapEXT vkCmdBindSamplerHeapEXT;
    PFN_vkCmdCopyMemoryToImageKHR vkCmdCopyMemoryToImageKHR;

    struct GpuMemory {
        std::byte* cpu;
        VkDeviceAddress gpu;
        VkBuffer buffer;
        VkDeviceMemory memory;
    };
    GpuMemory working_memory;

    // texture stuff
    VkPhysicalDeviceDescriptorHeapPropertiesEXT heap_properties{ .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_HEAP_PROPERTIES_EXT };
    VkImage font_image;
    VkDeviceMemory font_image_memory;
    GpuMemory descriptor_heap;

    // text
    std::vector<std::pair<glm::uvec2, std::string>> text;

    void add_text(glm::uvec2 pos, std::string str) {
        text.emplace_back(pos, str);
    }

    void create_instance() {
        const VkApplicationInfo app_info{
            .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
            .pApplicationName = "FMM",
            .applicationVersion = VK_MAKE_VERSION(1, 0, 0),
            .pEngineName = "No Engine",
            .engineVersion = VK_MAKE_VERSION(1, 0, 0),
            .apiVersion = VK_API_VERSION_1_4,
        };

        std::size_t extension_count = 0;
        auto required_extensions = RGFW_getRequiredInstanceExtensions_Vulkan(&extension_count);
        VkInstanceCreateInfo instance_info{
            .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
            .pApplicationInfo = &app_info,
            .enabledLayerCount = 0,
            .ppEnabledLayerNames = nullptr,
            .enabledExtensionCount = static_cast<std::uint32_t>(extension_count),
            .ppEnabledExtensionNames = required_extensions,
        };
#ifndef NDEBUG
        auto validation_layers = std::to_array({ "VK_LAYER_KHRONOS_validation" });
        instance_info.enabledLayerCount = validation_layers.size();
        instance_info.ppEnabledLayerNames = validation_layers.data();
#endif
        if (vkCreateInstance(&instance_info, nullptr, &instance) != VK_SUCCESS) {
            throw std::runtime_error("failed to create instance!");
        }
    }

    void create_device() {
        auto devices = vkGetThings(vkEnumeratePhysicalDevices, instance);
        physical_device = devices.front();

        auto properties = vkGetThings(vkGetPhysicalDeviceQueueFamilyProperties, physical_device);
        auto graphics_family = std::find_if(properties.begin(), properties.end(), [](auto& prop) { return prop.queueFlags & VK_QUEUE_GRAPHICS_BIT; });

        float priority = 0.5;
        queue_family_index = std::distance(properties.begin(), graphics_family);
        const VkDeviceQueueCreateInfo queue_info{
            .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
            .queueFamilyIndex = queue_family_index,
            .queueCount = 1,
            .pQueuePriorities = &priority,
        };
        VkPhysicalDeviceUnifiedImageLayoutsFeaturesKHR unified_image_layouts {
             .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_UNIFIED_IMAGE_LAYOUTS_FEATURES_KHR,
             .unifiedImageLayouts = VK_TRUE,
        };
        VkPhysicalDeviceShaderUntypedPointersFeaturesKHR shader_untyped_pointers {
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_UNTYPED_POINTERS_FEATURES_KHR,
            .pNext = &unified_image_layouts,
            .shaderUntypedPointers = VK_TRUE,
        };
        VkPhysicalDeviceDeviceAddressCommandsFeaturesKHR address_commands_features {
            .sType= VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DEVICE_ADDRESS_COMMANDS_FEATURES_KHR,
            .pNext = &shader_untyped_pointers,
            .deviceAddressCommands = VK_TRUE,
        };
        VkPhysicalDeviceDescriptorHeapFeaturesEXT descriptor_heap_features {
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_HEAP_FEATURES_EXT,
            .pNext =  &address_commands_features,
            .descriptorHeap = VK_TRUE,
        };
        VkPhysicalDeviceVulkan14Features features14 {
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_4_FEATURES,
            .pNext = &descriptor_heap_features,
            .maintenance5 = VK_TRUE,
        };
        VkPhysicalDeviceVulkan13Features features13 {
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES,
            .pNext = &features14,
            .synchronization2 = VK_TRUE,
            .dynamicRendering = VK_TRUE,
        };
        VkPhysicalDeviceVulkan12Features features12 { 
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES,
            .pNext = &features13,
            .bufferDeviceAddress = VK_TRUE,
        };
        VkPhysicalDeviceVulkan11Features features11{
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES,
            .pNext = &features12,
            .shaderDrawParameters = VK_TRUE,
        };
        VkPhysicalDeviceFeatures2 features_core {
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2,
            .pNext = &features11,
        };

        auto extensions = std::to_array({
            VK_KHR_SWAPCHAIN_EXTENSION_NAME,
            VK_EXT_EXTENDED_DYNAMIC_STATE_3_EXTENSION_NAME,
            VK_EXT_DESCRIPTOR_HEAP_EXTENSION_NAME,
            VK_KHR_SHADER_UNTYPED_POINTERS_EXTENSION_NAME,
            VK_KHR_DEVICE_ADDRESS_COMMANDS_EXTENSION_NAME,
            VK_KHR_UNIFIED_IMAGE_LAYOUTS_EXTENSION_NAME
        });
        const VkDeviceCreateInfo device_info {
            .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
            .pNext = &features_core,
            .queueCreateInfoCount = 1,
            .pQueueCreateInfos = &queue_info,
            .enabledExtensionCount = extensions.size(),
            .ppEnabledExtensionNames = extensions.data(),
        };
        vkCreateDevice(physical_device, &device_info, nullptr, &device);

#define load_device_proc_addr(device, method) reinterpret_cast<PFN_##method>(vkGetDeviceProcAddr(device, #method));
        vkCmdPushDataEXT = load_device_proc_addr(device, vkCmdPushDataEXT);
        vkWriteResourceDescriptorsEXT = load_device_proc_addr(device, vkWriteResourceDescriptorsEXT);
        vkWriteSamplerDescriptorsEXT = load_device_proc_addr(device, vkWriteSamplerDescriptorsEXT);
        vkCmdBindSamplerHeapEXT = load_device_proc_addr(device, vkCmdBindSamplerHeapEXT);
        vkCmdBindResourceHeapEXT = load_device_proc_addr(device, vkCmdBindResourceHeapEXT);
        vkCmdCopyMemoryToImageKHR = load_device_proc_addr(device, vkCmdCopyMemoryToImageKHR);
#undef load_device_proc_addr
        vkGetDeviceQueue(device, queue_family_index, 0, &graphics_queue);
    }


    void create_swapchain() {
        auto surface_formats = vkGetThings(vkGetPhysicalDeviceSurfaceFormatsKHR, physical_device, surface);
        auto surface_capabilities = vkGetThing(vkGetPhysicalDeviceSurfaceCapabilitiesKHR, physical_device, surface);

        const VkSwapchainCreateInfoKHR swapchain_info{
            .sType            = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR,
            .surface          = surface,
            .minImageCount    = 3,
            .imageFormat      = swapchain_format,
            .imageColorSpace  = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR,
            .imageExtent      = swapchain_extent,
            .imageArrayLayers = 1,
            .imageUsage       = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
            .imageSharingMode = VK_SHARING_MODE_EXCLUSIVE,
            .preTransform     = surface_capabilities.currentTransform,
            .compositeAlpha   = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
            .presentMode      = VK_PRESENT_MODE_FIFO_KHR,
            .clipped          = true,
        };
        vkCreateSwapchainKHR(device, &swapchain_info, nullptr, &swapchain);

        swapchain_images = vkGetThings(vkGetSwapchainImagesKHR, device, swapchain);
        VkImageViewCreateInfo image_view_info{
            .sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format   = swapchain_format,
            .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
        };

        swapchain_image_views.clear();
        for (auto image: swapchain_images) {
            image_view_info.image = image;
            VkImageView image_view;
            vkCreateImageView(device, &image_view_info, nullptr, &image_view);
            swapchain_image_views.push_back(image_view);
        }
    }

    static std::vector<char> read_file(std::string_view path) {
        std::ifstream file(path.data(), std::ios::ate | std::ios::binary);

        if (!file.is_open()) {
            throw std::runtime_error("failed to open file!");
        }

        std::vector<char> buffer(file.tellg());
        file.seekg(0, std::ios::beg);
        file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        file.close();

        return buffer;
    }

    void create_graphics_pipeline() {
        auto shader_code = read_file("shaders/slang.spv");

        const VkShaderModuleCreateInfo shader_module_info {
            .sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
            .codeSize = shader_code.size(),
            .pCode = reinterpret_cast<const std::uint32_t*>(shader_code.data()),
        };

        const VkDescriptorSetAndBindingMappingEXT root_mapping {
            .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_AND_BINDING_MAPPING_EXT,
            .bindingCount = 1,
            .resourceMask = VK_SPIRV_RESOURCE_TYPE_UNIFORM_BUFFER_BIT_EXT | VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
            .source = VK_DESCRIPTOR_MAPPING_SOURCE_PUSH_ADDRESS_EXT,
            .sourceData = {.pushAddressOffset = 0},
        };

        auto mappings = std::to_array<VkShaderDescriptorSetAndBindingMappingInfoEXT>({{
            .sType = VK_STRUCTURE_TYPE_SHADER_DESCRIPTOR_SET_AND_BINDING_MAPPING_INFO_EXT,
            .pNext = &shader_module_info,
            .mappingCount = 1,
            .pMappings = &root_mapping
         }});

        auto shader_stages = std::to_array<VkPipelineShaderStageCreateInfo>({{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .pNext = mappings.data(),
            .stage = VK_SHADER_STAGE_VERTEX_BIT,
            .pName = "vertMain"
        }, {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .pNext = mappings.data(),
            .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
            .pName = "fragMain"
        }});

        auto dynamic_state = std::to_array<VkDynamicState>({
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR,
            VK_DYNAMIC_STATE_PRIMITIVE_TOPOLOGY
        });
        const VkPipelineDynamicStateCreateInfo dynamic_state_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
            .dynamicStateCount = dynamic_state.size(),
            .pDynamicStates = dynamic_state.data()
        };

        const VkPipelineVertexInputStateCreateInfo vertex_input_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
        };

        const VkPipelineInputAssemblyStateCreateInfo input_assembly_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO
        };

        const VkPipelineViewportStateCreateInfo viewport_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
            .viewportCount = 1,
            .scissorCount = 1
        };

        const VkPipelineRasterizationStateCreateInfo raster_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
            .polygonMode = VK_POLYGON_MODE_FILL,
            .cullMode = VK_CULL_MODE_NONE,
            .frontFace = VK_FRONT_FACE_CLOCKWISE,
            .lineWidth = 1.0f,
        };

        const VkPipelineMultisampleStateCreateInfo multi_sample_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
            .rasterizationSamples = VK_SAMPLE_COUNT_1_BIT
        };

        const VkPipelineColorBlendAttachmentState attach_state {
            .blendEnable = VK_TRUE,
            .srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA,
            .dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
            .colorBlendOp = VK_BLEND_OP_ADD,
            .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
            .dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO,
            .alphaBlendOp = VK_BLEND_OP_ADD,
            .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT,
        };

        const VkPipelineColorBlendStateCreateInfo blend_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
            .logicOp = VK_LOGIC_OP_CLEAR,
            .attachmentCount = 1,
            .pAttachments = &attach_state
        };

        const VkPipelineRenderingCreateInfo rendering_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
            .colorAttachmentCount = 1,
            .pColorAttachmentFormats = &swapchain_format,
            //.depthAttachmentFormat = depthFormat
        };

        const VkPipelineCreateFlags2CreateInfo flags_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_CREATE_FLAGS_2_CREATE_INFO,
            .pNext = &rendering_info,
            .flags = VK_PIPELINE_CREATE_2_DESCRIPTOR_HEAP_BIT_EXT,
        };

        const VkGraphicsPipelineCreateInfo pipeline_info {
            .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
            .pNext = &flags_info,
            .stageCount = shader_stages.size(),
            .pStages = shader_stages.data(),
            .pVertexInputState = &vertex_input_info,
            .pInputAssemblyState = &input_assembly_info,
            .pViewportState = &viewport_info,
            .pRasterizationState = &raster_info,
            .pMultisampleState = &multi_sample_info,
            // .pDepthStencilState = &depth_stencil_info,
            .pColorBlendState = &blend_info,
            .pDynamicState = &dynamic_state_info,
            .basePipelineIndex = -1,
        };
        if (vkCreateGraphicsPipelines(device, nullptr, 1, &pipeline_info, nullptr, &pipeline) != VK_SUCCESS) {
            throw std::runtime_error("Error creating the pipeline");
        }
    }

    void create_command_buffers() {
        const VkCommandPoolCreateInfo pool_info {
            .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
            .flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
            .queueFamilyIndex = queue_family_index,
        };
        vkCreateCommandPool(device, &pool_info, nullptr, &command_pool);

        command_buffers.resize(2);
        const VkCommandBufferAllocateInfo command_buffer_info{
            .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
            .commandPool = command_pool,
            .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
            .commandBufferCount = static_cast<std::uint32_t>(command_buffers.size()),
        };
        vkAllocateCommandBuffers(device, &command_buffer_info, command_buffers.data());
    }

    void push_data(VkCommandBuffer command_buffer, std::uint32_t offset, std::size_t size, VkDeviceAddress data) {
        const VkPushDataInfoEXT push_data {
            .sType = VK_STRUCTURE_TYPE_PUSH_DATA_INFO_EXT,
            .offset = offset,
            .data = {.address = &data, .size = size}
        };
        vkCmdPushDataEXT(command_buffer, &push_data);
    }

    std::pair<VkCommandBuffer, std::uint32_t> begin_rendering() {
        vkWaitForFences(device, 1, in_flight_fences.data()+frame_index, true, UINT64_MAX);

        if (window->w != swapchain_extent.width || window->h != swapchain_extent.height) {
            vkDeviceWaitIdle(device);

            destroy_swapchain();

            swapchain_extent = { static_cast<std::uint32_t>(window->w), static_cast<std::uint32_t>(window->h) };
            create_swapchain();
        }

        std::uint32_t image_index;
        vkAcquireNextImageKHR(device, swapchain, UINT64_MAX, present_complete_semaphores[frame_index], VK_NULL_HANDLE, &image_index);

        vkResetFences(device, 1, in_flight_fences.data()+frame_index);

        auto command_buffer = command_buffers[frame_index];
        vkResetCommandBuffer(command_buffer, 0);

        const VkCommandBufferBeginInfo begin_buffer_info{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
        vkBeginCommandBuffer(command_buffer, &begin_buffer_info);

        VkImageMemoryBarrier2 barrier {
            .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
            .srcStageMask        = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            .dstStageMask        = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            .dstAccessMask       = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
            .oldLayout           = VK_IMAGE_LAYOUT_UNDEFINED,
            .newLayout           = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .image               = swapchain_images[image_index],
            .subresourceRange    = {
                .aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT,
                .levelCount     = 1,
                .layerCount     = 1
            }
        };
        const VkDependencyInfo dependency_info {
            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .imageMemoryBarrierCount = 1,
            .pImageMemoryBarriers = &barrier
        };
        vkCmdPipelineBarrier2(command_buffers[frame_index], &dependency_info);

        const VkRenderingAttachmentInfo attachment_info{
            .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .imageView   = swapchain_image_views[image_index],
            .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp     = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue  = {{0.0f, 0.0f, 0.0f, 0.0f}},
        };
        const VkRenderingInfo rendering_info{
            .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
            .renderArea = {
                .offset = {.x = 0, .y = 0},
                .extent = swapchain_extent,
            },
            .layerCount           = 1,
            .colorAttachmentCount = 1,
            .pColorAttachments    = &attachment_info
        };
        vkCmdBeginRendering(command_buffer, &rendering_info);

        auto extent_size = std::min(swapchain_extent.width, swapchain_extent.height);
        auto x = (swapchain_extent.width-extent_size)/2.0f;
        auto y = (swapchain_extent.height-extent_size)/2.0f;
        const VkViewport viewport{ x, y, extent_size, extent_size, 0.0f, 1.0f };
        vkCmdSetViewport(command_buffer, 0, 1, &viewport);

        const VkRect2D scissor{ {0,0}, swapchain_extent };
        vkCmdSetScissor(command_buffer, 0, 1, &scissor);

        return { command_buffer, image_index };
    }

    void end_rendering(VkCommandBuffer command_buffer, std::uint32_t image_index) {
        vkCmdEndRendering(command_buffer);

        const VkImageMemoryBarrier2 barrier {
            .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
            .srcStageMask        = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            .srcAccessMask       = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
            .dstStageMask        = VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
            .oldLayout           = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .newLayout           = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .image               = swapchain_images[image_index],
            .subresourceRange    = {
                .aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT,
                .levelCount     = 1,
                .layerCount     = 1
            }
        };
        const VkDependencyInfo dependency_info {
            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .imageMemoryBarrierCount = 1,
            .pImageMemoryBarriers = &barrier
        };
        vkCmdPipelineBarrier2(command_buffer, &dependency_info);

        vkEndCommandBuffer(command_buffer);

        const  VkPipelineStageFlags wait_dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        const VkSubmitInfo submit_info{
            .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
            .waitSemaphoreCount   = 1,
            .pWaitSemaphores      = present_complete_semaphores.data()+frame_index,
            .pWaitDstStageMask    = &wait_dst_stage_mask,
            .commandBufferCount   = 1,
            .pCommandBuffers      = &command_buffer,
            .signalSemaphoreCount = 1,
            .pSignalSemaphores    = render_finished_semaphores.data()+image_index
        };
        vkQueueSubmit(graphics_queue, 1, &submit_info, in_flight_fences[frame_index]);

        const VkPresentInfoKHR present_info{
            .sType              = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR,
            .waitSemaphoreCount = 1,
            .pWaitSemaphores    = render_finished_semaphores.data()+image_index,
            .swapchainCount     = 1,
            .pSwapchains        = &swapchain,
            .pImageIndices      = &image_index
        };
        vkQueuePresentKHR(graphics_queue, &present_info);

        frame_index = (frame_index+1)%command_buffers.size();
    }

    void create_sync_objects() {
        const VkSemaphoreCreateInfo semaphore_info { .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        const VkFenceCreateInfo fence_info {
            .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
            .flags = VK_FENCE_CREATE_SIGNALED_BIT
        };
        for (std::size_t i=0; i<swapchain_images.size(); ++i) {
            VkSemaphore semaphore;
            vkCreateSemaphore(device, &semaphore_info, nullptr, &semaphore);
            render_finished_semaphores.push_back(semaphore);
        }
        for (std::size_t i=0; i<command_buffers.size(); ++i) {
            VkSemaphore semaphore;
            vkCreateSemaphore(device, &semaphore_info, nullptr, &semaphore);
            present_complete_semaphores.push_back(semaphore);
            VkFence fence;
            vkCreateFence(device, &fence_info, nullptr, &fence);
            in_flight_fences.push_back(fence);
        }
    }

    uint32_t find_gpu_memory_type(uint32_t type_filter, VkMemoryPropertyFlags properties) {
        const auto memory_properties = vkGetThing(vkGetPhysicalDeviceMemoryProperties, physical_device);
        for (uint32_t i = 0; i < memory_properties.memoryTypeCount; i++) {
            if ((type_filter & (1 << i)) && (memory_properties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }
        throw std::runtime_error("failed to find suitable memory type!");
    }

    GpuMemory allocate_gpu_memory(std::size_t size, VkBufferUsageFlags usage, VkMemoryPropertyFlags memory_properties) {
        GpuMemory result{};

        const VkBufferCreateInfo buffer_info{
            .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
            .size = size,
            .usage = usage,
            .sharingMode = VK_SHARING_MODE_EXCLUSIVE
        };
        vkCreateBuffer(device, &buffer_info, nullptr, &result.buffer);

        const auto buffer_requirements = vkGetThing(vkGetBufferMemoryRequirements, device, result.buffer);
        const VkMemoryAllocateFlagsInfo alloc_flags_info{
            .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO,
            .flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT,
        };
        const VkMemoryAllocateInfo alloc_info{
            .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
            .pNext = &alloc_flags_info,
            .allocationSize = buffer_requirements.size,
            .memoryTypeIndex = find_gpu_memory_type(buffer_requirements.memoryTypeBits, memory_properties),
        };
        vkAllocateMemory(device, &alloc_info, nullptr, &result.memory);
        vkBindBufferMemory(device, result.buffer, result.memory, 0);
        vkMapMemory(device, result.memory, 0, VK_WHOLE_SIZE, 0, reinterpret_cast<void**>(&result.cpu));
        const VkBufferDeviceAddressInfo address_info{
            .sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO,
            .buffer = result.buffer,
        };
        result.gpu = vkGetBufferDeviceAddress(device, &address_info);

        return result;
    }

    static constexpr VkBufferUsageFlags universal_buffer_usage =
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT |
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

    static constexpr VkMemoryPropertyFlags cpu_visible_memory_properties =
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
        VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

    GpuMemory allocate_gpu_memory(std::size_t size) {
        return allocate_gpu_memory(size, universal_buffer_usage, cpu_visible_memory_properties);
    }

    GpuMemory allocate_descriptor_heap(std::size_t size) {
        return allocate_gpu_memory(size, universal_buffer_usage | VK_BUFFER_USAGE_DESCRIPTOR_HEAP_BIT_EXT, cpu_visible_memory_properties);
    }

    void load_textures() {
        auto command_buffer = command_buffers[frame_index];
        vkResetCommandBuffer(command_buffer, 0);

        const VkCommandBufferBeginInfo begin_buffer_info{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
        vkBeginCommandBuffer(command_buffer, &begin_buffer_info);

        const VkImageCreateInfo image_info = {
            .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
            .imageType = VK_IMAGE_TYPE_2D,
            .format = VK_FORMAT_R8_UNORM,
            .extent = {
                .width = 200,
                .height = 81,
                .depth = 1,
            },
            .mipLevels = 1,
            .arrayLayers = 1,
            .samples = VK_SAMPLE_COUNT_1_BIT,
            .tiling = VK_IMAGE_TILING_OPTIMAL,
            .usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
            .queueFamilyIndexCount = 1,
            .pQueueFamilyIndices = &queue_family_index,
            .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
        };
        vkCreateImage(device, &image_info, nullptr, &font_image);

        const VkDeviceImageMemoryRequirements requirements_info{
            .sType = VK_STRUCTURE_TYPE_DEVICE_IMAGE_MEMORY_REQUIREMENTS,
            .pCreateInfo = &image_info,
        };
        VkMemoryRequirements2 requirements{
            .sType = VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2,
        };
        vkGetDeviceImageMemoryRequirements(device, &requirements_info, &requirements);
        auto texture_memory_type = find_gpu_memory_type(requirements.memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT); // avoid VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;

        const VkMemoryAllocateInfo allocate_info{
            .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
            .allocationSize = 32*1024*1024,
            .memoryTypeIndex = texture_memory_type,
        };
        vkAllocateMemory(device, &allocate_info, nullptr, &font_image_memory);

        vkBindImageMemory(device, font_image, font_image_memory, 0);

        const VkImageMemoryBarrier2 barrier{
            .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
            .srcStageMask = VK_PIPELINE_STAGE_2_NONE,
            .dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
            .dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT,
            .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
            .newLayout = VK_IMAGE_LAYOUT_GENERAL,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .image = font_image,
            .subresourceRange = {
                .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
                .levelCount = 1,
                .layerCount = 1,
            },
        };
        const VkDependencyInfo dependency_info{
            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .imageMemoryBarrierCount = 1,
            .pImageMemoryBarriers = &barrier
        };
        vkCmdPipelineBarrier2(command_buffer, &dependency_info);

        std::ifstream file("font.r8", std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("failed to open file!");
        }
        file.read(reinterpret_cast<char*>(working_memory.cpu), 1024*1024);
        file.close();

        const VkDeviceMemoryImageCopyKHR region {
            .sType = VK_STRUCTURE_TYPE_DEVICE_MEMORY_IMAGE_COPY_KHR,
            .addressRange = {
                .address = working_memory.gpu,
                .size = 200*81,
            },
            .addressFlags = VK_ADDRESS_COMMAND_FULLY_BOUND_BIT_KHR,
            .addressRowLength = 0,
            .addressImageHeight = 0,
            .imageSubresource = {
                .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
                .mipLevel = 0,
                .baseArrayLayer = 0,
                .layerCount = 1,
            },
            .imageLayout = VK_IMAGE_LAYOUT_GENERAL,
            .imageOffset = {
                .x = 0,
                .y = 0,
                .z = 0,
            },
            .imageExtent = {
                .width = 200,
                .height = 81,
                .depth = 1,
            },
        };
        const VkCopyDeviceMemoryImageInfoKHR info {
            .sType = VK_STRUCTURE_TYPE_COPY_DEVICE_MEMORY_IMAGE_INFO_KHR,
            .image = font_image,
            .regionCount = 1,
            .pRegions = &region,
        };
        vkCmdCopyMemoryToImageKHR(command_buffer, &info);

        vkEndCommandBuffer(command_buffer);

        const VkSubmitInfo submitInfo{
            .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
            .commandBufferCount   = 1,
            .pCommandBuffers      = &command_buffer,
        };
        vkResetFences(device, 1, in_flight_fences.data());
        vkQueueSubmit(graphics_queue, 1, &submitInfo, in_flight_fences[frame_index]);
        vkWaitForFences(device, 1, in_flight_fences.data(), VK_TRUE, UINT64_MAX);

        VkPhysicalDeviceProperties2 properties2{
           .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2,
           .pNext = &heap_properties,
        };
        vkGetPhysicalDeviceProperties2(physical_device, &properties2);

        descriptor_heap = allocate_descriptor_heap(heap_properties.imageDescriptorSize+heap_properties.samplerDescriptorSize+heap_properties.minResourceHeapReservedRange+heap_properties.minSamplerHeapReservedRange);

        const VkImageUsageFlags descriptor_usage = VK_IMAGE_USAGE_SAMPLED_BIT;
        const VkImageAspectFlags descriptor_aspect = VK_IMAGE_ASPECT_COLOR_BIT;
        const VkImageViewUsageCreateInfo view_usage{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_USAGE_CREATE_INFO,
            .usage = descriptor_usage,
        };
        const VkImageViewCreateInfo view_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .pNext = &view_usage,
            .image = font_image,
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format = VK_FORMAT_R8_UNORM,
            .subresourceRange = {
                .aspectMask = descriptor_aspect,
                .baseMipLevel =0,
                .levelCount = 1,
                .baseArrayLayer = 0,
                .layerCount = 1,
            },
        };
        const VkImageDescriptorInfoEXT image_descriptor{
            .sType = VK_STRUCTURE_TYPE_IMAGE_DESCRIPTOR_INFO_EXT,
            .pView = &view_info,
            .layout = VK_IMAGE_LAYOUT_GENERAL,
        };
        const VkResourceDescriptorInfoEXT descriptor_info{
            .sType = VK_STRUCTURE_TYPE_RESOURCE_DESCRIPTOR_INFO_EXT,
            .type = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,
            .data = {.pImage = &image_descriptor},
        };
        const VkHostAddressRangeEXT tex_destination{
            .address = descriptor_heap.cpu,
            .size = heap_properties.imageDescriptorSize,
        };
        vkWriteResourceDescriptorsEXT(device, 1, &descriptor_info, &tex_destination);

        const VkSamplerCreateInfo sampler_info{
            .sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
            .magFilter = VK_FILTER_NEAREST,
            .minFilter = VK_FILTER_NEAREST,
            .mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST,
            .addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .anisotropyEnable = VK_FALSE,
            .maxAnisotropy = 1.0f,
            .compareEnable = VK_FALSE,
            .compareOp = VK_COMPARE_OP_LESS_OR_EQUAL,
            .maxLod = VK_LOD_CLAMP_NONE,
        };
        const VkHostAddressRangeEXT samp_destination{
            .address = descriptor_heap.cpu + heap_properties.imageDescriptorSize,
            .size = heap_properties.samplerDescriptorSize,
        };
        vkWriteSamplerDescriptorsEXT(device, 1, &sampler_info, &samp_destination);
    }

    Renderer(RGFW_window* window_) {
        window = window_;
        create_instance();
        RGFW_window_createSurface_Vulkan(window, instance, &surface);
        create_device();
        create_swapchain();
        create_graphics_pipeline();
        create_command_buffers();
        create_sync_objects();
        working_memory = allocate_gpu_memory(32*1024*1024);
        load_textures();
    }

    void destroy_swapchain() {
        for (auto view: swapchain_image_views) {
            vkDestroyImageView(device, view, nullptr);
        }
        vkDestroySwapchainKHR(device, swapchain, nullptr);
    }

    ~Renderer() {
        vkDeviceWaitIdle(device);

        vkFreeMemory(device, font_image_memory, nullptr);
        vkDestroyImage(device, font_image, nullptr);

        vkFreeMemory(device, descriptor_heap.memory, nullptr);
        vkDestroyBuffer(device, descriptor_heap.buffer, nullptr);

        vkFreeMemory(device, working_memory.memory, nullptr);
        vkDestroyBuffer(device, working_memory.buffer, nullptr);
        for (auto semaphore: present_complete_semaphores) {
            vkDestroySemaphore(device, semaphore, nullptr);
        }
        for (auto semaphore: render_finished_semaphores) {
            vkDestroySemaphore(device, semaphore, nullptr);
        }
        for (auto fence: in_flight_fences) {
            vkDestroyFence(device, fence, nullptr);
        }
        vkDestroyCommandPool(device, command_pool, nullptr);
        vkDestroyPipeline(device, pipeline, nullptr);

        destroy_swapchain();

        vkDestroyDevice(device, nullptr);
        vkDestroySurfaceKHR(instance, surface, nullptr);
        vkDestroyInstance(instance, nullptr);
    }

    void render(const Simulation& simulation) {
        if (window->w == 0 || window->h == 0) {
            return;
        }

        auto [command_buffer, image_index] = begin_rendering();

        std::byte* frame_working_mem = working_memory.cpu + frame_index*16*1024*1024 + 256;
        for (std::size_t i=0; i<simulation.positions.size(); ++i) {
            reinterpret_cast<glm::vec2*>(frame_working_mem)[i] = simulation.positions[i];
        }
        frame_working_mem += simulation.positions.size()*sizeof(glm::vec2);
        for (std::size_t i=0; i<simulation.masses.size(); ++i) {
            reinterpret_cast<float*>(frame_working_mem)[i] = simulation.masses[i];
        }
        frame_working_mem += simulation.masses.size()*sizeof(float);
        std::uint32_t n_lines = 0;
        if (display_quadtree) {
            std::vector<glm::vec2> lineVertices = quadtree_lines(simulation.quadtree, 7+std::log2(zoom));
            for (std::size_t i=0; i<lineVertices.size(); ++i) {
                reinterpret_cast<glm::vec2*>(frame_working_mem)[i] = lineVertices[i];
            }
            n_lines = lineVertices.size();
            frame_working_mem += lineVertices.size()*sizeof(glm::vec2);
        }

        std::uint32_t n_points = simulation.positions.size();
        std::byte* cpu_shader_data = working_memory.cpu + frame_index*16*1024*1024;
        VkDeviceAddress gpu_shader_data = working_memory.gpu + frame_index*16*1024*1024;
        struct ShaderData {
            VkDeviceAddress pos;
            VkDeviceAddress mass;
            std::array<float,3> color;
            float zoom;
            glm::vec2 center;
            int mode;
        };
        vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

        const VkBindHeapInfoEXT bind_info_img{
            .sType = VK_STRUCTURE_TYPE_BIND_HEAP_INFO_EXT,
            .heapRange = {
                .address = descriptor_heap.gpu,
                .size = heap_properties.imageDescriptorSize+heap_properties.minResourceHeapReservedRange,
            },
            .reservedRangeOffset = heap_properties.imageDescriptorSize,
            .reservedRangeSize = heap_properties.minResourceHeapReservedRange,
        };
        vkCmdBindResourceHeapEXT(command_buffer, &bind_info_img);

        const VkBindHeapInfoEXT bind_info_samp{
            .sType = VK_STRUCTURE_TYPE_BIND_HEAP_INFO_EXT,
            .heapRange = {
                .address = descriptor_heap.gpu+heap_properties.imageDescriptorSize+heap_properties.minResourceHeapReservedRange,
                .size = heap_properties.samplerDescriptorSize + heap_properties.minSamplerHeapReservedRange,
            },
            .reservedRangeOffset = heap_properties.samplerDescriptorSize,
            .reservedRangeSize = heap_properties.minSamplerHeapReservedRange,
        };
        vkCmdBindSamplerHeapEXT(command_buffer, &bind_info_samp);

        ShaderData* shader_data = reinterpret_cast<ShaderData*>(cpu_shader_data);
        if (n_lines) {
            vkCmdSetPrimitiveTopology(command_buffer, VK_PRIMITIVE_TOPOLOGY_LINE_LIST);
            shader_data[1].pos = gpu_shader_data+256+n_points*sizeof(glm::vec2)+n_points*sizeof(float);
            shader_data[1].color = { 0.0f, 0.0f, 0.6f };
            shader_data[1].zoom = zoom;
            shader_data[1].center = center;
            shader_data[1].mode = 1;
            push_data(command_buffer, 0, sizeof(ShaderData), gpu_shader_data+sizeof(ShaderData));
            vkCmdDraw(command_buffer, n_lines, 1, 0, 0);
        }
        vkCmdSetPrimitiveTopology(command_buffer, VK_PRIMITIVE_TOPOLOGY_POINT_LIST);
        shader_data[0].pos = gpu_shader_data+256;
        shader_data[0].mass = gpu_shader_data+256+n_points*sizeof(glm::vec2);
        shader_data[0].color = { 1.0f, 1.0f, 1.0f };
        shader_data[0].zoom = zoom;
        shader_data[0].center = center;
        shader_data[0].mode = 0;
        push_data(command_buffer, 0, sizeof(ShaderData), gpu_shader_data);
        vkCmdDraw(command_buffer, n_points, 1, 0, 0);

        if (not text.empty()) {
#include "font.inl"
            auto points = reinterpret_cast<std::pair<glm::vec2, glm::vec2>*>(frame_working_mem);
            std::uint32_t n_text_points = 0;
            for (auto& [pos, str]: text) {
                std::uint32_t text_x = pos.x;
                for (auto& c: str) {
                    auto& font_char = *std::find_if(font_characters, font_characters+font.characterCount, [&](Character& fc) { return fc.codePoint == c; });
                    points[0].first.x  = points[4].first.x  = points[5].first.x = 2.0f*(text_x - font_char.originX)/window->w - 1.0f;
                    points[0].second.x = points[4].second.x = points[5].second.x = static_cast<float>(font_char.x)/font.width;
                    points[0].first.y  = points[1].first.y  = points[5].first.y = 2.0f*(pos.y - font_char.originY)/window->h - 1.0f;
                    points[0].second.y = points[1].second.y = points[5].second.y = static_cast<float>(font_char.y)/font.height;
                    points[1].first.x  = points[2].first.x  = points[3].first.x = 2.0f*(text_x + font_char.width - font_char.originX)/window->w - 1.0f;
                    points[1].second.x = points[2].second.x = points[3].second.x = static_cast<float>(font_char.x + font_char.width)/font.width;
                    points[2].first.y  = points[3].first.y  = points[4].first.y = 2.0f*(pos.y + font_char.height - font_char.originY)/window->h - 1.0f;
                    points[2].second.y = points[3].second.y = points[4].second.y = static_cast<float>(font_char.y + font_char.height)/font.height;
                    points += 6;
                    n_text_points += 6;
                    text_x += font_char.width;
                }
            }

            vkCmdSetPrimitiveTopology(command_buffer, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
            shader_data[2].pos = gpu_shader_data+256+n_points*sizeof(glm::vec2)+n_points*sizeof(float)+n_lines*sizeof(glm::vec2);
            shader_data[2].color = { 0.0f, 0.6f, 0.0f };
            shader_data[2].zoom = zoom;
            shader_data[2].center = center;
            shader_data[2].mode = 2;
            push_data(command_buffer, 0, sizeof(ShaderData), gpu_shader_data+2*sizeof(ShaderData));
            vkCmdDraw(command_buffer, n_text_points, 1, 0, 0);
        }
        
        end_rendering(command_buffer, image_index);
    }
};
