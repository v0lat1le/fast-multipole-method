#include <fstream>

template<typename F>
struct extract_vulkan_type;
template<typename R, typename... Args>
struct extract_vulkan_type<R(VKAPI_CALL*)(Args...)> {
    using type = typename std::tuple_element<sizeof...(Args)-1, std::tuple<Args...>>::type;
};

template<typename F, typename... Args>
auto vkGetThing(F func, Args&&... args) {
    using T = std::remove_pointer<typename extract_vulkan_type<F>::type>::type;
    T result;
    func(std::forward<Args>(args)..., &result);
    return result;
}

template<typename F, typename... Args>
auto vkGetThings(F func, Args&&... args) {
    std::uint32_t count{};
    func(std::forward<Args>(args)..., &count, nullptr);
    using T = std::remove_pointer<typename extract_vulkan_type<F>::type>::type;
    std::vector<T> result(count);
    func(std::forward<Args>(args)..., &count, result.data());
    return result;
}

struct Renderer {
    bool display_quadtree = false;

    RGFW_window* window;
    VkInstance instance;
    VkSurfaceKHR surface = nullptr;
    VkPhysicalDevice physical_device;
    VkDevice device = nullptr;

    std::uint32_t queue_family_index = 0;
    VkQueue graphics_queue = nullptr;

    VkFormat swapchain_format = VK_FORMAT_B8G8R8A8_SRGB;
    VkExtent2D swapchain_extent = {1200, 1200};
    VkSwapchainKHR swapchain = nullptr;
    std::vector<VkImage> swapchain_images;
    std::vector<VkImageView> swapchain_image_views;

    VkShaderModule shader_module = nullptr;
    VkPipelineLayout pipeline_layout = nullptr;
    VkPipeline pipeline = nullptr;

    VkCommandPool command_pool = nullptr;
    std::uint32_t frame_index = 0;
    std::vector<VkCommandBuffer> command_buffers;
    std::vector<VkSemaphore> present_complete_semaphores;
    std::vector<VkSemaphore> render_finished_semaphores;
    std::vector<VkFence> in_flight_fences;

    struct GpuMemory {
        std::byte* cpu;
        VkDeviceAddress gpu;
        VkBuffer buffer;
        VkDeviceMemory memory;
    };
    GpuMemory working_memory;

    void create_instance() {
        VkApplicationInfo app_info{
            .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
            .pApplicationName = "FMM",
            .applicationVersion = VK_MAKE_VERSION(1, 0, 0),
            .pEngineName = "No Engine",
            .engineVersion = VK_MAKE_VERSION(1, 0, 0),
            .apiVersion = VK_API_VERSION_1_4,
        };

        std::size_t extension_count = 0;
        auto requred_extensions = RGFW_getRequiredInstanceExtensions_Vulkan(&extension_count);
        VkInstanceCreateInfo instace_info{
            .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
            .pApplicationInfo = &app_info,
            .enabledLayerCount = 0,
            .ppEnabledLayerNames = nullptr,
            .enabledExtensionCount = static_cast<std::uint32_t>(extension_count),
            .ppEnabledExtensionNames = requred_extensions,
        };
#ifndef NDEBUG
        auto validationLayers = std::to_array({ "VK_LAYER_KHRONOS_validation" });
        instace_info.enabledLayerCount = validationLayers.size();
        instace_info.ppEnabledLayerNames = validationLayers.data();
#endif
        if (vkCreateInstance(&instace_info, nullptr, &instance) != VK_SUCCESS) {
            throw std::runtime_error("failed to create instance!");
        }
    }

    void create_device() {
        std::uint32_t physicalDevices = 1;
        vkEnumeratePhysicalDevices(instance, &physicalDevices, &physical_device);

        auto properties = vkGetThings(vkGetPhysicalDeviceQueueFamilyProperties, physical_device);
        auto graphics_family = std::find_if(properties.begin(), properties.end(), [](auto& prop) { return prop.queueFlags & VK_QUEUE_GRAPHICS_BIT; });

        float priority = 0.5;
        queue_family_index = std::distance(properties.begin(), graphics_family);
        VkDeviceQueueCreateInfo queue_info{ 
            .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
            .queueFamilyIndex = queue_family_index,
            .queueCount = 1,
            .pQueuePriorities = &priority,
        };

        VkPhysicalDeviceVulkan11Features features11{
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES,
            .shaderDrawParameters = true,
        };
        VkPhysicalDeviceVulkan12Features features12 { 
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES,
            .pNext = &features11,
            .descriptorIndexing = true,
            .shaderSampledImageArrayNonUniformIndexing = true,
            .descriptorBindingVariableDescriptorCount = true,
            .runtimeDescriptorArray = true,
            .bufferDeviceAddress = true,
        };
        VkPhysicalDeviceVulkan13Features features13 {
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES,
            .pNext = &features12,
            .synchronization2 = true,
            .dynamicRendering = true,
        };
        auto extensions = std::to_array({ VK_KHR_SWAPCHAIN_EXTENSION_NAME, VK_EXT_EXTENDED_DYNAMIC_STATE_3_EXTENSION_NAME });
        VkDeviceCreateInfo device_info{
            .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
            .pNext = &features13,
            .queueCreateInfoCount = 1,
            .pQueueCreateInfos = &queue_info,
            .enabledExtensionCount = extensions.size(),
            .ppEnabledExtensionNames = extensions.data(),
        };
        vkCreateDevice(physical_device, &device_info, nullptr, &device);

        vkGetDeviceQueue(device, queue_family_index, 0, &graphics_queue);
    }

    void create_swapchain() {
        auto surfaceFormats = vkGetThings(vkGetPhysicalDeviceSurfaceFormatsKHR, physical_device, surface);
        auto surfaceCapabilities = vkGetThing(vkGetPhysicalDeviceSurfaceCapabilitiesKHR, physical_device, surface);

        VkSwapchainCreateInfoKHR swapchain_info{ 
            .sType            = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR,
            .surface          = surface,
            .minImageCount    = 3,
            .imageFormat      = swapchain_format,
            .imageColorSpace  = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR,
            .imageExtent      = swapchain_extent,
            .imageArrayLayers = 1,
            .imageUsage       = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
            .imageSharingMode = VK_SHARING_MODE_EXCLUSIVE,
            .preTransform     = surfaceCapabilities.currentTransform,
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
            VkImageView imageView;
            vkCreateImageView(device, &image_view_info, nullptr, &imageView);
            swapchain_image_views.push_back(imageView);
        }
    }

    std::vector<char> read_file(std::string_view path) {
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

        VkShaderModuleCreateInfo shader_module_info {
            .sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
            .codeSize = shader_code.size(),
            .pCode = reinterpret_cast<const std::uint32_t*>(shader_code.data()),
        };
        vkCreateShaderModule(device, &shader_module_info, nullptr, &shader_module);

        auto shader_stages = std::to_array<VkPipelineShaderStageCreateInfo>({{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .stage = VK_SHADER_STAGE_VERTEX_BIT,
            .module = shader_module,
            .pName = "vertMain"
        }, {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
            .module = shader_module,
            .pName = "fragMain"
        }});

        auto dynamic_state = std::to_array<VkDynamicState>({
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR,
            VK_DYNAMIC_STATE_PRIMITIVE_TOPOLOGY
        });
        VkPipelineDynamicStateCreateInfo dynamic_state_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
            .dynamicStateCount = dynamic_state.size(),
            .pDynamicStates = dynamic_state.data()
        };

        VkPipelineVertexInputStateCreateInfo vertex_input_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
        };

        VkPipelineInputAssemblyStateCreateInfo input_assembly_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO
        };

        VkPipelineViewportStateCreateInfo viewport_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
            .viewportCount = 1,
            .scissorCount = 1
        };

        VkPipelineRasterizationStateCreateInfo raster_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
            .polygonMode = VK_POLYGON_MODE_FILL,
            .cullMode = VK_CULL_MODE_BACK_BIT,
            .frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
            .lineWidth = 1.0f,
        };

        VkPipelineMultisampleStateCreateInfo multi_sample_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
            .rasterizationSamples = VK_SAMPLE_COUNT_1_BIT
        };

        //VkPipelineDepthStencilStateCreateInfo depth_stencil_info{
        //    .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
        //    .depthTestEnable = VK_TRUE,
        //    .depthWriteEnable = VK_TRUE,
        //    .depthCompareOp = VK_COMPARE_OP_LESS,
        //    .stencilTestEnable = VK_FALSE
        //};

        VkPipelineColorBlendAttachmentState attach_state {
            .blendEnable = VK_FALSE,
            .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT
        };

        VkPipelineColorBlendStateCreateInfo blend_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
            .logicOp = VK_LOGIC_OP_CLEAR,
            .attachmentCount = 1,
            .pAttachments = &attach_state
        };
        VkPushConstantRange push_constant_range {
            .stageFlags = VK_SHADER_STAGE_VERTEX_BIT,
            .size = 128
        };

        VkPipelineLayoutCreateInfo pipeline_layout_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
            .setLayoutCount = 0,
            .pushConstantRangeCount = 1,
            .pPushConstantRanges = &push_constant_range,
        };
        if (vkCreatePipelineLayout(device, &pipeline_layout_info, nullptr, &pipeline_layout) != VK_SUCCESS) {
            throw std::runtime_error("Unable to create the pipeline layout");
        }

        VkPipelineRenderingCreateInfo render_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
            .colorAttachmentCount = 1,
            .pColorAttachmentFormats = &swapchain_format,
            //.depthAttachmentFormat = depthFormat
        };

        VkGraphicsPipelineCreateInfo pipeline_info {
            .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
            .pNext = &render_info,
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
            .layout = pipeline_layout,
            .renderPass = VK_NULL_HANDLE,
        };
        if (vkCreateGraphicsPipelines(device, nullptr, 1, &pipeline_info, nullptr, &pipeline) != VK_SUCCESS) {
            throw std::runtime_error("Error creating the pipeline");
        }
    }

    void create_command_buffers() {
        VkCommandPoolCreateInfo pool_info {
            .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
            .flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
            .queueFamilyIndex = queue_family_index,
        };
        vkCreateCommandPool(device, &pool_info, nullptr, &command_pool);

        command_buffers.resize(2);
        VkCommandBufferAllocateInfo command_buffer_info{
            .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
            .commandPool = command_pool,
            .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
            .commandBufferCount = static_cast<std::uint32_t>(command_buffers.size()),
        };
        vkAllocateCommandBuffers(device, &command_buffer_info, command_buffers.data());
    }

    void record_command_buffer(std::uint32_t image_index, std::size_t n_points, std::size_t n_lines, VkDeviceAddress shader_data) {
        auto command_buffer = command_buffers[frame_index];
        VkCommandBufferBeginInfo begin_buffer_info { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
        vkBeginCommandBuffer(command_buffer, &begin_buffer_info);

        transition_image_layout(
            image_index,
            VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            {},
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT
        );

        VkRenderingAttachmentInfo attachment_info {
            .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .imageView   = swapchain_image_views[image_index],
            .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp     = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue  = {0.0, 0.0, 0.0, 0.0},
        };

        VkRenderingInfo rendering_info {
            .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
            .renderArea = {
                .offset = {0, 0},
                .extent = swapchain_extent,
            },
            .layerCount           = 1,
            .colorAttachmentCount = 1,
            .pColorAttachments    = &attachment_info
        };

        vkCmdBeginRendering(command_buffer, &rendering_info);
        vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        VkViewport viewport{0.0f, 0.0f, swapchain_extent.width, swapchain_extent.height, 0.0f, 1.0f};
        vkCmdSetViewport(command_buffer, 0, 1, &viewport);
        VkRect2D scissor{{0,0}, swapchain_extent };
        vkCmdSetScissor(command_buffer, 0, 1, &scissor);

        int mode = 0;
        if (n_lines) {
            VkDeviceAddress lines_data = shader_data+n_points*sizeof(glm::vec2)+n_points*sizeof(float);
            vkCmdSetPrimitiveTopology(command_buffer, VK_PRIMITIVE_TOPOLOGY_LINE_LIST);
            vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(VkDeviceAddress), &lines_data);
            float lines_color[3] = {0.0, 0.0, 0.6};
            vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, 16, 3*sizeof(float), lines_color);
            mode = 1;
            vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, 16+3*sizeof(float), sizeof(int), &mode);
            vkCmdDraw(command_buffer, n_lines, 1, 0, 0);
        }

        vkCmdSetPrimitiveTopology(command_buffer, VK_PRIMITIVE_TOPOLOGY_POINT_LIST);
        vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(VkDeviceAddress), &shader_data);
        VkDeviceAddress mass_data = shader_data+n_points*sizeof(glm::vec2);
        vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, sizeof(VkDeviceAddress), sizeof(VkDeviceAddress), &mass_data);
        float points_color[3] = { 1.0f, 1.0f, 1.0f };
        vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, 16, 3*sizeof(float), points_color);
        mode = 0;
        vkCmdPushConstants(command_buffer, pipeline_layout, VK_SHADER_STAGE_VERTEX_BIT, 16+3*sizeof(float), sizeof(int), &mode);
        vkCmdDraw(command_buffer, n_points, 1, 0, 0);
        vkCmdEndRendering(command_buffer);

        transition_image_layout(
            image_index,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
            {},
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT
        );

        vkEndCommandBuffer(command_buffer);
    }

    void transition_image_layout(uint32_t image_index, VkImageLayout old_layout, VkImageLayout new_layout,
        VkAccessFlagBits2 src_access_mask, VkAccessFlagBits2 dst_access_mask,
        VkPipelineStageFlagBits2 src_stage_mask, VkPipelineStageFlagBits2 dst_stage_mask) {
        VkImageMemoryBarrier2 barrier {
            .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
            .srcStageMask        = src_stage_mask,
            .srcAccessMask       = src_access_mask,
            .dstStageMask        = dst_stage_mask,
            .dstAccessMask       = dst_access_mask,
            .oldLayout           = old_layout,
            .newLayout           = new_layout,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .image               = swapchain_images[image_index],
            .subresourceRange    = {
                .aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT,
                .baseMipLevel   = 0,
                .levelCount     = 1,
                .baseArrayLayer = 0,
                .layerCount     = 1
            }
        };
        VkDependencyInfo dependency_info {
            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .imageMemoryBarrierCount = 1,
            .pImageMemoryBarriers = &barrier
        };
        vkCmdPipelineBarrier2(command_buffers[frame_index], &dependency_info);
    }

    void create_sync_objects() {
        VkSemaphoreCreateInfo semaphore_info { .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        VkFenceCreateInfo fence_info {
            .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
            .flags = VK_FENCE_CREATE_SIGNALED_BIT
        };
        for (int i=0; i<swapchain_images.size(); ++i) {
            VkSemaphore semaphore;
            vkCreateSemaphore(device, &semaphore_info, nullptr, &semaphore);
            render_finished_semaphores.push_back(semaphore);
        }
        for (int i=0; i<command_buffers.size(); ++i) {
            VkSemaphore semaphore;
            vkCreateSemaphore(device, &semaphore_info, nullptr, &semaphore);
            present_complete_semaphores.push_back(semaphore);
            VkFence fence;
            vkCreateFence(device, &fence_info, nullptr, &fence);
            in_flight_fences.push_back(fence);
        }
    }

    uint32_t find_gpu_memory_type(uint32_t type_filter, VkMemoryPropertyFlags properties) {
        auto memory_properties = vkGetThing(vkGetPhysicalDeviceMemoryProperties, physical_device);
        for (uint32_t i = 0; i < memory_properties.memoryTypeCount; i++) {
            if ((type_filter & (1 << i)) && (memory_properties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }
        throw std::runtime_error("failed to find suitable memory type!");
    }

    GpuMemory allocate_gpu_memory(std::size_t size) {
        constexpr VkBufferUsageFlags universal_buffer_usage =
            VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT |
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

        constexpr VkMemoryPropertyFlags cpu_visible_memory_properties =
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

        GpuMemory result{};

        VkBufferCreateInfo buffer_info {
            .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
            .size = size,
            .usage = universal_buffer_usage,
            .sharingMode = VK_SHARING_MODE_EXCLUSIVE
        };
        vkCreateBuffer(device, &buffer_info, nullptr, &result.buffer);

        auto buffer_requirements = vkGetThing(vkGetBufferMemoryRequirements, device, result.buffer);
        const VkMemoryAllocateFlagsInfo alloc_flags_info{
            .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO,
            .flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT,
        };
        VkMemoryAllocateInfo alloc_info{
            .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
            .pNext = &alloc_flags_info,
            .allocationSize = buffer_requirements.size,
            .memoryTypeIndex = find_gpu_memory_type(buffer_requirements.memoryTypeBits, cpu_visible_memory_properties),
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
    }

    void destroy_swapchain() {
        for (auto view: swapchain_image_views) {
            vkDestroyImageView(device, view, nullptr);
        }
        vkDestroySwapchainKHR(device, swapchain, nullptr);
    }

    ~Renderer() {
        vkDeviceWaitIdle(device);

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
        vkDestroyPipelineLayout(device, pipeline_layout, nullptr);
        vkDestroyShaderModule(device, shader_module, nullptr);

        destroy_swapchain();

        vkDestroyDevice(device, nullptr);
        vkDestroySurfaceKHR(instance, surface, nullptr);
        vkDestroyInstance(instance, nullptr);
    }

    void render(const Simulation& simulation) {
        if (window->w == 0 || window->h == 0) {
            return;
        }

        std::byte* frame_working_mem = working_memory.cpu + frame_index*16*1024*1024;
        for (std::size_t i=0; i<simulation.positions.size(); ++i) {
            reinterpret_cast<glm::vec2*>(frame_working_mem)[i] = simulation.positions[i];
        }
        frame_working_mem += simulation.positions.size()*sizeof(glm::vec2);
        for (std::size_t i=0; i<simulation.masses.size(); ++i) {
            reinterpret_cast<float*>(frame_working_mem)[i] = simulation.masses[i];
        }
        frame_working_mem += simulation.masses.size()*sizeof(float);
        std::size_t n_lines = 0;
        if (display_quadtree) {
            std::vector<glm::vec2> lineVertices = quadtree_lines(simulation.quadtree);
            for (std::size_t i=0; i<lineVertices.size(); ++i) {
                reinterpret_cast<glm::vec2*>(frame_working_mem)[i] = lineVertices[i];
            }
            n_lines = lineVertices.size()/2;
            frame_working_mem += lineVertices.size()*sizeof(glm::vec2);
        }

        vkWaitForFences(device, 1, in_flight_fences.data()+frame_index, true, UINT64_MAX);
        if (window->w != swapchain_extent.width || window->h != swapchain_extent.height) {
            vkDeviceWaitIdle(device);

            destroy_swapchain();

            swapchain_extent = { static_cast<std::uint32_t>(window->w), static_cast<std::uint32_t>(window->h) };
            create_swapchain();
        }

        std::uint32_t image_index;
        auto result = vkAcquireNextImageKHR(device, swapchain, UINT64_MAX, present_complete_semaphores[frame_index], VK_NULL_HANDLE, &image_index);

        vkResetFences(device, 1, in_flight_fences.data()+frame_index);
        vkResetCommandBuffer(command_buffers[frame_index], 0);
        record_command_buffer(image_index, simulation.positions.size(), n_lines, working_memory.gpu + frame_index*16*1024*1024);
        VkPipelineStageFlags wait_dst_stage_mask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkSubmitInfo submitInfo {
            .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
            .waitSemaphoreCount   = 1,
            .pWaitSemaphores      = present_complete_semaphores.data()+frame_index,
            .pWaitDstStageMask    = &wait_dst_stage_mask,
            .commandBufferCount   = 1,
            .pCommandBuffers      = command_buffers.data()+frame_index,
            .signalSemaphoreCount = 1,
            .pSignalSemaphores    = render_finished_semaphores.data()+image_index
        };
        vkQueueSubmit(graphics_queue, 1, &submitInfo, in_flight_fences[frame_index]);

        VkPresentInfoKHR present_info {
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
};
