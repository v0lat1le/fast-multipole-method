cmake_minimum_required (VERSION 3.20)

project ("fast-multipole-method")

option(FMM_RENDERER "Renderer to use: opengl/vulkan" "default")


# External libs
set(EXTERNAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

add_library(external_glm INTERFACE)
add_library(external::glm ALIAS external_glm)
target_include_directories(external_glm INTERFACE ${EXTERNAL_DIR}/glm)
target_compile_definitions(external_glm INTERFACE GLM_FORCE_MESSAGES GLM_FORCE_INTRINSICS)


# Core
add_executable (fast_multipole_method "src/main.cpp" "src/simulation.cpp")
target_include_directories(fast_multipole_method PRIVATE "include" ${EXTERNAL_DIR}/rgfw ${EXTERNAL_DIR}/glad/include)
target_link_libraries(fast_multipole_method PRIVATE external::glm)
target_compile_options(fast_multipole_method PRIVATE "-fexperimental-library")
if(WIN32)
	if(${FMM_RENDERER} STREQUAL vulkan)
		find_package (Vulkan 1.4.335 REQUIRED)
		find_program(SLANGC_EXECUTABLE NAMES slangc HINTS "$ENV{VULKAN_SDK}/bin" REQUIRED)

		function (target_add_slang_shaders TARGET)
			cmake_parse_arguments ("SHADER" "" "" "SOURCES" ${ARGN})
			set (SHADERS_DIR shaders)
			set (ENTRY_POINTS -entry vertMain -entry fragMain)
			add_custom_command (
					OUTPUT ${SHADERS_DIR}
					COMMAND ${CMAKE_COMMAND} -E make_directory ${SHADERS_DIR}
			)
			add_custom_command (
					OUTPUT  ${SHADERS_DIR}/slang.spv
					COMMAND ${SLANGC_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_SOURCES} -target spirv -profile spirv_1_4 -emit-spirv-directly -fvk-use-entrypoint-name ${ENTRY_POINTS} -o slang.spv
					WORKING_DIRECTORY ${SHADERS_DIR}
					DEPENDS ${SHADERS_DIR} ${SHADER_SOURCES}
					COMMENT "Compiling Slang Shaders"
					VERBATIM
			)
			add_custom_target (${TARGET}_shaders DEPENDS ${SHADERS_DIR}/slang.spv)
			add_dependencies(${TARGET} ${TARGET}_shaders)
		endfunction()

		target_link_libraries(fast_multipole_method PRIVATE Vulkan::Vulkan)
		target_compile_definitions(fast_multipole_method PRIVATE FMM_RENDERER_VULKAN)
		target_add_slang_shaders(fast_multipole_method SOURCES "src/shader.slang")
	endif()
elseif(APPLE)
    target_link_libraries(fast_multipole_method PRIVATE "-framework Cocoa")
elseif(UNIX)
	find_package(X11 REQUIRED COMPONENTS Xrandr Xi Xcursor)
	target_link_libraries(fast_multipole_method PRIVATE X11::X11 X11::Xrandr X11::Xi X11::Xcursor)
endif()


# Tests

enable_testing()
add_subdirectory(${EXTERNAL_DIR}/grtest)

add_executable(test_simulation "tests/test_simulation.cpp" "src/simulation.cpp")
target_include_directories(test_simulation PRIVATE "include")
target_link_libraries(test_simulation PRIVATE grtest external::glm)
grtest_discover_tests(test_simulation)

add_executable(test_quad_tree "tests/test_quad_tree.cpp")
target_include_directories(test_quad_tree PRIVATE "include")
target_link_libraries(test_quad_tree PRIVATE grtest external::glm)
grtest_discover_tests(test_quad_tree)

add_executable(test_multipole "tests/test_multipole.cpp")
target_include_directories(test_multipole PRIVATE "include")
target_link_libraries(test_multipole PRIVATE grtest external::glm)
grtest_discover_tests(test_multipole)
