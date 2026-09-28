cmake_minimum_required (VERSION 3.20)

project ("fast-multipole-method")


set(EXTERNAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

add_subdirectory(${EXTERNAL_DIR}/glm)


add_executable (fast_multipole_method "src/main.cpp" "src/simulation.cpp")
target_include_directories(fast_multipole_method PRIVATE "include" ${EXTERNAL_DIR}/rgfw ${EXTERNAL_DIR}/glad/include)
target_link_libraries(fast_multipole_method PRIVATE glm::glm-header-only)
if(APPLE)
    target_link_libraries(fast_multipole_method PRIVATE glm::glm-header-only "-framework Cocoa")
elseif(UNIX)
	find_package(X11 REQUIRED COMPONENTS Xrandr Xi Xcursor)
	target_link_libraries(fast_multipole_method PRIVATE glm::glm-header-only X11::X11 X11::Xrandr X11::Xi X11::Xcursor)
endif()

enable_testing()
add_subdirectory(${EXTERNAL_DIR}/grtest)

add_executable(test_simulation "tests/test_simulation.cpp" "src/simulation.cpp")
target_include_directories(test_simulation PRIVATE "include")
target_link_libraries(test_simulation PRIVATE grtest glm::glm-header-only)
grtest_discover_tests(test_simulation)

add_executable(test_quad_tree "tests/test_quad_tree.cpp")
target_include_directories(test_quad_tree PRIVATE "include")
target_link_libraries(test_quad_tree PRIVATE grtest glm::glm-header-only)
grtest_discover_tests(test_quad_tree)

add_executable(test_multipole "tests/test_multipole.cpp")
target_include_directories(test_multipole PRIVATE "include")
target_link_libraries(test_multipole PRIVATE grtest glm::glm-header-only)
grtest_discover_tests(test_multipole)
