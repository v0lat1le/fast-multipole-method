cmake_minimum_required (VERSION 3.20)

project ("fast-multipole-method")

find_package( OpenGL REQUIRED )
include_directories( ${OPENGL_INCLUDE_DIRS} )

set( GLFW_BUILD_DOCS OFF CACHE BOOL  "GLFW lib only" )
set( GLFW_INSTALL OFF CACHE BOOL  "GLFW lib only" )
include_directories("include")
include_directories("glfw/deps")

add_subdirectory( glfw )

add_executable (fast_multipole_method "src/main.cpp" "src/simulation.cpp")
target_link_libraries(fast_multipole_method ${OPENGL_LIBRARIES} glfw)

enable_testing()
add_subdirectory(grtest)

add_executable(test_simulation "tests/test_simulation.cpp" "src/simulation.cpp")
target_link_libraries(test_simulation PRIVATE grtest)
grtest_discover_tests(test_simulation)

add_executable(test_quad_tree "tests/test_quad_tree.cpp")
target_link_libraries(test_quad_tree PRIVATE grtest)
grtest_discover_tests(test_quad_tree)

add_executable(test_multipole "tests/test_multipole.cpp")
target_link_libraries(test_multipole PRIVATE grtest)
grtest_discover_tests(test_multipole)
