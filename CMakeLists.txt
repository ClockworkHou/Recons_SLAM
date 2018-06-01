project(recons_slam)

add_executable(recons_slam main.cpp)

install(TARGETS recons_slam RUNTIME DESTINATION bin)

add_subdirectory(src)
add_subdirectory(Include)