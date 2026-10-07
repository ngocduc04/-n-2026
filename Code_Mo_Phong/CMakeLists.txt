# cmake_minimum_required(VERSION 3.15)
# project(pid_flight LANGUAGES CXX)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # ---- Duong dan toi MuJoCo SDK: sua lai cho dung noi ban giai nen ----
# # Tai ban Windows (mujoco-x.x.x-windows-x86_64.zip) hoac Linux (mujoco-x.x.x-linux-x86_64.tar.gz)
# # tu: https://github.com/google-deepmind/mujoco/releases
# if(NOT DEFINED MUJOCO_DIR)
#   set(MUJOCO_DIR "D:/File_tai_lieu/DATN/Mujoco" CACHE PATH "Duong dan toi thu muc MuJoCo SDK da giai nen")
# endif()

# # ---- GLFW: dung ban binary co san (khong qua vcpkg) ----
# if(NOT DEFINED GLFW_DIR)
#   set(GLFW_DIR "D:/File_tai_lieu/DATN/glfw-3.5.1.bin.WIN64" CACHE PATH "Duong dan toi thu muc GLFW da giai nen")
# endif()

# set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/imgui")
# set(IMGUI_SOURCES
#     ${IMGUI_DIR}/imgui.cpp
#     ${IMGUI_DIR}/imgui_draw.cpp
#     ${IMGUI_DIR}/imgui_tables.cpp
#     ${IMGUI_DIR}/imgui_widgets.cpp
#     ${IMGUI_DIR}/imgui_demo.cpp
#     ${IMGUI_DIR}/imgui_impl_glfw.cpp
#     ${IMGUI_DIR}/imgui_impl_opengl3.cpp
# )

# add_executable(pid_flight pid_flight.cpp ${IMGUI_SOURCES})
# target_include_directories(pid_flight PRIVATE "${IMGUI_DIR}" "${GLFW_DIR}/include")

# target_include_directories(pid_flight PRIVATE "${MUJOCO_DIR}/include")
# if(WIN32)
#   if(MSVC)
#     target_link_directories(pid_flight PRIVATE "${MUJOCO_DIR}/lib" "${GLFW_DIR}/lib-vc2022")
#   else()
#     target_link_directories(pid_flight PRIVATE "${MUJOCO_DIR}/lib" "${GLFW_DIR}/lib-mingw-w64")
#   endif()
#   target_link_libraries(pid_flight PRIVATE mujoco glfw3 opengl32 gdi32)
# else()
#   target_link_directories(pid_flight PRIVATE "${MUJOCO_DIR}/lib")
#   target_link_libraries(pid_flight PRIVATE mujoco glfw GL dl pthread)
#   set_target_properties(pid_flight PROPERTIES
#     BUILD_RPATH "${MUJOCO_DIR}/lib")
# endif()

cmake_minimum_required(VERSION 3.15)
project(mpc_flight LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Duong dan toi MuJoCo SDK: sua lai cho dung noi ban giai nen ----
if(NOT DEFINED MUJOCO_DIR)
  set(MUJOCO_DIR "${CMAKE_SOURCE_DIR}/mujoco-3.3.2" CACHE PATH "Duong dan toi thu muc MuJoCo SDK da giai nen")
endif()

# ---- GLFW: dung ban binary co san (khong qua vcpkg) ----
if(NOT DEFINED GLFW_DIR)
  set(GLFW_DIR "D:/File_tai_lieu/DATN/Mujoco/glfw-3.5.1.bin.WIN64" CACHE PATH "Duong dan toi thu muc GLFW da giai nen")
endif()

# ---- TinyMPC: git clone https://github.com/TinyMPC/TinyMPC.git vao thu muc nay ----
if(NOT DEFINED TINYMPC_DIR)
  set(TINYMPC_DIR "${CMAKE_SOURCE_DIR}/TinyMPC" CACHE PATH "Duong dan toi thu muc da git clone TinyMPC")
endif()
add_subdirectory(${TINYMPC_DIR}/src/tinympc ${CMAKE_BINARY_DIR}/tinympc_build)

set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/imgui")
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/imgui_impl_opengl3.cpp
)

add_executable(mpc_flight mpc_flight.cpp ${IMGUI_SOURCES})
target_include_directories(mpc_flight PRIVATE "${IMGUI_DIR}" "${GLFW_DIR}/include")
target_include_directories(mpc_flight PRIVATE "${MUJOCO_DIR}/include")
target_link_libraries(mpc_flight PRIVATE tinympcstatic)

if(WIN32)
  if(MSVC)
    target_link_directories(mpc_flight PRIVATE "${MUJOCO_DIR}/lib" "${GLFW_DIR}/lib-vc2022")
  else()
    target_link_directories(mpc_flight PRIVATE "${MUJOCO_DIR}/lib" "${GLFW_DIR}/lib-mingw-w64")
  endif()
  target_link_libraries(mpc_flight PRIVATE mujoco glfw3 opengl32 gdi32)
else()
  target_link_directories(mpc_flight PRIVATE "${MUJOCO_DIR}/lib")
  target_link_libraries(mpc_flight PRIVATE mujoco glfw GL dl pthread)
  set_target_properties(mpc_flight PROPERTIES
    BUILD_RPATH "${MUJOCO_DIR}/lib")
endif()