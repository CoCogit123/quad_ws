cmake_minimum_required(VERSION 3.5)
project(test_test)

#找到工作空间路径 存在WORKSPACE_PATH
get_filename_component(LEVEL1 ${CATKIN_DEVEL_PREFIX} DIRECTORY)
get_filename_component(LEVEL2 ${LEVEL1} DIRECTORY)
get_filename_component(WORKSPACE_PATH ${LEVEL2} DIRECTORY)
# message( "Eigen3_PATH: ${EIGEN3_INCLUDE_DIRS}")  # 是否找到

# 查找依赖
find_package(Eigen3 REQUIRED)
set(OSQP_INCLUDE_DIRS      ${WORKSPACE_PATH}/third_party/osqp/include) #osqp
set(OsqpEigen_INCLUDE_DIRS ${WORKSPACE_PATH}/third_party/osqp-eigen/include) #osqp-eigen
# message( "OSQP_INCLUDE_DIRS: ${OSQP_INCLUDE_DIRS}")  # 是否找到
# message( "OsqpEigen_INCLUDE_DIRS: ${OsqpEigen_INCLUDE_DIRS}")  # 是否找到


add_executable(osqp_test  test.cpp  )

target_include_directories(osqp_test PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${OSQP_INCLUDE_DIRS}
    ${OsqpEigen_INCLUDE_DIRS}
)

target_link_libraries(osqp_test
                       ${EIGEN3_LIBRARIES}
                       ${WORKSPACE_PATH}/third_party/osqp/lib/libosqp.so
                       ${WORKSPACE_PATH}/third_party/osqp/lib/libqdldl.so 
                       ${WORKSPACE_PATH}/third_party/osqp-eigen/lib/libOsqpEigen.so
                       )
