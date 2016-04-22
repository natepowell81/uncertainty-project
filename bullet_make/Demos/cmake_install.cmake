# Install script for directory: /Users/np/Desktop/Bullet/bullet-2.82-r2704/Demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/HelloWorld/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/MultiThreadedDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/OpenCLClothDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/OpenGL/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/AllBulletDemos/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/ConvexDecompositionDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/CcdPhysicsDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/BulletXmlImportDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/ConstraintDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/SliderConstraintDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/GenericJointDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/Raytracer/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/ForkLiftDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/BasicDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/FeatherstoneMultiBodyDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/RollingFrictionDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/RaytestDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/VoronoiFractureDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/GyroscopicDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/FractureDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/Box2dDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/BspDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/MovingConcaveDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/VehicleDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/UserCollisionAlgorithm/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/CharacterDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/SoftDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/CollisionInterfaceDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/ConcaveConvexcastDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/SimplexDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/DynamicControlDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/ConvexHullDistance/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/DoublePrecisionDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/ConcaveDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/CollisionDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/ContinuousConvexCollision/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/ConcaveRaycastDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/GjkConvexCastDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/MultiMaterialDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/SerializeDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/InternalEdgeDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/Benchmarks/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/ThreadingDemo/cmake_install.cmake")
  include("/Users/np/Desktop/Bullet/bullet_make/Demos/VectorAdd_OpenCL/cmake_install.cmake")

endif()

