set(CMAKE_TARGET_OS darwin)
set(CMAKE_TARGET_CPU_ARCH arm64)
set(CMAKE_TARGET_CPU_ARCH_FAMILY arm64)
set(CMAKE_SHARED_LIBS_ALLOWED_ON_TARGET TRUE)
set(CMAKE_STATIC_LIBS_ALLOWED_ON_TARGET TRUE)
set(CMAKE_COMPILER_FAMILY clang)
set(CMAKE_ASM_FLAGS "-stdlib=libc++ -fPIC")
set(CMAKE_C_FLAGS "-O2 -stdlib=libc++ -arch arm64 -fPIC -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
set(CMAKE_CXX_FLAGS "-O2 -stdlib=libc++ -fPIC -std=c++11 -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
set(CMAKE_SHARED_LINKER_FLAGS "-stdlib=libc++ -fPIC")
set(CMAKE_MODULE_LINKER_FLAGS "-stdlib=libc++ -fPIC")
set(CMAKE_EXE_LINKER_FLAGS "-stdlib=libc++ -fPIC")
set(CMAKE_OSX_ARCHITECTURES "arm64")
set(CMAKE_OSX_DEPLOYMENT_TARGET "14.4")