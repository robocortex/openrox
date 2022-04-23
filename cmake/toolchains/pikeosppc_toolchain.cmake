SET (CMAKE_SYSTEM_NAME PikeOS)
SET (CMAKE_SYSTEM_VERSION 1)
SET (CMAKE_SYSTEM_PROCESSOR powerpc-eabi)
SET (CMAKE_CROSSCOMPILING 1)

INCLUDE(CMakeForceCompiler)

message(CMAKE_GENERATOR ${CMAKE_GENERATOR})

SET (TOOLSROOT "/opt/cdk/ppc/e500")
SET (SYSROOT "/opt/target/ppc/e500/bposix")
SET (SDKROOT "${TOOLSROOT}/lib/gcc/powerpc-unknown-elfspe/4.4.5")

CMAKE_FORCE_C_COMPILER( "${TOOLSROOT}/bin/ppc_e500-gcc.exe" GNU)
CMAKE_FORCE_CXX_COMPILER( "${TOOLSROOT}/bin/ppc_e500-g++.exe" GNU)

# SET (CMAKE_AR      "${TOOLSROOT}/bin/ppc_e500-ar.exe")
# SET (CMAKE_LINKER  "${TOOLSROOT}/bin/ppc_e500-ld.exe")
# SET (CMAKE_NM      "${TOOLSROOT}/bin/ppc_e500-nm.exe")
# SET (CMAKE_OBJDUMP "${TOOLSROOT}/bin/ppc_e500-objdump.exe")
# SET (CMAKE_RANLIB  "${TOOLSROOT}/bin/ppc_e500-ranlib.exe")


#remove 03 and NDEBUG from flags
SET (CMAKE_CXX_FLAGS_RELEASE "")
SET (CMAKE_C_FLAGS_RELEASE "")

#set our preferred flags
SET (CMAKE_C_FLAGS "-fvisibility=hidden -s -Wl,--exclude-libs,ALL -Wl,--no-undefined -fPIC -std=c99 -ffreestanding -fno-builtin -fno-tree-scev-cprop -fno-tree-vrp -DPPC_E500 -DPPC_FSL_BOOKE -mfloat-gprs=single -mspe=no -nostdinc -fno-strict-aliasing -O2 -W -Wall")

INCLUDE_DIRECTORIES(SYSTEM "${SDKROOT}/include" "${SYSROOT}/include")


SET (PIKEOS 1)