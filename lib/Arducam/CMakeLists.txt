project(Arducam)
set( DIR_CAM_SRCS
    picoHal.h
    cam_spi_master.h
    ArducamCamera.h
    Arducam_Mega.h
    cam_spi_master.c 
    ArducamCamera.c
    Arducam_Mega.cpp
    )
add_library(libarducam ${DIR_CAM_SRCS})
target_link_libraries(libarducam PUBLIC hardware_spi pico_stdlib )
