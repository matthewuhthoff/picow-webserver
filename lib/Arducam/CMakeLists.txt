project(Arducam)
set( DIR_CAM_SRCS
    picoHal.h
    cam_spi_master.h
    ArducamCamera.h
    cam_spi_master.c 
    ArducamCamera.c
    )
add_library(libarducam ${DIR_CAM_SRCS})
target_link_libraries(libarducam PUBLIC hardware_spi pico_stdlib )
