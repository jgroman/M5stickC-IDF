FILE(GLOB SOURCES *.c)

idf_component_register(
    SRCS
        ${SOURCES}
    INCLUDE_DIRS 
        "."
    REQUIRES
        axp192
        mpu6886
        st7735s
        wire
)
