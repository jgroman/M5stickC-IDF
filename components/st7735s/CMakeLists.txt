FILE(GLOB SOURCES *.c)

idf_component_register(
    SRCS
        ${SOURCES}
        "./font/comic24.c"
        "./font/def_small.c"
        "./font/default_tiny_9pt.c"
        "./font/DefaultFont.c"
        "./font/DejaVuSans18.c"
        "./font/DejaVuSans24.c"
        "./font/minya24.c"
        "./font/SmallFont.c"
        "./font/tooney32.c"
        "./font/Ubuntu16.c"
    INCLUDE_DIRS 
        "."
)
