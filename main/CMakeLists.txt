
# file(GLOB_RECURSE IMAGE_SOURCES images/*.c)

idf_component_register(SRCS "i80_main.c" "i80_ui.c" # ${IMAGE_SOURCES}
                    INCLUDE_DIRS "." )

