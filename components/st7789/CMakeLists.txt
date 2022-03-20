set(COMPONENT_SRCS "src/st7789.c" "src/fontx.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_REQUIRES
    driver
)

register_component()
