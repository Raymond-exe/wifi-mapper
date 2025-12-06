file(GLOB_RECURSE app_sources "*.c")

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "."
    REQUIRES GUI LCD XPT2046
)
