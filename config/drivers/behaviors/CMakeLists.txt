zephyr_library_named(hanc_behavior_tp_tune)

zephyr_library_sources(behavior_tp_tune.c)

# ZMK repo lives as a sibling of zephyr in the west workspace; reach its
# app/include for <drivers/behavior.h> and <zmk/behavior.h>.
zephyr_library_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../input/include
    ${ZEPHYR_BASE}/../zmk/app/include
)
