# ZCL Cluster Auto-Registration Component

idf_component_register(
    SRCS "zcl_cluster_config.c"
    INCLUDE_DIRS "include"
    REQUIRES
        sensor_universal
        esp-zigbee-lib
    PRIV_REQUIRES
        esp_timer
)

# Math library for log10 (illuminance ZCL conversion)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
