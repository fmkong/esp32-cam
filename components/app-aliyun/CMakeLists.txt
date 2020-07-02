set(COMPONENT_ADD_INCLUDEDIRS .
    mqtt_handle
    )

set(COMPONENT_SRCS "mqtt_handle/mqtt_solo.c")

set(COMPONENT_REQUIRES "esp-aliyun")

register_component()
