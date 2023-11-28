idf_component_register(SRCS "main_rt_model.cpp" "include/drivers/manchester_encoder.c"
                    REQUIRES driver console esp_rom
                    INCLUDE_DIRS "." "include" "/home/sasisekhar/esp/esp-idf/components/esp_rom/include/esp32s3/rom")

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
target_compile_options(${COMPONENT_LIB} PRIVATE "-frtti")
target_compile_options(${COMPONENT_LIB} PRIVATE "-fexceptions")
target_compile_options(${COMPONENT_LIB} PRIVATE "-DRT_ESP32")
target_compile_options(${COMPONENT_LIB} PRIVATE "-DNO_LOGGING")