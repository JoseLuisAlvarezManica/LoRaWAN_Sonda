idf_component_register(SRCS "main.c" "lora_hal.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver spi_flash nvs_flash esp_timer espressif__esp-modbus)
