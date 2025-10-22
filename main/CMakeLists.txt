set (src    "./main.cpp"
            "./led.cpp"
            "./button.cpp"
    )

set (inc    "."
            "./include/"
    )

idf_component_register(SRCS ${src}
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver
                    INCLUDE_DIRS ${inc})