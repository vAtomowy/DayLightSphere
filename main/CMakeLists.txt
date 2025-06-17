# AB

set (src    "./main.cpp"
            "./led.cpp"
            "./button.cpp"
            "./driver.cpp"
            "./ina219.c"
            "./pi4ioe5v9554.c"
    )

set (inc    "."
            "./include/"
    )

idf_component_register(SRCS ${src}
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver
                    INCLUDE_DIRS ${inc})