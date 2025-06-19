set (src    "./main.cpp"
            "./led.cpp"
            "./button.cpp"
            "./driver.cpp"
            "./current_sense.cpp"
            "../drivers/INA219/ina219.c"
    )

set (inc    "."
            "./include/"
            "../drivers/INA219/"
    )

idf_component_register(SRCS ${src}
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver
                    INCLUDE_DIRS ${inc})