set (src    "./main.cpp"
            "./led.cpp"
            "./button.cpp"
            "./driver.cpp"
            "./current_sense.cpp"
            "./pid_controller.cpp"
            "../components/drivers/INA219/ina219.c"
    )

set (inc    "."
            "./include/"
            "../components/drivers/INA219/"
    )

idf_component_register(SRCS ${src}
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver
                    INCLUDE_DIRS ${inc})