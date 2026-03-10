idf_component_register(SRCS "main.c" "mpu6050.c" "motor.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver freertos
                       )