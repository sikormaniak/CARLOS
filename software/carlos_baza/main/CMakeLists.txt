idf_component_register(SRCS "carlos_baza.c"
                       INCLUDE_DIRS "."
                       REQUIRES micro_ros_espidf_component
                                esp_hosted
                                bdc_motor
                                pid_ctrl
                                driver)