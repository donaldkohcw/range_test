idf_component_register(SRCS 
                    "blink_example_main.c"
                    "cc1101.c"
                    "moduledata.c"                     
                    "spi_task.c"                    
                    "gap.c"
                    "gatt_svr.c"                 
                    "bgt60l_spi.c"
                    "bgt60ltr11aip.c"
                    "adc_bat.c"
                    "sths34pf80_task.c"
                    "sths34pf80/sths34pf80_reg.c"
                    "gpio_wakeup.c"
                    INCLUDE_DIRS "." "sths34pf80"
                       REQUIRES bt app_update nvs_flash driver esp_adc
                    )

