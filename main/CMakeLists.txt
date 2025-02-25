idf_component_register(SRCS "usb_cdc_ecm_example_main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES usb esp_eth esp_wifi usb_host_cdc_ecm
                    )
