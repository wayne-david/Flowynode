idf_component_register(SRCS "flowynode.c"
                       INCLUDE_DIRS "."
                        REQUIRES driver OLEDDisplay lora protocol
                       )
