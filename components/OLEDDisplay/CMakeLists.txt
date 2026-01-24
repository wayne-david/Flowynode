set(component_srcs "OLEDDisplayFonts.c")

idf_component_register(SRCS "OLEDDisplay.c" 
										"OLEDDisplayFonts.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "driver")


