idf_component_register(SRCS "controller_module.cpp"
                    REQUIRES arduino
                    REQUIRES driver
                    REQUIRES esp-dsp
                    INCLUDE_DIRS "."
                    )
