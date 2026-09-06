# AIGC note: component sources for the ESP-IDF classic-BT build. Only the
# transport-agnostic protocol files, the classic transport and the IDF entry
# point are compiled here; Arduino-only files (BLE, MPU, CLI) stay out.
idf_component_register(
    SRCS "joycon_report.cpp"
         "motion.cpp"
         "joycon_subcmd.cpp"
         "joycon_btclassic.cpp"
         "idf_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash)
