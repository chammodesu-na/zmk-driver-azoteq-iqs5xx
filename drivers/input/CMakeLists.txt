zephyr_library_amend()

# ZMK 내부 헤더 경로 추가
zephyr_include_directories(${ZEPHYR_ZMK_MODULE_DIR}/app/include)

zephyr_library_sources_ifdef(CONFIG_INPUT_AZOTEQ_IQS5XX iqs5xx.c)
