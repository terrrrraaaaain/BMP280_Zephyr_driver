# Copyright (c) 2026 Franciszek Trzeciak
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_CUSTOM_BMP280 bmp280.c)
zephyr_library_link_libraries(zephyr_interface)
zephyr_library_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)
zephyr_library_include_directories(.)