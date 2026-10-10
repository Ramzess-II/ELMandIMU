# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Версия прошивки: попадает в описание образа, в строки NGS и NGI и в имя файла выпуска.
set(PROJECT_VER "0.3.3")

# Прошивка подписывается (sdkconfig.defaults). Ключа в репозитории нет: у каждого он свой.
if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/keys/nogps_signing_key.pem")
    message(WARNING "Нет ключа подписи keys/nogps_signing_key.pem. Создать: "
                    "python -m espsecure generate_signing_key --version 2 --scheme rsa3072 "
                    "keys/nogps_signing_key.pem  (README, раздел «Подпись»)")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ELMandIMU)
