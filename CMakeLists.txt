# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Inicializa o pico_sdk a partir da localização instalada
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(u3-i4-mpu6050 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add all source files to the project
file(GLOB SOURCE_FILES "src/*.c")
file(GLOB_RECURSE DRIVER_FILES "src/drivers/*.c")

# -----------------------------------------------------------------------------
# 1. CRIAR O EXECUTÁVEL (A "Casa")
# O target "u3-i4-mpu6050" passa a existir a partir desta linha.
# É crucial que este bloco venha ANTES de gerar o header PIO.
# -----------------------------------------------------------------------------
add_executable(u3-i4-mpu6050 
    ${SOURCE_FILES} 
    ${DRIVER_FILES}
)

# -----------------------------------------------------------------------------
# 2. GERAR O HEADER PIO (Os "Móveis")
# Agora que o target existe, podemos anexar o arquivo PIO a ele.
# Certifique-se que ws2812.pio está dentro da pasta 'src'.
# -----------------------------------------------------------------------------
pico_generate_pio_header(u3-i4-mpu6050 ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)

# Detalhes do programa
pico_set_program_name(u3-i4-mpu6050 "u3-i4-mpu6050")
pico_set_program_version(u3-i4-mpu6050 "0.1")

# Habilitar/Desabilitar saída USB/UART
pico_enable_stdio_uart(u3-i4-mpu6050 0)
pico_enable_stdio_usb(u3-i4-mpu6050 1)

# -----------------------------------------------------------------------------
# 3. VINCULAR BIBLIOTECAS (Link Libraries)
# Adicionado hardware_pio para controlar o NeoPixel
# -----------------------------------------------------------------------------
target_link_libraries(u3-i4-mpu6050
        pico_stdlib
        hardware_i2c
        hardware_pwm
        hardware_pio  # <--- ESSENCIAL PARA O NEOPIXEL
    )

# Adicionar diretórios de include
target_include_directories(u3-i4-mpu6050 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/drivers
)

# Gerar arquivos extras (.uf2, .hex, etc)
pico_add_extra_outputs(u3-i4-mpu6050)