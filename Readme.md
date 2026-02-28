# 🔭 BitDogLab — Sistema de Monitoramento com Sensor de Distância Laser

Sistema embarcado para a placa **BitDogLab (Raspberry Pi Pico W)** que monitora distância e inclinação em tempo real, exibe os dados em um display OLED e publica as leituras via **MQTT no broker HiveMQ** usando **FreeRTOS**.

---

## 📋 Funcionalidades

- **Sensor de distância laser VL53L0X** — leitura em milímetros via I2C
- **Sensor inercial MPU6050** — leitura de inclinação em graus via I2C
- **Display OLED SSD1306** — exibe distância, inclinação, barra de proximidade e status MQTT
- **LEDs NeoPixel WS2812 (5x5)** — indicação visual de estado (verde/laranja/vermelho)
- **Buzzer PWM** — alertas sonoros para colisão e inclinação excessiva
- **Botão A** — toggle de silenciar/ativar os LEDs (via interrupção de hardware)
- **Wi-Fi + MQTT HiveMQ** — publicação dos dados a cada 5 segundos no broker público
- **FreeRTOS** — arquitetura multitask com queue de dados entre tasks

---

## 🏗️ Arquitetura de Software

O sistema é dividido em **4 tasks FreeRTOS** que se comunicam por uma **queue** central:

```
main()
  └─ vInitTask (prioridade 2) — inicializa hardware, Wi-Fi e MQTT; se auto-deleta
       │
       ├─ vSensorTask  (prioridade 3) — lê VL53L0X + MPU6050 a cada 100ms
       │    └─ publica SensorData_t na xSensorQueue
       │
       ├─ vDisplayTask (prioridade 2) — consome queue → display + LEDs + buzzer
       │
       └─ vMqttTask    (prioridade 1) — consome queue → publica no HiveMQ a cada 5s
```

O **botão A** é tratado por **interrupção de GPIO** (não polling), garantindo resposta imediata independente das tasks.

---

## 📁 Estrutura de Arquivos

```
sensor_distancia_laser/
├── laser_distance.c        # Código principal (tasks, MQTT, sensores)
├── CMakeLists.txt          # Build system
├── FreeRTOSConfig.h        # Configuração do FreeRTOS (heap 128KB)
├── lwipopts.h              # Configuração do lwIP (OBRIGATÓRIO para MQTT)
├── ws2812.pio              # Programa PIO para LEDs WS2812
├── mpu6050/
│   ├── mpu6050.c
│   └── mpu6050.h
├── ssd1306.c               # Driver display OLED
├── display.c               # Funções de alto nível do display
└── display.h
```

---

## 🔧 Hardware Necessário

| Componente | Conexão |
|---|---|
| Raspberry Pi Pico W (BitDogLab) | — |
| Sensor laser VL53L0X | I2C0: SDA=GP0, SCL=GP1 |
| IMU MPU6050 | I2C0: SDA=GP0, SCL=GP1 |
| Display OLED SSD1306 | I2C0: SDA=GP0, SCL=GP1 |
| LEDs NeoPixel WS2812 5x5 | GP7 |
| Buzzer | GP21 (PWM) |
| Botão A | GP5 (pull-up interno) |

---

## ⚙️ Configuração

### 1. Credenciais Wi-Fi

Edite as linhas no topo de `laser_distance.c`:

```c
#define WIFI_SSID       "SEU_WIFI_AQUI"
#define WIFI_PASSWORD   "SUA_SENHA_AQUI"
```

### 2. MQTT

O broker usado é o **HiveMQ público** (sem autenticação, sem TLS):

```c
#define MQTT_BROKER     "broker.hivemq.com"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID  "bitdoglab_pico_001"  // deve ser único por dispositivo
```

> ⚠️ Se houver mais de um dispositivo na rede com o mesmo `MQTT_CLIENT_ID`, eles vão se desconectar mutuamente. Altere o ID conforme necessário.

---

## 📡 Tópicos MQTT

| Tópico | Conteúdo | Exemplo |
|---|---|---|
| `bitdoglab/distancia` | Distância em mm | `342` |
| `bitdoglab/inclinacao` | Inclinação em graus | `12.5` |
| `bitdoglab/alerta` | JSON com estado dos alertas | `{"colisao":0,"inclinacao":1}` |
| `bitdoglab/leds` | Estado dos LEDs | `ativo` ou `mute` |
| `bitdoglab/status` | Status do dispositivo | `online` / `offline` (LWT) |

### Monitorar no navegador

Acesse [hivemq.com/demos/websocket-client](http://www.hivemq.com/demos/websocket-client/), conecte ao broker `broker.hivemq.com` na porta `8000` (WebSocket) e inscreva-se nos tópicos `bitdoglab/#`.

---

## 🚨 Lógica de Alertas

| Condição | LED | Buzzer | Display |
|---|---|---|---|
| Normal | 🟢 Verde | Silencioso | — |
| Distância < 100mm | 🔴 Vermelho | Tom agudo (PWM 2000) | `! COLISAO !` |
| Inclinação > 70° ou < -70° | 🟠 Laranja | Tom médio (PWM 1000) | `! INCLINACAO !` |
| LEDs silenciados (botão A) | ⚫ Desligado | — | `MODO MUTE` |

---

## 🛠️ Compilação

### Pré-requisitos

- [Pico SDK 2.2.0](https://github.com/raspberrypi/pico-sdk)
- [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
- CMake 3.13+
- ARM GCC Toolchain 14.2

### Build

```bash
# Delete o cache do CMake antes de recompilar (importante após mudanças no lwipopts.h)
rm -rf build/

mkdir build && cd build
cmake ..
make -j4
```

O arquivo `laser_distance.uf2` será gerado em `build/`.

### Gravação

1. Segure o botão **BOOTSEL** da BitDogLab e conecte o USB
2. Arraste o arquivo `laser_distance.uf2` para a unidade `RPI-RP2` que aparecer
3. A placa reinicia automaticamente

### Monitor serial

```bash
# Linux/Mac
minicom -b 115200 -D /dev/ttyACM0

# Windows — use PuTTY ou o monitor serial do VS Code (115200 baud)
```

---

## 🐞 Solução de Problemas

| Sintoma | Causa provável | Solução |
|---|---|---|
| `*** PANIC *** size > 0` | `lwipopts.h` ausente ou incorreto | Confirme que `lwipopts.h` está na raiz do projeto |
| `MEMP_SYS_TIMEOUT is empty` | Poucos slots de timeout no lwIP | Verifique `MEMP_NUM_SYS_TIMEOUT 16` no `lwipopts.h` |
| LED não pisca, serial vazio | `cyw43_arch_init()` chamado fora do FreeRTOS | `cyw43_arch_init()` deve ser chamado dentro de uma task |
| Botão A não responde | — | O botão usa interrupção de GPIO; confirme que `gpio_set_irq_enabled_with_callback` foi chamado na init |
| MQTT não conecta | DNS falhou ou Wi-Fi instável | Verifique SSID/senha e aguarde as mensagens de reconexão no serial |
| `MQTT_PORT redefined` | Conflito com define interno do lwIP | Use `MQTT_BROKER_PORT` em vez de `MQTT_PORT` |

---

## 📦 Dependências

| Biblioteca | Versão | Uso |
|---|---|---|
| Pico SDK | 2.2.0 | Base do hardware |
| FreeRTOS Kernel | 202107+ | Multitasking |
| lwIP | (incluso no SDK) | TCP/IP + DNS + MQTT |
| pico_cyw43_arch_lwip_sys_freertos | (incluso no SDK) | Wi-Fi integrado ao FreeRTOS |

---

## 📝 Licença

Projeto acadêmico — EmbarcaTech. Livre para uso educacional.
