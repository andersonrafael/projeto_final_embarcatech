#include "servo.h"

// Variáveis estáticas para guardar a configuração do PWM
static uint slice_num;
static uint channel_num;
static uint32_t wrap_value;

// Função auxiliar para converter ângulo em largura de pulso
static uint32_t angle_to_pulse_width(uint angle)
{
    // Mapeia o ângulo (0-180) para a faixa de pulso (MIN-MAX)
    uint32_t pulse = (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180) + SERVO_MIN_PULSE_US;
    return pulse;
}

// Inicializa o servo para ser usado nas funções principais
void servo_init()
{
    printf("[SERVO] Configurando Servo SG90 no pino GPIO %d...\n", SERVO_PIN);

    // 1. Associa o pino GPIO à função de hardware PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    // 2. Descobre qual "fatia" (slice) e "canal" de PWM o pino usa
    slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    channel_num = pwm_gpio_to_channel(SERVO_PIN);

    // 3. Configura o PWM
    pwm_config config = pwm_get_default_config();

    // Calcula o divisor do clock para ter um período de 20ms (50Hz)
    // Para simplificar, vamos fazer cada ciclo do PWM contar 1 microssegundo (us)
    float div = (float)clock_get_hz(clk_sys) / 1000000;
    pwm_config_set_clkdiv(&config, div);

    // Define o valor de "wrap" (o período total) para 20.000us = 20ms
    wrap_value = PWM_PERIOD_MS * 1000;
    pwm_config_set_wrap(&config, wrap_value);

    // 4. Carrega a configuração na fatia PWM e a ativa
    pwm_init(slice_num, &config, true);

    printf("[SERVO] Servo SG90 pronto.\n");
}

void servo_set_angle(uint angle)
{
    if (angle > 180)
    {
        angle = 180; // Limita o ângulo a 180
    }

    // Converte o ângulo para a largura do pulso e define o nível do canal PWM
    uint32_t pulse = angle_to_pulse_width(angle);
    pwm_set_chan_level(slice_num, channel_num, pulse);
}

// tete