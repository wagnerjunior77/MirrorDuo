#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// OLED (usa sua lib)
#include "src/ssd1306.h"
#include "src/ssd1306_i2c.h"
#include "src/ssd1306_font.h"

/* ===================== Pines/Interfaces ===================== */
// OLED em I2C1 (BitDogLab)
#define OLED_I2C          i2c1
#define OLED_SDA          14
#define OLED_SCL          15
#define OLED_ADDR         0x3C
#define OLED_W            128
#define OLED_H             64

// TCS34725 no I2C0 (BitDogLab)
#define TCS_I2C           i2c0
#define TCS_SDA           0
#define TCS_SCL           1
#define TCS_ADDR          0x29

// Botões
#define BTN_A             5
#define BTN_B             6

/* ===================== OLED helpers ===================== */
static ssd1306_t oled;

static void oled_init(void) {
    i2c_init(OLED_I2C, 400000);
    gpio_set_function(OLED_SDA, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA);
    gpio_pull_up(OLED_SCL);

    oled.external_vcc = false;
    if (!ssd1306_init(&oled, OLED_W, OLED_H, OLED_ADDR, OLED_I2C)) {
        while (1) { /* erro fatal */ }
    }
    ssd1306_clear(&oled);
    ssd1306_show(&oled);
}

static void oled_msg2(const char *l1, const char *l2) {
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 8,  1, l1);
    ssd1306_draw_string(&oled, 0, 24, 1, l2);
    ssd1306_show(&oled);
}

static void oled_menu_pergunta(void) {
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 0,  1, "Leitura de cor?");
    ssd1306_draw_string(&oled, 0, 20, 1, "(A) Sim");
    ssd1306_draw_string(&oled, 0, 36, 1, "(B) Nao");
    ssd1306_show(&oled);
}

/* ===================== Botões (polling simples) ===================== */
static void buttons_init(void) {
    gpio_init(BTN_A); gpio_set_dir(BTN_A, GPIO_IN); gpio_pull_up(BTN_A);
    gpio_init(BTN_B); gpio_set_dir(BTN_B, GPIO_IN); gpio_pull_up(BTN_B);
}
static inline bool btnA_pressed(void) { return !gpio_get(BTN_A); }
static inline bool btnB_pressed(void) { return !gpio_get(BTN_B); }
static void wait_release_all(void){
    // espera “despressionar” (debounce simples)
    sleep_ms(20);
    while (!gpio_get(BTN_A) || !gpio_get(BTN_B)) { sleep_ms(10); }
}

/* ===================== TCS34725 – driver mínimo ===================== */
// Registros (comando sempre OR 0x80)
#define TCS_CMD(reg)          ((reg) | 0x80)
#define TCS_REG_ENABLE        0x00
#define TCS_REG_ATIME         0x01
#define TCS_REG_CONTROL       0x0F
#define TCS_REG_ID            0x12
#define TCS_REG_CDATAL        0x14  // C, R, G, B: LSB..MSB em pares

// ENABLE bits
#define TCS_EN_PON            0x01
#define TCS_EN_AEN            0x02

static bool tcs_w8(uint8_t reg, uint8_t v) {
    uint8_t b[2] = { TCS_CMD(reg), v };
    return i2c_write_blocking(TCS_I2C, TCS_ADDR, b, 2, false) == 2;
}
static bool tcs_rn(uint8_t reg, uint8_t *dst, size_t n) {
    uint8_t r = TCS_CMD(reg);
    if (i2c_write_blocking(TCS_I2C, TCS_ADDR, &r, 1, true) != 1) return false;
    return i2c_read_blocking(TCS_I2C, TCS_ADDR, dst, n, false) == (int)n;
}
static bool tcs_read_colors(uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b) {
    uint8_t d[8];
    if (!tcs_rn(TCS_REG_CDATAL, d, 8)) return false;
    *c = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
    *r = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
    *g = (uint16_t)d[4] | ((uint16_t)d[5] << 8);
    *b = (uint16_t)d[6] | ((uint16_t)d[7] << 8);
    return true;
}
static bool tcs_init(void) {
    // I2C0 setup
    i2c_init(TCS_I2C, 100000);
    gpio_set_function(TCS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TCS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TCS_SDA);
    gpio_pull_up(TCS_SCL);

    // Teste simples de presença (ID 0x44/0x4D geralmente)
    uint8_t id = 0;
    if (!tcs_rn(TCS_REG_ID, &id, 1)) return false;

    // Integração e ganho (ATIME=0xC0 ~ 160ms; GAIN=16x)
    tcs_w8(TCS_REG_ATIME,   0xC0);
    tcs_w8(TCS_REG_CONTROL, 0x10);  // 16x

    // Liga (PON) e ADC (AEN)
    tcs_w8(TCS_REG_ENABLE, TCS_EN_PON);
    sleep_ms(3);
    tcs_w8(TCS_REG_ENABLE, TCS_EN_PON | TCS_EN_AEN);
    sleep_ms(3);
    return true;
}

/* ===================== Classificador de cor ===================== */
typedef enum {
    COL_VERMELHO, COL_VERDE, COL_AZUL,
    COL_AMARELO, COL_LARANJA, COL_ROXO,
    COL_CIANO, COL_BRANCO, COL_PRETO,
    COL_DESCONHECIDA,
    COL_COUNT
} color_t;

static const char* color_name(color_t c){
    switch(c){
        case COL_VERMELHO:  return "VERMELHO";
        case COL_VERDE:     return "VERDE";
        case COL_AZUL:      return "AZUL";
        case COL_AMARELO:   return "AMARELO";
        case COL_LARANJA:   return "LARANJA";
        case COL_ROXO:      return "ROXO";
        case COL_CIANO:     return "CIANO";
        case COL_BRANCO:    return "BRANCO";
        case COL_PRETO:     return "PRETO";
        default:            return "DESCONH.";
    }
}

static color_t classify_rgb(uint16_t c, uint16_t r, uint16_t g, uint16_t b) {
    // brilho baixo -> PRETO
    if (c < 80) return COL_PRETO;
    uint32_t sum = (uint32_t)r + g + b;
    if (sum == 0) return COL_PRETO;

    float rn = (float)r / (float)sum;
    float gn = (float)g / (float)sum;
    float bn = (float)b / (float)sum;

    // branco se claro e balanceado
    if (c > 1200 && rn > 0.28f && gn > 0.28f && bn > 0.28f) return COL_BRANCO;

    if (rn > 0.42f && gn < 0.35f && bn < 0.35f) return COL_VERMELHO;
    if (gn > 0.42f && rn < 0.35f && bn < 0.35f) return COL_VERDE;
    if (bn > 0.42f && rn < 0.35f && gn < 0.35f) return COL_AZUL;

    if (rn > 0.36f && gn > 0.36f && bn < 0.26f) return COL_AMARELO;
    if (rn > 0.36f && bn > 0.36f && gn < 0.26f) return COL_ROXO;
    if (gn > 0.36f && bn > 0.36f && rn < 0.26f) return COL_CIANO;

    // laranja (entre vermelho e amarelo)
    if (rn > 0.40f && gn > 0.30f && bn < 0.20f) return COL_LARANJA;

    return COL_DESCONHECIDA;
}

/* ===== Buffer de ocorrências (contagem por cor) ===== */
static uint32_t color_counts[COL_COUNT] = {0};

/* ===================== Fluxo de captura de cor ===================== */
static void color_capture_flow(void) {
    // Inicializa TCS
    oled_msg2("Inicializando", "sensor de cor...");
    if (!tcs_init()) {
        oled_msg2("TCS34725", "nao encontrado!");
        sleep_ms(1500);
        return;
    }

    // Tela de leitura contínua
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 0, 1, "Lendo Cor...");
    ssd1306_draw_string(&oled, 0, 12,1, "A: confirmar cor");
    ssd1306_draw_string(&oled, 0, 24,1, "B: cancelar");
    ssd1306_show(&oled);

    // Laço de leitura em tempo real até A (confirma) ou B (cancela)
    color_t current = COL_DESCONHECIDA;
    absolute_time_t last_refresh = get_absolute_time();

    while (true) {
        uint16_t c, r, g, b;
        if (tcs_read_colors(&c, &r, &g, &b)) {
            current = classify_rgb(c, r, g, b);
        }

        // Atualiza display ~10 Hz
        if (absolute_time_diff_us(last_refresh, get_absolute_time()) > 100000) {
            last_refresh = get_absolute_time();
            char line1[24], line2[24], line3[24];
            snprintf(line1, sizeof line1, "Cor: %s", color_name(current));
            snprintf(line2, sizeof line2, "R=%4u G=%4u", r, g);
            snprintf(line3, sizeof line3, "B=%4u C=%4u", b, c);

            ssd1306_clear(&oled);
            ssd1306_draw_string(&oled, 0, 0,  1, "Lendo Cor...");
            ssd1306_draw_string(&oled, 0, 12, 1, "A: confirmar cor");
            ssd1306_draw_string(&oled, 0, 24, 1, "B: cancelar");
            ssd1306_draw_string(&oled, 0, 40, 1, line1);
            ssd1306_draw_string(&oled, 0, 52, 1, line2);
            // como a tela e' pequena, alterna linhas 52/40 se quiser
            // aqui só mais uma linha pequena:
            // (se quiser mostrar os 2, troque fontes/linhas)
            ssd1306_show(&oled);
        }

        // Confirma com A
        if (btnA_pressed()) {
            wait_release_all();
            color_counts[current]++;
            char ok[24];
            snprintf(ok, sizeof ok, "Cor %s captada!", color_name(current));
            oled_msg2(ok, "");
            sleep_ms(1200);
            return;
        }
        // Cancela com B
        if (btnB_pressed()) {
            wait_release_all();
            oled_msg2("Leitura cancelada", "");
            sleep_ms(900);
            return;
        }
        sleep_ms(5);
    }
}

/* ===================== MAIN ===================== */
int main(void) {
    stdio_init_all();
    sleep_ms(300);
    oled_init();
    buttons_init();

    while (true) {
        // Pergunta inicial
        oled_menu_pergunta();

        // Espera resposta
        while (true) {
            if (btnA_pressed()) { // SIM
                wait_release_all();
                color_capture_flow();
                break; // volta ao menu após capturar
            }
            if (btnB_pressed()) { // NAO
                wait_release_all();
                oled_msg2("Leitura de cor", "ignorada.");
                sleep_ms(900);
                break; // volta ao menu
            }
            sleep_ms(10);
        }
    }
    return 0;
}
