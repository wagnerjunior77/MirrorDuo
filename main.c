// main.c — Fluxo inicial: pergunta A/B e captura opcional de cor
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "src/ssd1306.h"
#include "src/ssd1306_i2c.h"
#include "src/ssd1306_font.h"

// ==== OLED em I2C1 (BitDog) ====
#define OLED_I2C   i2c1
#define OLED_SDA   14
#define OLED_SCL   15
#define OLED_ADDR  0x3C

// ==== TCS34725 em I2C0 (BitDog) ====
#define COL_I2C    i2c0
#define COL_SDA    0
#define COL_SCL    1

// Botões BitDog
#define BUTTON_A   5
#define BUTTON_B   6

// --------- COR LIB ----------
#include "src/cor.h"  // cor_init(...), cor_read_rgb_norm(...), cor_classify(...), cor_class_to_str(...)
static uint32_t g_cor_counts[COR_CLASS_COUNT] = {0};  // buffer de contagem por classe

// --------- OLED helpers ----------
static ssd1306_t oled;
static bool oled_ok = false;

static void i2c_setup(i2c_inst_t *i2c, uint sda, uint scl, uint hz) {
    i2c_init(i2c, hz);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

static void oled_lines(const char *l1, const char *l2, const char *l3, const char *l4) {
    if (!oled_ok) return;
    ssd1306_clear(&oled);
    if (l1) ssd1306_draw_string(&oled, 0,  0, 1, l1);
    if (l2) ssd1306_draw_string(&oled, 0, 16, 1, l2);
    if (l3) ssd1306_draw_string(&oled, 0, 32, 1, l3);
    if (l4) ssd1306_draw_string(&oled, 0, 48, 1, l4);
    ssd1306_show(&oled);
}

// --------- botões (detecção de borda com polling simples) ----------
static bool edge_press(bool now, bool *prev) {
    bool fired = (now && !*prev);
    *prev = now;
    return fired;
}

// --------- estados ----------
typedef enum {
    ST_ASK,          // pergunta A/B
    ST_COLOR_PREP,   // msg de instrução
    ST_COLOR_LOOP,   // lendo cor ao vivo
    ST_SKIP_COLOR    // pulou etapa de cor
} state_t;

int main(void) {
    stdio_init_all();
    sleep_ms(300);

    // OLED
    i2c_setup(OLED_I2C, OLED_SDA, OLED_SCL, 400000);
    oled.external_vcc = false;
    oled_ok = ssd1306_init(&oled, 128, 64, OLED_ADDR, OLED_I2C);

    // Botões
    gpio_init(BUTTON_A); gpio_set_dir(BUTTON_A, GPIO_IN); gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B); gpio_set_dir(BUTTON_B, GPIO_IN); gpio_pull_up(BUTTON_B);
    bool a_prev = false, b_prev = false;

    // Cor (somente quando necessário; lazy-init)
    bool cor_ready = false;

    state_t st = ST_ASK;
    uint32_t t_last = 0;

    while (true) {
        bool a_now = !gpio_get(BUTTON_A); // ativo em nível baixo
        bool b_now = !gpio_get(BUTTON_B);
        bool a_edge = edge_press(a_now, &a_prev);
        bool b_edge = edge_press(b_now, &b_prev);

        switch (st) {
        case ST_ASK:
            oled_lines("Fazer leitura de cor?", "(A) Sim   (B) Nao", "", "");
            if (a_edge) {
                // init do sensor de cor (I2C0) só aqui
                if (!cor_ready) {
                    i2c_setup(COL_I2C, COL_SDA, COL_SCL, 100000);
                    cor_ready = cor_init(COL_I2C, COL_SDA, COL_SCL);
                }
                if (!cor_ready) {
                    oled_lines("TCS34725 Nao encontrado", "Pulando etapa de cor", "", "");
                    sleep_ms(1500);
                    st = ST_SKIP_COLOR;
                } else {
                    st = ST_COLOR_PREP;
                }
            } else if (b_edge) {
                st = ST_SKIP_COLOR;
            }
            break;

        case ST_COLOR_PREP:
            oled_lines("Lendo Cor...", "Aperte A p/ capturar", "Posicione o cartao", "");
            t_last = to_ms_since_boot(get_absolute_time());
            st = ST_COLOR_LOOP;
            break;

        case ST_COLOR_LOOP: {
            // atualiza leitura/rotulo ~5 Hz
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - t_last > 200) {
                t_last = now;

                float r = 0.f, g = 0.f, b = 0.f, c = 0.f;
                if (cor_read_rgb_norm(&r, &g, &b, &c)) {
                    cor_class_t cls = cor_classify(r, g, b, c);
                    const char *nome = cor_class_to_str(cls);

                    // r,g,b são normalizados (0..~1). Convertemos para 0..255 apenas para exibir.
                    int R = (int)(r * 255.f + 0.5f);
                    int G = (int)(g * 255.f + 0.5f);
                    int B = (int)(b * 255.f + 0.5f);

                    char l3[24], l4[24];
                    snprintf(l3, sizeof l3, "Cor: %s", nome);
                    snprintf(l4, sizeof l4, "R%3d G%3d B%3d", R, G, B);
                    oled_lines("Lendo Cor...", "Aperte A p/ capturar", l3, l4);
                } else {
                    oled_lines("Lendo Cor...", "Aperte A p/ capturar", "Sem leitura", "");
                }
            }

            if (a_edge) {
                // captura a cor atual
                float r = 0.f, g = 0.f, b = 0.f, c = 0.f;
                if (cor_read_rgb_norm(&r, &g, &b, &c)) {
                    cor_class_t cls = cor_classify(r, g, b, c);
                    g_cor_counts[cls]++;
                    const char *nome = cor_class_to_str(cls);

                    char msg[26];
                    snprintf(msg, sizeof msg, "Cor %s captada!", nome);
                    oled_lines(msg, "", "", "");
                    sleep_ms(1200);

                    // volta para a pergunta (por enquanto)
                    st = ST_ASK;
                } else {
                    oled_lines("Falha na leitura", "Tente novamente", "", "");
                    sleep_ms(1200);
                }
            }
            break;
        }

        case ST_SKIP_COLOR:
            oled_lines("Pulando etapa de cor", "Prosseguiremos apos", "integracao do oxi.", "");
            sleep_ms(1500);
            // por enquanto volta para a pergunta
            st = ST_ASK;
            break;
        }

        sleep_ms(10);
    }
}
