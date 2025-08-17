// main.c — Fluxo com pergunta A/B, leitura opcional de cor e avanço para oxímetro (placeholder)
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "src/ssd1306.h"
#include "src/ssd1306_i2c.h"
#include "src/ssd1306_font.h"
#include "src/cor.h"         // cor_init, cor_read_rgb_norm, cor_classify, cor_class_to_str
// #include "src/oximetro.h" // 

#define OLED_I2C   i2c1
#define OLED_SDA   14
#define OLED_SCL   15
#define OLED_ADDR  0x3C

#define COL_I2C    i2c0
#define COL_SDA    0
#define COL_SCL    1

#define BUTTON_A   5
#define BUTTON_B   6

static ssd1306_t oled;
static bool oled_ok = false;

// contagem por classe de cor
static uint32_t g_cor_counts[COR_CLASS_COUNT] = {0};

// ---------- I2C / OLED helpers ----------
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

// ---------- Botões (edge + holdoff) ----------
static bool edge_press(bool now, bool *prev) {
    bool fired = (now && !*prev);
    *prev = now;
    return fired;
}

// ---------- Estados ----------
typedef enum {
    ST_ASK = 0,        // Pergunta inicial
    ST_COLOR_PREP,     // Mensagem de instrução para cor
    ST_COLOR_LOOP,     // Loop de leitura de cor (A captura)
    ST_OXI_PREP,       // Mensagem de transição para oxímetro
    ST_OXI_RUN         // (placeholder) Etapa do oxímetro
} state_t;

int main(void) {
    stdio_init_all();
    sleep_ms(300);

    // OLED em I2C1 (14/15)
    i2c_setup(OLED_I2C, OLED_SDA, OLED_SCL, 400000);
    oled.external_vcc = false;
    oled_ok = ssd1306_init(&oled, 128, 64, OLED_ADDR, OLED_I2C);

    // Botões
    gpio_init(BUTTON_A); gpio_set_dir(BUTTON_A, GPIO_IN); gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B); gpio_set_dir(BUTTON_B, GPIO_IN); gpio_pull_up(BUTTON_B);
    bool a_prev = false, b_prev = false;

    // Cor (inicializa só quando necessário)
    bool cor_ready = false;

    state_t st = ST_ASK;
    state_t last_st = -1;
    uint32_t t_last = 0;

    // holdoff para ignorar cliques enquanto o dedo ainda está em cima do botão
    uint32_t holdoff_until_ms = 0;

    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // Aplica holdoff (evita “travar” por clique longo + sleep)
        bool a_now_raw = !gpio_get(BUTTON_A); // ativo baixo
        bool b_now_raw = !gpio_get(BUTTON_B);
        bool a_now = (now_ms >= holdoff_until_ms) ? a_now_raw : false;
        bool b_now = (now_ms >= holdoff_until_ms) ? b_now_raw : false;

        bool a_edge = edge_press(a_now, &a_prev);
        bool b_edge = edge_press(b_now, &b_prev);

        // Desenha a tela só quando muda de estado
        if (st != last_st) {
            switch (st) {
                case ST_ASK:
                    oled_lines("Fazer leitura de cor?", "(A) Sim   (B) Nao", "", "");
                    break;
                case ST_COLOR_PREP:
                    oled_lines("Lendo Cor...", "Aperte A p/ capturar", "Posicione o cartao", "");
                    break;
                case ST_COLOR_LOOP:
                    // a tela é atualizada no loop (dinamica)
                    break;
                case ST_OXI_PREP:
                    oled_lines("Oximetro ativado", "Posicione o dedo", "Iniciando...", "");
                    break;
                case ST_OXI_RUN:
                    oled_lines("Oximetro (demo)", "Leitura em breve", "Press B p/ voltar", "");
                    break;
            }
            last_st = st;
        }

        switch (st) {
        case ST_ASK:
            if (a_edge) {
                // Inicia etapa de cor
                if (!cor_ready) {
                    i2c_setup(COL_I2C, COL_SDA, COL_SCL, 100000);
                    cor_ready = cor_init(COL_I2C, COL_SDA, COL_SCL);
                }
                if (!cor_ready) {
                    oled_lines("TCS34725 nao encontrado", "Pulando etapa de cor", "", "");
                    holdoff_until_ms = now_ms + 300;
                    sleep_ms(800);
                    st = ST_OXI_PREP;
                } else {
                    holdoff_until_ms = now_ms + 300;
                    st = ST_COLOR_PREP;
                }
            } else if (b_edge) {
                // Pula direto para oxímetro
                holdoff_until_ms = now_ms + 300;
                st = ST_OXI_PREP;
            }
            break;

        case ST_COLOR_PREP:
            // Pequena pausa visual
            if (now_ms - t_last > 250) {
                t_last = now_ms;
                st = ST_COLOR_LOOP;
            }
            break;

        case ST_COLOR_LOOP: {
            // Atualiza leitura ~5 Hz
            if (now_ms - t_last > 200) {
                t_last = now_ms;

                float rf, gf, bf, cf;
                if (cor_read_rgb_norm(&rf, &gf, &bf, &cf)) {
                    cor_class_t cls = cor_classify(rf, gf, bf, cf);
                    const char *nome = cor_class_to_str(cls);

                    char l3[24], l4[24];
                    // mostra também valores normalizados (0..255)
                    int R = (int)(rf + 0.5f);
                    int G = (int)(gf + 0.5f);
                    int B = (int)(bf + 0.5f);

                    snprintf(l3, sizeof l3, "Cor: %s", nome);
                    snprintf(l4, sizeof l4, "R%3d G%3d B%3d", R, G, B);
                    oled_lines("Lendo Cor...", "Aperte A p/ capturar", l3, l4);
                } else {
                    oled_lines("Lendo Cor...", "Aperte A p/ capturar", "Sem leitura", "");
                }
            }

            if (a_edge) {
                // Captura a cor atual e avança
                float rf, gf, bf, cf;
                if (cor_read_rgb_norm(&rf, &gf, &bf, &cf)) {
                    cor_class_t cls = cor_classify(rf, gf, bf, cf);
                    g_cor_counts[cls]++;
                    const char *nome = cor_class_to_str(cls);

                    char msg[26];
                    snprintf(msg, sizeof msg, "Cor %s captada!", nome);
                    oled_lines(msg, "", "", "");
                    holdoff_until_ms = now_ms + 350;  // desarma clique
                    sleep_ms(900);

                    st = ST_OXI_PREP;                 // **agora sempre avança**
                } else {
                    oled_lines("Falha na leitura", "Tente novamente", "", "");
                    holdoff_until_ms = now_ms + 300;
                    sleep_ms(800);
                    // permanece em ST_COLOR_LOOP
                }
            }
            break;
        }

        case ST_OXI_PREP:
            // Aqui faremos a inicialização real do oxímetro.
            // Por ora, apenas transição visual e segue para RUN (placeholder).
            if (now_ms - t_last > 250) {
                t_last = now_ms;
                holdoff_until_ms = now_ms + 300;
                st = ST_OXI_RUN;
            }
            break;

        case ST_OXI_RUN:
            // Placeholder do oxímetro: por enquanto só mostra tela
            // e permite voltar com B (para você não ficar preso)
            if (b_edge) {
                holdoff_until_ms = now_ms + 300;
                st = ST_ASK; // volta ao início
            }
            break;
        }

        sleep_ms(10);
    }
}
