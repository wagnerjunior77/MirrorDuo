#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// OLED driver (pasta src/)
#include "src/ssd1306.h"
#include "src/ssd1306_i2c.h"
#include "src/ssd1306_font.h"

// ==== Pinos da BitDog ====
#define OLED_I2C       i2c1
#define OLED_SDA_PIN   14   // conector GND VCC SCL SDA ao lado do display
#define OLED_SCL_PIN   15
#define OLED_ADDR      0x3C

#define BTN_A          5    // Botão A (lado esquerdo)
#define BTN_B          6    // Botão B (lado direito)

// ==== OLED ====
static ssd1306_t oled;

static void oled_setup(void) {
    // I2C a 400 kHz para o OLED
    i2c_init(OLED_I2C, 400000);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    oled.external_vcc = false;
    // 128x64, endereço 0x3C, I2C1
    if (!ssd1306_init(&oled, 128, 64, OLED_ADDR, OLED_I2C)) {
        // travar se falhar
        while (true) tight_loop_contents();
    }
    ssd1306_clear(&oled);
    ssd1306_show(&oled);
}

static void draw_center(ssd1306_t *d, int y, const char *txt) {
    // fonte 5x8 + 1px de espaçamento → largura aprox = (5+1)*len
    int len = 0; while (txt[len] != '\0') len++;
    int w = len * 6;
    int x = (128 - w) / 2;
    if (x < 0) x = 0;
    ssd1306_draw_string(d, x, y, 1, txt);
}

int main(void) {
    stdio_init_all();
    sleep_ms(200); // tempo p/ USB subir (caso queira debug via serial)

    // botões com pull-up interno (ativos em nível baixo)
    gpio_init(BTN_A); gpio_set_dir(BTN_A, GPIO_IN); gpio_pull_up(BTN_A);
    gpio_init(BTN_B); gpio_set_dir(BTN_B, GPIO_IN); gpio_pull_up(BTN_B);

    oled_setup();

    // Tela da pergunta
    ssd1306_clear(&oled);
    draw_center(&oled, 4,  "Leitura de COR?");
    draw_center(&oled, 20, "(A = Sim)");
    draw_center(&oled, 32, "(B = Nao)");
    draw_center(&oled, 50, "Escolha um botao");
    ssd1306_show(&oled);

    // Espera escolha
    bool color_required = false;
    while (true) {
        if (!gpio_get(BTN_A)) {           // A pressionado (nível 0)
            color_required = true;
            break;
        }
        if (!gpio_get(BTN_B)) {           // B pressionado
            color_required = false;
            break;
        }
        sleep_ms(10);
    }
    // Debounce simples
    sleep_ms(200);

    // Confirmação no display
    ssd1306_clear(&oled);
    if (color_required) {
        draw_center(&oled, 20, "Opcao: SIM");
        draw_center(&oled, 36, "Ler cor depois");
    } else {
        draw_center(&oled, 20, "Opcao: NAO");
        draw_center(&oled, 36, "Pular leitura de cor");
    }
    ssd1306_show(&oled);

    // >>> Próximos passos:
    // - se color_required == true: chamar rotina da cor (loop até confirmar com A)
    // - depois: iniciar rotina do oxímetro e mostrar BPM no OLED
    // - por fim: perguntar se deseja recomeçar (A/B)
    while (true) { tight_loop_contents(); }
    return 0;
}
