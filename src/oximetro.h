#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OXI_WAIT = 0,   // aguardando dedo
    OXI_SETTLE,     // calibrando baseline
    OXI_RUN,        // coletando batimentos e validando
    OXI_FROZEN      // fechou 20 leituras válidas (resultado final travado)
} oxi_state_t;

// Inicializa I2C1 em GP2/GP3 e detecta MAX30100/30102.
// Retorna false se não achar o dispositivo.
bool        oxi_init(void);

// Inicia uma nova medição (zera filtros e contadores).
void        oxi_start(void);

// Executa um passo da máquina de estados e leitura do sensor.
// - now_ms: timestamp em ms (use to_ms_since_boot(...))
// - bpm_display: valor filtrado para mostrar em tempo real (run)
// - valid_count: nº de leituras válidas acumuladas (0..20)
// - done: true quando atingir 20 válidas (resultado final fechado)
// - bpm_final: valor final quando done==true
// Retorna true se houve atualização de dados (opcional).
bool        oxi_poll(uint32_t now_ms, float *bpm_display,
                     int *valid_count, bool *done, float *bpm_final);

// Estado atual (para o orquestrador formatar as mensagens no OLED)
oxi_state_t oxi_get_state(void);

#ifdef __cplusplus
}
#endif
