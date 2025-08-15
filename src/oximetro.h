#pragma once
#include <stdbool.h>
#include <stdint.h>

// Inicializa MAX30100/30102 em I2C1 (GP2 SDA, GP3 SCL). Retorna false se não achar.
bool oxi_init(void);

// Reinicia estado interno de medição
void oxi_reset_session(void);

// Passo de leitura/estimativa.
// - target_valid: número de leituras válidas desejadas (ex.: 20)
// Retorna:
//  - *bpm_progress: BPM filtrado (se disponível, >0) para exibir progresso
//  - *valid_count: quantas leituras válidas já acumulamos
//  - *done: true quando atingiu target_valid e travou o resultado
//  - *bpm_final: preenchido quando done=true
// Retorna false se leitura I2C falhar (bus transient etc.)
bool oxi_step(int target_valid, float *bpm_progress, int *valid_count, bool *done, float *bpm_final);
