#ifndef AD9106_H
#define AD9106_H

#include <stdint.h>

typedef enum
{
    AD9106_WAVE_SINE = 0,
    AD9106_WAVE_TRIANGLE,
    AD9106_WAVE_SAW,
    AD9106_WAVE_DC
} AD9106_Waveform_t;

typedef struct
{
    uint8_t output;             /* 1..4 */
    AD9106_Waveform_t waveform; /* sine / triangle / saw / dc */
    uint32_t freq_hz;           /* requested output frequency */
    float phase_deg;            /* 0..360 nominal */
    float gain;                 /* around -2.0 .. +2.0 */
    int32_t offset;             /* signed 12-bit style code */
} AD9106_Config_t;

void AD9106_Init(void);
void AD9106_Reset(void);

void AD9106_WriteReg(uint16_t reg, uint16_t data);
uint16_t AD9106_ReadReg(uint16_t reg);

void AD9106_Update(void);
void AD9106_Run(uint8_t enable);

void AD9106_GetDefaultConfig(AD9106_Config_t *cfg);
void AD9106_ApplyConfig(const AD9106_Config_t *cfg);

#endif