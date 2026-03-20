#include "ad9106.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "stm32l4xx_hal.h"

static uint32_t g_dac_clock_hz = 156250000UL;

/* Register addresses */
#define AD9106_REG_RAMUPDATE 0x001D
#define AD9106_REG_PAT_STATUS 0x001E
#define AD9106_REG_PAT_TYPE 0x001F
#define AD9106_REG_PATTERN_DLY 0x0020

#define AD9106_REG_DAC4_DOF 0x0022
#define AD9106_REG_DAC3_DOF 0x0023
#define AD9106_REG_DAC2_DOF 0x0024
#define AD9106_REG_DAC1_DOF 0x0025

#define AD9106_REG_WAV4_3CONFIG 0x0026
#define AD9106_REG_WAV2_1CONFIG 0x0027
#define AD9106_REG_PAT_TIMEBASE 0x0028
#define AD9106_REG_PAT_PERIOD 0x0029

#define AD9106_REG_DAC4_CST 0x002E
#define AD9106_REG_DAC3_CST 0x002F
#define AD9106_REG_DAC2_CST 0x0030
#define AD9106_REG_DAC1_CST 0x0031

#define AD9106_REG_DAC4_DGAIN 0x0032
#define AD9106_REG_DAC3_DGAIN 0x0033
#define AD9106_REG_DAC2_DGAIN 0x0034
#define AD9106_REG_DAC1_DGAIN 0x0035

#define AD9106_REG_SAW4_3CONFIG 0x0036
#define AD9106_REG_SAW2_1CONFIG 0x0037

#define AD9106_REG_DDS_TW32 0x003E
#define AD9106_REG_DDS_TW1 0x003F
#define AD9106_REG_DDS4_PW 0x0040
#define AD9106_REG_DDS3_PW 0x0041
#define AD9106_REG_DDS2_PW 0x0042
#define AD9106_REG_DDS1_PW 0x0043

/* WAVx_yCONFIG field values */
#define AD9106_PRESTORE_CONST 0x0U
#define AD9106_PRESTORE_SAW 0x1U
#define AD9106_PRESTORE_PN 0x2U
#define AD9106_PRESTORE_DDS 0x3U

#define AD9106_WAVESEL_RAM 0x0U
#define AD9106_WAVESEL_PRESTORE 0x1U
#define AD9106_WAVESEL_DELAYED 0x2U
#define AD9106_WAVESEL_RAM_MOD 0x3U

/* SAW types */
#define AD9106_SAW_RAMP_UP 0x0U
#define AD9106_SAW_RAMP_DOWN 0x1U
#define AD9106_SAW_TRIANGLE 0x2U
#define AD9106_SAW_ZERO 0x3U

static uint16_t AD9106_GetGainReg(uint8_t output);
static uint16_t AD9106_GetOffsetReg(uint8_t output);
static uint16_t AD9106_GetConstReg(uint8_t output);
static uint16_t AD9106_GetPhaseReg(uint8_t output);
static void AD9106_SetCommonDefaults(void);
static void AD9106_SetFrequencyHz_DDS(uint32_t freq_hz);
static void AD9106_SetSawFrequency(uint8_t output, AD9106_Waveform_t wave, uint32_t freq_hz);
static void AD9106_SetPhaseDeg(uint8_t output, float phase_deg);
static void AD9106_SetGain(uint8_t output, float gain);
static void AD9106_SetOffset(uint8_t output, int32_t offset);
static void AD9106_SetConstMidscale(uint8_t output);
static void AD9106_SetWaveConfig(uint8_t output, uint8_t prestore_sel, uint8_t wave_sel);
static void AD9106_SetSawConfig(uint8_t output, uint8_t saw_type, uint8_t saw_step);
static uint16_t AD9106_EncodeSigned12Shift4(int32_t value);
static uint16_t AD9106_EncodeUnsigned12Shift4(uint16_t value);

void AD9106_Init(void)
{
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

void AD9106_Reset(void)
{
    HAL_GPIO_WritePin(FUNC_RESET_GPIO_Port, FUNC_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(FUNC_RESET_GPIO_Port, FUNC_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(FUNC_RESET_GPIO_Port, FUNC_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

void AD9106_WriteReg(uint16_t reg, uint16_t data)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)((reg >> 8) & 0x7F);
    buf[1] = (uint8_t)(reg & 0xFF);
    buf[2] = (uint8_t)(data >> 8);
    buf[3] = (uint8_t)(data & 0xFF);

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, buf, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

uint16_t AD9106_ReadReg(uint16_t reg)
{
    uint8_t tx[4];
    uint8_t rx[4];

    tx[0] = (uint8_t)(((reg >> 8) & 0x7F) | 0x80);
    tx[1] = (uint8_t)(reg & 0xFF);
    tx[2] = 0x00;
    tx[3] = 0x00;

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    return (uint16_t)(((uint16_t)rx[2] << 8) | rx[3]);
}

void AD9106_Update(void)
{
    AD9106_WriteReg(AD9106_REG_RAMUPDATE, 0x0001);
}

void AD9106_Run(uint8_t enable)
{
    AD9106_WriteReg(AD9106_REG_PAT_STATUS, enable ? 0x0001 : 0x0000);
    AD9106_Update();
}

void AD9106_GetDefaultConfig(AD9106_Config_t *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    cfg->output = 3;
    cfg->waveform = AD9106_WAVE_SINE;
    cfg->freq_hz = 1000;
    cfg->phase_deg = 0.0f;
    cfg->gain = 1.0f;
    cfg->offset = 0;
}

void AD9106_ApplyConfig(const AD9106_Config_t *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    AD9106_SetCommonDefaults();
    AD9106_SetConstMidscale(cfg->output);
    AD9106_SetGain(cfg->output, cfg->gain);
    AD9106_SetOffset(cfg->output, cfg->offset);
    AD9106_SetPhaseDeg(cfg->output, cfg->phase_deg);

    switch (cfg->waveform)
    {
    case AD9106_WAVE_SINE:
        AD9106_SetWaveConfig(cfg->output, AD9106_PRESTORE_DDS, AD9106_WAVESEL_PRESTORE);
        AD9106_SetFrequencyHz_DDS(cfg->freq_hz);
        break;

    case AD9106_WAVE_TRIANGLE:
        AD9106_SetWaveConfig(cfg->output, AD9106_PRESTORE_SAW, AD9106_WAVESEL_PRESTORE);
        AD9106_SetSawFrequency(cfg->output, cfg->waveform, cfg->freq_hz);
        break;

    case AD9106_WAVE_SAW:
        AD9106_SetWaveConfig(cfg->output, AD9106_PRESTORE_SAW, AD9106_WAVESEL_PRESTORE);
        AD9106_SetSawFrequency(cfg->output, cfg->waveform, cfg->freq_hz);
        break;

    case AD9106_WAVE_DC:
    default:
        AD9106_SetWaveConfig(cfg->output, AD9106_PRESTORE_CONST, AD9106_WAVESEL_PRESTORE);
        break;
    }

    AD9106_Update();
}

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

static uint16_t AD9106_GetGainReg(uint8_t output)
{
    switch (output)
    {
    case 1:
        return AD9106_REG_DAC1_DGAIN;
    case 2:
        return AD9106_REG_DAC2_DGAIN;
    case 3:
        return AD9106_REG_DAC3_DGAIN;
    case 4:
        return AD9106_REG_DAC4_DGAIN;
    default:
        return 0;
    }
}

static uint16_t AD9106_GetOffsetReg(uint8_t output)
{
    switch (output)
    {
    case 1:
        return AD9106_REG_DAC1_DOF;
    case 2:
        return AD9106_REG_DAC2_DOF;
    case 3:
        return AD9106_REG_DAC3_DOF;
    case 4:
        return AD9106_REG_DAC4_DOF;
    default:
        return 0;
    }
}

static uint16_t AD9106_GetConstReg(uint8_t output)
{
    switch (output)
    {
    case 1:
        return AD9106_REG_DAC1_CST;
    case 2:
        return AD9106_REG_DAC2_CST;
    case 3:
        return AD9106_REG_DAC3_CST;
    case 4:
        return AD9106_REG_DAC4_CST;
    default:
        return 0;
    }
}

static uint16_t AD9106_GetPhaseReg(uint8_t output)
{
    switch (output)
    {
    case 1:
        return AD9106_REG_DDS1_PW;
    case 2:
        return AD9106_REG_DDS2_PW;
    case 3:
        return AD9106_REG_DDS3_PW;
    case 4:
        return AD9106_REG_DDS4_PW;
    default:
        return 0;
    }
}

static void AD9106_SetCommonDefaults(void)
{
    /* Keep these at sensible defaults. */
    AD9106_WriteReg(AD9106_REG_PAT_TYPE, 0x0000);
    AD9106_WriteReg(AD9106_REG_PATTERN_DLY, 0x000E);
    AD9106_WriteReg(AD9106_REG_PAT_TIMEBASE, 0x0111);
    AD9106_WriteReg(AD9106_REG_PAT_PERIOD, 0xFFFF);
}

static void AD9106_SetFrequencyHz_DDS(uint32_t freq_hz)
{
    uint32_t tw;
    uint16_t tw_msb;
    uint16_t tw_lsb;

    if (freq_hz == 0U)
    {
        freq_hz = 1U;
    }

    tw = (uint32_t)((((uint64_t)freq_hz) << 24) / g_dac_clock_hz);

    tw_msb = (uint16_t)((tw >> 8) & 0xFFFFU);
    tw_lsb = (uint16_t)((tw & 0xFFU) << 8);

    AD9106_WriteReg(AD9106_REG_DDS_TW32, tw_msb);
    AD9106_WriteReg(AD9106_REG_DDS_TW1, tw_lsb);
}

static void AD9106_SetSawFrequency(uint8_t output, AD9106_Waveform_t wave, uint32_t freq_hz)
{
    uint32_t denom_factor;
    uint32_t step;
    uint8_t saw_type;

    if (freq_hz == 0U)
    {
        freq_hz = 1U;
    }

    if (wave == AD9106_WAVE_TRIANGLE)
    {
        denom_factor = 2U;
        saw_type = AD9106_SAW_TRIANGLE;
    }
    else
    {
        denom_factor = 1U;
        saw_type = AD9106_SAW_RAMP_UP;
    }

    /* step ~= fclk / (freq * N * 2^14) */
    step = (uint32_t)(g_dac_clock_hz / (freq_hz * denom_factor * 16384UL));

    if (step < 1U)
    {
        step = 1U;
    }
    if (step > 63U)
    {
        step = 63U;
    }

    AD9106_SetSawConfig(output, saw_type, (uint8_t)step);
}

static void AD9106_SetPhaseDeg(uint8_t output, float phase_deg)
{
    uint16_t reg;
    uint32_t phase_word;

    reg = AD9106_GetPhaseReg(output);
    if (reg == 0U)
    {
        return;
    }

    while (phase_deg < 0.0f)
    {
        phase_deg += 360.0f;
    }
    while (phase_deg >= 360.0f)
    {
        phase_deg -= 360.0f;
    }

    phase_word = (uint32_t)((phase_deg * 65536.0f) / 360.0f);
    AD9106_WriteReg(reg, (uint16_t)phase_word);
}

static void AD9106_SetGain(uint8_t output, float gain)
{
    int32_t code;
    uint16_t reg;

    reg = AD9106_GetGainReg(output);
    if (reg == 0U)
    {
        return;
    }

    if (gain > 1.999f)
    {
        gain = 1.999f;
    }
    if (gain < -2.0f)
    {
        gain = -2.0f;
    }

    if (gain >= 0.0f)
    {
        code = (int32_t)(gain * 1024.0f + 0.5f);
    }
    else
    {
        code = (int32_t)(gain * 1024.0f - 0.5f);
    }

    if (code > 2047)
    {
        code = 2047;
    }
    if (code < -2048)
    {
        code = -2048;
    }

    AD9106_WriteReg(reg, AD9106_EncodeSigned12Shift4(code));
}

static void AD9106_SetOffset(uint8_t output, int32_t offset)
{
    uint16_t reg;

    reg = AD9106_GetOffsetReg(output);
    if (reg == 0U)
    {
        return;
    }

    if (offset > 2047)
    {
        offset = 2047;
    }
    if (offset < -2048)
    {
        offset = -2048;
    }

    AD9106_WriteReg(reg, AD9106_EncodeSigned12Shift4(offset));
}

static void AD9106_SetConstMidscale(uint8_t output)
{
    uint16_t reg;

    reg = AD9106_GetConstReg(output);
    if (reg == 0U)
    {
        return;
    }

    AD9106_WriteReg(reg, AD9106_EncodeUnsigned12Shift4(0x0800U));
}

static void AD9106_SetWaveConfig(uint8_t output, uint8_t prestore_sel, uint8_t wave_sel)
{
    uint16_t reg_addr;
    uint16_t reg_val;

    if ((output == 3U) || (output == 4U))
    {
        reg_addr = AD9106_REG_WAV4_3CONFIG;
    }
    else
    {
        reg_addr = AD9106_REG_WAV2_1CONFIG;
    }

    reg_val = AD9106_ReadReg(reg_addr);

    switch (output)
    {
    case 4:
        reg_val &= (uint16_t)~((uint16_t)(0x3U << 12) | (uint16_t)(0x3U << 8));
        reg_val |= (uint16_t)(((uint16_t)(prestore_sel & 0x3U) << 12) |
                              ((uint16_t)(wave_sel & 0x3U) << 8));
        break;

    case 3:
        reg_val &= (uint16_t)~((uint16_t)(0x3U << 4) | (uint16_t)(0x3U << 0));
        reg_val |= (uint16_t)(((uint16_t)(prestore_sel & 0x3U) << 4) |
                              ((uint16_t)(wave_sel & 0x3U) << 0));
        break;

    case 2:
        reg_val &= (uint16_t)~((uint16_t)(0x3U << 12) | (uint16_t)(0x3U << 8));
        reg_val &= (uint16_t)~((uint16_t)(1U << 11) | (uint16_t)(1U << 10));
        reg_val |= (uint16_t)(((uint16_t)(prestore_sel & 0x3U) << 12) |
                              ((uint16_t)(wave_sel & 0x3U) << 8));
        break;

    case 1:
        reg_val &= (uint16_t)~((uint16_t)(0x3U << 4) | (uint16_t)(0x3U << 0));
        reg_val &= (uint16_t)~((uint16_t)(1U << 3) | (uint16_t)(1U << 2));
        reg_val |= (uint16_t)(((uint16_t)(prestore_sel & 0x3U) << 4) |
                              ((uint16_t)(wave_sel & 0x3U) << 0));
        break;

    default:
        return;
    }

    AD9106_WriteReg(reg_addr, reg_val);
}

static void AD9106_SetSawConfig(uint8_t output, uint8_t saw_type, uint8_t saw_step)
{
    uint16_t reg_addr;
    uint16_t reg_val;

    if (saw_step < 1U)
    {
        saw_step = 1U;
    }
    if (saw_step > 63U)
    {
        saw_step = 63U;
    }

    if ((output == 3U) || (output == 4U))
    {
        reg_addr = AD9106_REG_SAW4_3CONFIG;
    }
    else
    {
        reg_addr = AD9106_REG_SAW2_1CONFIG;
    }

    reg_val = AD9106_ReadReg(reg_addr);

    switch (output)
    {
    case 4:
        reg_val &= (uint16_t)~((uint16_t)(0x3FU << 10) | (uint16_t)(0x3U << 8));
        reg_val |= (uint16_t)(((uint16_t)(saw_step & 0x3FU) << 10) |
                              ((uint16_t)(saw_type & 0x3U) << 8));
        break;

    case 3:
        reg_val &= (uint16_t)~((uint16_t)(0x3FU << 2) | (uint16_t)(0x3U << 0));
        reg_val |= (uint16_t)(((uint16_t)(saw_step & 0x3FU) << 2) |
                              ((uint16_t)(saw_type & 0x3U) << 0));
        break;

    case 2:
        reg_val &= (uint16_t)~((uint16_t)(0x3FU << 10) | (uint16_t)(0x3U << 8));
        reg_val |= (uint16_t)(((uint16_t)(saw_step & 0x3FU) << 10) |
                              ((uint16_t)(saw_type & 0x3U) << 8));
        break;

    case 1:
        reg_val &= (uint16_t)~((uint16_t)(0x3FU << 2) | (uint16_t)(0x3U << 0));
        reg_val |= (uint16_t)(((uint16_t)(saw_step & 0x3FU) << 2) |
                              ((uint16_t)(saw_type & 0x3U) << 0));
        break;

    default:
        return;
    }

    AD9106_WriteReg(reg_addr, reg_val);
}

static uint16_t AD9106_EncodeSigned12Shift4(int32_t value)
{
    uint16_t raw12 = (uint16_t)(value & 0x0FFF);
    return (uint16_t)(raw12 << 4);
}

static uint16_t AD9106_EncodeUnsigned12Shift4(uint16_t value)
{
    value &= 0x0FFFU;
    return (uint16_t)(value << 4);
}