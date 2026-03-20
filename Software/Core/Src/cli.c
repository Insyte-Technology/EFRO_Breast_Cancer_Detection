#include "cli.h"
#include "usbd_cdc_if.h"
#include "ad9106.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#define CLI_RX_BUFFER_SIZE 128
#define CLI_TX_BUFFER_SIZE 128
#define CLI_MAX_ARGS 8

static char cli_rx_line[CLI_RX_BUFFER_SIZE];
static uint32_t cli_rx_index = 0;
static volatile uint8_t cli_line_ready = 0;

static AD9106_Config_t cli_state;

static void CLI_Print(const char *text);
static void CLI_Printf(const char *fmt, ...);
static int CLI_Tokenize(char *line, char *argv[], int max_args);
static void CLI_Execute(char *line);
static const char *CLI_WaveToString(AD9106_Waveform_t wave);

static void CLI_Print(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (CDC_Transmit_FS((uint8_t *)text, (uint16_t)strlen(text)) == USBD_BUSY)
    {
        HAL_Delay(1);
    }
}

static void CLI_Printf(const char *fmt, ...)
{
    char buf[CLI_TX_BUFFER_SIZE];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    CLI_Print(buf);
}

static int CLI_Tokenize(char *line, char *argv[], int max_args)
{
    int argc = 0;
    char *token = strtok(line, " \t");

    while ((token != NULL) && (argc < max_args))
    {
        argv[argc++] = token;
        token = strtok(NULL, " \t");
    }

    return argc;
}

static const char *CLI_WaveToString(AD9106_Waveform_t wave)
{
    switch (wave)
    {
    case AD9106_WAVE_SINE:
        return "sine";
    case AD9106_WAVE_TRIANGLE:
        return "triangle";
    case AD9106_WAVE_SAW:
        return "saw";
    case AD9106_WAVE_DC:
        return "dc";
    default:
        return "unknown";
    }
}

void CLI_Init(void)
{
    memset(cli_rx_line, 0, sizeof(cli_rx_line));
    cli_rx_index = 0;
    cli_line_ready = 0;

    AD9106_GetDefaultConfig(&cli_state);

    CLI_Print("\r\nAD9106 CLI ready\r\nType 'help' for commands\r\n> ");
}

void CLI_RxBytes(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        char c = (char)data[i];

        if ((c == '\r') || (c == '\n'))
        {
            if (cli_rx_index > 0)
            {
                cli_rx_line[cli_rx_index] = '\0';
                cli_line_ready = 1;
                cli_rx_index = 0;
            }
        }
        else if ((c == '\b') || (c == 127))
        {
            if (cli_rx_index > 0)
            {
                cli_rx_index--;
            }
        }
        else
        {
            if (cli_rx_index < (CLI_RX_BUFFER_SIZE - 1))
            {
                cli_rx_line[cli_rx_index++] = c;
            }
        }
    }
}

void CLI_Process(void)
{
    if (!cli_line_ready)
    {
        return;
    }

    cli_line_ready = 0;

    char line_copy[CLI_RX_BUFFER_SIZE];
    strncpy(line_copy, cli_rx_line, sizeof(line_copy) - 1);
    line_copy[sizeof(line_copy) - 1] = '\0';

    CLI_Execute(line_copy);
    CLI_Print("> ");
}

static void CLI_Execute(char *line)
{
    char *argv[CLI_MAX_ARGS];
    int argc = CLI_Tokenize(line, argv, CLI_MAX_ARGS);

    if (argc == 0)
    {
        CLI_Print("\r\n");
        return;
    }

    if (strcmp(argv[0], "help") == 0)
    {
        CLI_Print(
            "\r\nCommands:\r\n"
            "  help\r\n"
            "  status\r\n"
            "  read <reg>\r\n"
            "  write <reg> <value>\r\n"
            "  update\r\n"
            "  run <0|1>\r\n"
            "  reset\r\n"
            "  output <1|2|3|4>\r\n"
            "  wave <sine|triangle|saw|dc>\r\n"
            "  freq <hz>\r\n"
            "  phase <deg>\r\n"
            "  gain <value>\r\n"
            "  offset <value>\r\n");
    }
    else if (strcmp(argv[0], "status") == 0)
    {
        CLI_Printf(
            "\r\nState:\r\n"
            "  output : %u\r\n"
            "  wave   : %s\r\n"
            "  freq   : %lu Hz\r\n"
            "  phase  : %.2f deg\r\n"
            "  gain   : %.3f\r\n"
            "  offset : %ld\r\n",
            cli_state.output,
            CLI_WaveToString(cli_state.waveform),
            cli_state.freq_hz,
            cli_state.phase_deg,
            cli_state.gain,
            cli_state.offset);
    }
    else if (strcmp(argv[0], "read") == 0)
    {
        if (argc < 2)
        {
            CLI_Print("\r\nERR usage: read <reg>\r\n");
            return;
        }

        uint16_t reg = (uint16_t)strtoul(argv[1], NULL, 0);
        uint16_t val = AD9106_ReadReg(reg);
        CLI_Printf("\r\nREG[0x%04X] = 0x%04X\r\n", reg, val);
    }
    else if (strcmp(argv[0], "write") == 0)
    {
        if (argc < 3)
        {
            CLI_Print("\r\nERR usage: write <reg> <value>\r\n");
            return;
        }

        uint16_t reg = (uint16_t)strtoul(argv[1], NULL, 0);
        uint16_t val = (uint16_t)strtoul(argv[2], NULL, 0);

        AD9106_WriteReg(reg, val);
        CLI_Printf("\r\nOK write 0x%04X = 0x%04X\r\n", reg, val);
    }
    else if (strcmp(argv[0], "update") == 0)
    {
        AD9106_Update();
        CLI_Print("\r\nOK update\r\n");
    }
    else if (strcmp(argv[0], "run") == 0)
    {
        if (argc < 2)
        {
            CLI_Print("\r\nERR usage: run <0|1>\r\n");
            return;
        }

        uint8_t enable = (uint8_t)strtoul(argv[1], NULL, 0);
        AD9106_Run(enable ? 1U : 0U);
        CLI_Printf("\r\nOK run %u\r\n", enable ? 1U : 0U);
    }
    else if (strcmp(argv[0], "reset") == 0)
    {
        AD9106_Reset();
        CLI_Print("\r\nOK reset\r\n");
    }
    else if (strcmp(argv[0], "output") == 0)
    {
        if (argc < 2)
        {
            CLI_Print("\r\nERR usage: output <1|2|3|4>\r\n");
            return;
        }

        uint8_t out = (uint8_t)strtoul(argv[1], NULL, 0);
        if ((out < 1U) || (out > 4U))
        {
            CLI_Print("\r\nERR output must be 1..4\r\n");
            return;
        }

        cli_state.output = out;
        AD9106_ApplyConfig(&cli_state);
        CLI_Printf("\r\nOK output %u\r\n", out);
    }
    else if (strcmp(argv[0], "wave") == 0)
    {
        if (argc < 2)
        {
            CLI_Print("\r\nERR usage: wave <sine|triangle|saw|dc>\r\n");
            return;
        }

        if (strcmp(argv[1], "sine") == 0)
        {
            cli_state.waveform = AD9106_WAVE_SINE;
        }
        else if (strcmp(argv[1], "triangle") == 0)
        {
            cli_state.waveform = AD9106_WAVE_TRIANGLE;
        }
        else if (strcmp(argv[1], "saw") == 0)
        {
            cli_state.waveform = AD9106_WAVE_SAW;
        }
        else if (strcmp(argv[1], "dc") == 0)
        {
            cli_state.waveform = AD9106_WAVE_DC;
        }
        else
        {
            CLI_Print("\r\nERR wave must be sine, triangle, saw or dc\r\n");
            return;
        }

        AD9106_ApplyConfig(&cli_state);
        CLI_Printf("\r\nOK wave %s\r\n", CLI_WaveToString(cli_state.waveform));
    }
    else if (strcmp(argv[0], "freq") == 0)
    {
        if (argc < 2)
        {
            CLI_Print("\r\nERR usage: freq <hz>\r\n");
            return;
        }

        cli_state.freq_hz = (uint32_t)strtoul(argv[1], NULL, 0);
        AD9106_ApplyConfig(&cli_state);
        CLI_Printf("\r\nOK freq %lu Hz\r\n", cli_state.freq_hz);
    }
    else if (strcmp(argv[0], "phase") == 0)
    {
        if (argc < 2)
        {
            CLI_Print("\r\nERR usage: phase <deg>\r\n");
            return;
        }

        cli_state.phase_deg = (float)atof(argv[1]);
        AD9106_ApplyConfig(&cli_state);
        CLI_Printf("\r\nOK phase %.2f deg\r\n", cli_state.phase_deg);
    }
    else if (strcmp(argv[0], "gain") == 0)
    {
        if (argc < 2)
        {
            CLI_Print("\r\nERR usage: gain <value>\r\n");
            return;
        }

        cli_state.gain = (float)atof(argv[1]);
        AD9106_ApplyConfig(&cli_state);
        CLI_Printf("\r\nOK gain %.3f\r\n", cli_state.gain);
    }
    else if (strcmp(argv[0], "offset") == 0)
    {
        if (argc < 2)
        {
            CLI_Print("\r\nERR usage: offset <value>\r\n");
            return;
        }

        cli_state.offset = (int32_t)strtol(argv[1], NULL, 0);
        AD9106_ApplyConfig(&cli_state);
        CLI_Printf("\r\nOK offset %ld\r\n", cli_state.offset);
    }
    else
    {
        CLI_Print("\r\nERR unknown command\r\n");
    }

    CLI_Print("\r\n");
}