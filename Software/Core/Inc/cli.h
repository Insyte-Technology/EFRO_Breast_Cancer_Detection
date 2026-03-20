#ifndef CLI_H
#define CLI_H

#include <stdint.h>

void CLI_Init(void);
void CLI_RxBytes(const uint8_t *data, uint32_t len);
void CLI_Process(void);

#endif