#ifndef TELEMETRY_LINK_H
#define TELEMETRY_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "telemetry.h"

bool TelemetryLink_Init(UART_HandleTypeDef *huart);

void TelemetryLink_Update(void);

bool TelemetryLink_SendCombinedFast(const TelemetryCombinedFast *combined);

void TelemetryLink_GetCounters(TelemetryCounters *outCounters);

void TelemetryLink_OnUartRxCplt(UART_HandleTypeDef *huart);

void TelemetryLink_OnUartTxCplt(UART_HandleTypeDef *huart);

void TelemetryLink_OnUartError(UART_HandleTypeDef *huart);

#endif
