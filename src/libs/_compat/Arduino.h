

#pragma once

#include <stdint.h>



#include "platform.h"
#include "gpio_api.h"

#include "Marduino_compat.h"
#include "millis.h"

#include "utils.h"


#define LOW 0
#define HIGH 1
#define OUTPUT 1

void digitalWrite(uint16_t a, uint16_t b);
void pinMode(uint16_t a, uint16_t b);

#define delay safe_delay_ms


