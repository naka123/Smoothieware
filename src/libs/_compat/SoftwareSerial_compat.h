/**
 * ... modified
 *
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef SOFTWARE_SERIAL_H
#define SOFTWARE_SERIAL_H

#pragma once

#include <stdint.h>
#include <SoftwareSerial_compat2.h>
#include "mbed.h"

#include "libs/SoftSerial/BufferedSoftSerial.h"
#include "libs/gpio.h"


#define SW_SERIAL_PLACEHOLDER 1

class SoftwareSerial : public Stream1 {
public:
    SoftwareSerial(PinName TX_pin, PinName RX_pin);

    void begin(const uint32_t baudrate);

    bool available();

    uint8_t read();

    uint16_t write(uint8_t byte);

    void flush();


    void listen();

    void stopListening();

protected:
    bool listening;

    void calculate_delay(int baudrate, int bits, int parity, int stop);

    SoftSerial *serial;

    float delay_time;

};

#endif // SOFTWARE_SERIAL_H