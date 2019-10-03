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
//#if defined(__STM32F1__) && !defined(HAVE_SW_SERIAL)

/**
 * Empty class for Software Serial implementation (Custom RX/TX pins)
 *
 * TODO: Optionally use https://github.com/FYSETC/SoftwareSerialM if TMC UART is wanted
 */

#include "SoftwareSerial_compat.h"

#include "Kernel.h"
#include "libs/StreamOutputPool.h"


// Constructor

SoftwareSerial::SoftwareSerial(PinName TX_pin, PinName RX_pin) {
    serial = new SoftSerial(TX_pin, RX_pin);
}

// Public

void SoftwareSerial::calculate_delay(int baudrate, int bits, int parity, int stop) {

    float bittime = 1000.0 / baudrate;
    // here we calculate how long a byte with all surrounding bits take
    // startbit + number of bits + parity bit + stop bit
    delay_time = bittime * (1 + bits + parity + 1);
}

void SoftwareSerial::begin(const uint32_t baudrate) {

    serial->baud(baudrate);
    serial->format(8,serial->Parity::None,1);
    calculate_delay(baudrate, 8, 0, 1);

}

bool SoftwareSerial::available() {
    return false;
}

uint8_t SoftwareSerial::read() {
    return 0;
}

uint16_t SoftwareSerial::write(uint8_t byte) {
//    THEKERNEL->streams->printf("SoftwareSerial::write byte: %02x\n", byte);
    //serial->write(&byte, 1);
    serial->putc(byte);
    return 1;
}

void SoftwareSerial::flush() {}


void SoftwareSerial::listen() {
  listening = true;
}

void SoftwareSerial::stopListening() {
  listening = false;
}

//#endif //__STM32F1__
