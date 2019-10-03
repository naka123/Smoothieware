//
// Created by user on 26-Sep-19.
//

#ifndef SRC_STREAM1_H
#define SRC_STREAM1_H

#pragma once

class Stream1 {
public:
    virtual bool available();

    virtual uint8_t read();

    virtual uint16_t write(uint8_t byte);

    virtual void flush();

};

#endif //SRC_STREAM1_H
