//
// Created by ckyf on 23-3-28.
//
#ifndef AUTOSENTRY_UART_H
#define AUTOSENTRY_UART_H
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include "check.h"
#include <iostream>
auto uart_start(const char* uart_name){//启动串口
    auto uart_com = open(uart_name, O_RDWR);
    struct termios Opt;
    tcgetattr(uart_com, &Opt);
    cfsetispeed(&Opt,B115200);
    cfsetospeed(&Opt,B115200);
    tcsetattr(uart_com,TCSANOW,&Opt);
    return uart_com;
}

std::string uart_buff;

struct __attribute__((packed)) Head{
    uint8_t SOF;
    uint16_t length;
    uint8_t seq;
    uint8_t CRC8;
    uint16_t cmd_id;
};

struct __attribute__((packed)) Tail{
    uint16_t CRC16;
};

bool checker_head(Head head){//头校验
    //std::cout<<(int)head.SOF<<(int)head.length<<(int)head.seq<<(int)head.CRC8<<(int)head.cmd_id<<std::endl;
    if(head.SOF != 0xA5){
        return true;
    }
    uint8_t crc_check = ms::crc8check((uint8_t*)&head, 4);
    if(crc_check == head.CRC8){
        return false;
    }
    return true;
}

bool checker_all(Head head, Tail tail, uint8_t* p){//全帧长度校验
    uint16_t crc16_check = ms::crc16check(p, head.length+7);
    if(crc16_check == tail.CRC16){
        return false;
    }
    return true;
}

int getLength(Head head){
    return head.length + 9; //返回全包长度
}

uint16_t getId(Head head){
    return head.cmd_id;
}
#endif //AUTOSENTRY_UART_H
