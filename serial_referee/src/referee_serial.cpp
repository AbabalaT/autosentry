//
// Created by ckyf on 23-3-27.
//
#include <string>
#include <ros/ros.h>
#include "serial_referee/message_game_status.h"
#include "serial_referee/message_game_hurt.h"
#include "serial_referee/message_game_command.h"
#include "serial_referee/message_game_HP.h"
#include <iostream>
#include "../include/uart.h"

ros::Publisher game_status_publisher;
ros::Publisher game_HP_publisher;
ros::Publisher game_commmand_publisher;
ros::Publisher game_hurt_publisher;

namespace Referee {
    typedef struct __attribute__((packed))
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    } ext_game_status_t;//比赛状态信息
    typedef struct __attribute__((packed))
    {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
    } ext_game_robot_HP_t;//血量信息
    typedef struct __attribute__((packed))
    {
        uint8_t armor_id : 4;
        uint8_t hurt_type : 4;
    } ext_robot_hurt_t;//收击指令
    typedef struct __attribute__((packed))
    {
        float target_position_x;
        float target_position_y;
        float target_position_z;
        uint8_t command_keyboard;
        uint16_t target_robot_ID;
    } ext_robot_command_t;//云台手指令

    void get_status(uint8_t* msg){
        ext_game_status_t ext_status = *(ext_game_status_t*)msg;
        serial_referee::message_game_status status_msg;
        status_msg.game_type = ext_status.game_type,
        status_msg.game_progress = ext_status.game_progress,
        status_msg.stage_remain_time = ext_status.stage_remain_time,
        status_msg.SyncTimeStamp = ext_status.SyncTimeStamp,
        game_status_publisher.publish(status_msg);
    }
    void get_HP(uint8_t* msg){
        ext_game_robot_HP_t ext_HP = *(ext_game_robot_HP_t*)msg;
        serial_referee::message_game_HP HP_msg;
        HP_msg.red_1_robot_HP = ext_HP.red_1_robot_HP;
        HP_msg.red_2_robot_HP = ext_HP.red_2_robot_HP;
        HP_msg.red_3_robot_HP = ext_HP.red_3_robot_HP;
        HP_msg.red_4_robot_HP = ext_HP.red_4_robot_HP;
        HP_msg.red_5_robot_HP = ext_HP.red_5_robot_HP;
        HP_msg.red_7_robot_HP = ext_HP.red_7_robot_HP;
        HP_msg.red_outpost_HP = ext_HP.red_outpost_HP;
        HP_msg.red_base_HP = ext_HP.red_base_HP;
        HP_msg.blue_1_robot_HP = ext_HP.blue_1_robot_HP;
        HP_msg.blue_2_robot_HP = ext_HP.blue_2_robot_HP;
        HP_msg.blue_3_robot_HP = ext_HP.blue_3_robot_HP;
        HP_msg.blue_4_robot_HP = ext_HP.blue_4_robot_HP;
        HP_msg.blue_5_robot_HP = ext_HP.blue_5_robot_HP;
        HP_msg.blue_7_robot_HP = ext_HP.blue_7_robot_HP;
        HP_msg.blue_outpost_HP = ext_HP.blue_outpost_HP;
        HP_msg.blue_base_HP = ext_HP.blue_base_HP;
        game_HP_publisher.publish(HP_msg);
    }
    void get_hurt(uint8_t* msg){
        ext_robot_hurt_t ext_hurt = *(ext_robot_hurt_t*)msg;
        serial_referee::message_game_hurt hurt_msg;
        hurt_msg.armor_id = ext_hurt.armor_id;
        hurt_msg.hurt_type = ext_hurt.hurt_type;
        game_hurt_publisher.publish(hurt_msg);
    }
    void get_command(uint8_t* msg){
        ext_robot_command_t ext_command = *(ext_robot_command_t*)msg;
        serial_referee::message_game_command command_msg;
        command_msg.target_position_x = ext_command.target_position_x;
        command_msg.target_position_y = ext_command.target_position_y;
        command_msg.target_position_z = ext_command.target_position_z;
        command_msg.command_keyboard = ext_command.command_keyboard;
        command_msg.target_robot_ID = ext_command.target_robot_ID;
        game_commmand_publisher.publish(command_msg);
    }
}

bool scan() {
    int eraseSize = 0;
    if ((int)uart_buff.size() < sizeof(Head) + sizeof(Tail) +1) {
        return false;
    }
    bool okHeadFound = false;
    bool frameFound = false;
    for (long i = 0; i < ((long)uart_buff.size() - sizeof(Head) - sizeof(Tail)); i=i+1) {
        uint8_t* p = (uint8_t*)uart_buff.data() + i;
        int size = (int)uart_buff.size() - i;
        Head head;
        memcpy(&head, p, sizeof(Head));

        int headCheckerResult = checker_head(head);
        if (headCheckerResult) {
            if (!okHeadFound) {
                eraseSize = i-1;
            }
            continue;
        }
        //ROS_INFO("OK444444!!!");
        int length = getLength(head);
        if (size < length ) {
            okHeadFound = 1;
            continue;
        }

        Tail tail;
        memcpy(&tail, p + length - 2, sizeof(Tail));
        if(checker_all(head, tail, p)){
            continue;
        }
        eraseSize = i + head.length;
        p = (uint8_t*)uart_buff.data() + i + 7;

        //ROS_INFO("current buff length = %ld", uart_buff.size());
        switch (head.cmd_id) {
            case 0x0001:
                if(head.length == 11){
                    Referee::get_status(p);
                }
                break;
            case 0x0003:
                //ROS_INFO("referee message with ID 0x0003 has detected and length = %d",head.length);
                if(head.length == 32){
                    Referee::get_HP(p);
                }
                break;
            case 0x0206:
                if(head.length == 1){
                    Referee::get_hurt(p);
                }
                break;
            case 0x0303:
                if(head.length == 15){
                    Referee::get_command(p);
                }
                break;
        }
        eraseSize = i + size;
        frameFound = true;
    }
    uart_buff.erase(0, eraseSize);
    return frameFound;
}

int main(int argc, char** argv) {
    std::string serial_name;
    ros::init(argc, argv, "serial_referee");
    ros::NodeHandle nh("~");
    nh.param<std::string>("serial_referee_name", serial_name, "/dev/ttyUSB1");

    auto uart_com = open(serial_name.data(), O_RDWR);
    if(uart_com == -1){
        ROS_INFO("UART OPEN FAIL!");
        return 0;
    }
    struct termios Opt;
    tcgetattr(uart_com, &Opt);
    cfsetispeed(&Opt,B115200);
    cfsetospeed(&Opt,B115200);
    Opt.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    Opt.c_oflag  &= ~OPOST;   /*Output*/
    tcsetattr(uart_com,TCSANOW,&Opt);

    game_status_publisher = nh.advertise<serial_referee::message_game_status>("/referee/status", 5);
    game_HP_publisher = nh.advertise<serial_referee::message_game_HP>("/referee/HP", 5);
    game_commmand_publisher = nh.advertise<serial_referee::message_game_command>("/referee/command", 5);
    game_hurt_publisher = nh.advertise<serial_referee::message_game_hurt>("/referee/hurt", 5);
    ROS_INFO("Loop Start!");
    while(ros::ok()){
        uint8_t frame_hex[1] = {0x00};
        if(read(uart_com, &frame_hex, 1)){
            //std::cout<<"HEX:"<<(int)frame_hex[0]<<std::endl;
            uart_buff.push_back(frame_hex[0]);
            scan();
        }else{
            //ROS_INFO("Nothing Received!");
        }
        ros::spinOnce();
    }
    return 0;
}