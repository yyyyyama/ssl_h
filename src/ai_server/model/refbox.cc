#include <iostream>
#include "refbox.h"

namespace ai_server{
namespace model{

    refbox::refbox(){
        packet_timestamp_ = 0;
        stage_time_left_ = 0;
        stage_ = static_cast<int>(refbox::stage_name::NORMAL_FIRST_HALF_PRE);
        command_ = static_cast<int>(refbox::game_command::HALF);
    }

    int refbox::packet_timestamp(){
        return packet_timestamp_;
    }

    int refbox::stage_time_left(){
        return stage_time_left_;
    }

    int refbox::stage(){
        return stage_;
    }

    int refbox::command(){
        return command_;
    }

    refbox::team_info refbox::team_yellow(){
        return team_yellow_;
    }

    refbox::team_info refbox::team_blue(){
        return team_blue_;
    }

    void refbox::set_packet_timestamp(int value){
        packet_timestamp_ = value;
    }

    void refbox::set_stage_time_left(int value){
        stage_time_left_ = value;
    }

    void refbox::set_stage(int value){
        stage_ = value;
    }

    void refbox::set_command(int value){
        command_ = value;
    }

    void refbox::set_team_yellow(refbox::team_info value){
        team_yellow_ = value;
    }

    void refbox::set_team_blue(refbox::team_info value){
        team_blue_ = value;
    }

    refbox::team_info::team_info(std::string name){
        name_ = name;
        score_ = 0;
        goalie_ = 0;
        red_cards_ = 0;
        yellow_cards_ = 0;
        yellow_card_times_ = 0;
        timeouts_ = 0;
        timeout_time_ = 0;
    }

    int refbox::team_info::score(){
        return score_;
    }

    int refbox::team_info::goalie(){
        return goalie_;
    }

    int refbox::team_info::red_cards(){
        return red_cards_;
    }

    int refbox::team_info::yellow_cards(){
        return yellow_cards_;
    }

    int refbox::team_info::yellow_card_times(){
        return yellow_card_times_;
    }

    int refbox::team_info::timeouts(){
        return timeouts_;
    }

    int refbox::team_info::timeout_time(){
        return timeout_time_;
    }

    void refbox::team_info::set_score(int value){
        score_ = value;
    }

    void refbox::team_info::set_goalie(int value){
        goalie_ = value;
    }

    void refbox::team_info::set_red_cards(int value){
        red_cards_ = value;
    }

    void refbox::team_info::set_yellow_cards(int value){
        yellow_cards_ = value;
    }

    void refbox::team_info::set_yellow_card_times(int value){
        yellow_card_times_ = value;
    }

    void refbox::team_info::set_timeouts(int value){
        timeouts_ = value;
    }

    void refbox::team_info::set_timeout_time(int value){
        timeout_time_ = value;
    }

}
}