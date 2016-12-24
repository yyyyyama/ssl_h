#include <iostream>

namespace ai_server{
namespace model{

    class refbox{
        class team_info{
            std::string name_;
            int score_, goalie_;
            int red_cards_, yellow_cards_, yellow_card_times_;
            int timeouts_, timeout_time_;

        public:
            team_info(std::string name);
            std::string name();
            int score();
            int goalie();
            int red_cards();
            int yellow_cards();
            int yellow_card_times();
            int timeouts();
            int timeout_time();
            void set_score(int value);
            void set_goalie(int value);
            void set_red_cards(int value);
            void set_yellow_cards(int value);
            void set_yellow_card_times(int value);
            void set_timeouts(int value);
            void set_timeout_time(int value);

        };
        
        enum class stage_name {NORMAL_FIRST_HALF_PRE, NORMAL_FIRST_HALF,
                                NORMAL_HALF_TIME, NORMAL_SECOND_HALF_PRE,
                                NORMAL_SECOND_HALF, EXTRA_TIME_BREAK,
                                EXTRA_FIRST_HALF_PRE, EXTRA_FIRST_HALF,
                                EXTRA_HALF_TIME, EXTRA_SECOND_HALF_PRE,
                                EXTRA_SECOND_HALF, PENALTY_SHOOTOUT_BREAK,
                                PENALTY_SHOOTOUT, POST_GAME};
        enum class game_command {HALF, STOP, NORMAL_START, FORCE_START,
                                PREPARE_KICKOFF_YELLOW, PREPARE_KICKOFF_BLUE,
                                PREPARE_PENALTY_YELLOW, PREPARE_PENALTY_BLUE,
                                DIRECT_FREE_YELLOW, DIRECT_FREE_BLUE,
                                INDIRECT_FREE_YELLOW, INDIRECT_FREE_BLUE,
                                TIMEOUT_YELLOW, TIMEOUT_BLUE,
                                GOAL_YELLOW, GOAL_BLUE,
                                BALL_PLACEMENT_YELLOW, BALL_PLACEMENT_BLUE};
     
        int packet_timestamp_, stage_time_left_;
        int stage_;
        int command_;
        team_info team_yellow_ = {"yellow"}, team_blue_ = {"blue"};
    
    public:
        refbox();
        int packet_timestamp();
        int stage_time_left();
        int stage();
        int command();
        team_info team_yellow();
        team_info team_blue();
        void set_packet_timestamp(int value);
        void set_stage_time_left(int value);
        void set_stage(int value);
        void set_command(int value);
        void set_team_yellow(team_info value);
        void set_team_blue(team_info value);


    };
}
}