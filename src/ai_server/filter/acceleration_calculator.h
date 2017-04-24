#ifndef AI_SERVER_FILTER_ACCELERATION_CALCULATOR_H
#define AI_SERVER_FILTER_ACCELERATION_CALCULATOR_H

#include "base.h"
#include "ai_server/model/robot.h"
#include <chrono>
#include <cmath>

namespace ai_server{
namespace filter{

  template <class T>
    class acceleration_calculator:public base<T>{
      private:
        std::chrono::high_resolution_clock::time_point prev_time_;
        T prev_state_;

      public:
        acceleration_calculator();
        void apply(T&, std::chrono::high_resolution_clock::time_point) override;
        void reset() override;
    };

  template <class T>
    acceleration_calculator<T>::acceleration_calculator():prev_state_(T{}){}

  template <class T>
    void acceleration_calculator<T>::reset(){
      prev_state_=T{};
    }
} // filter
} // ai_server
#endif  // AI_SERVER_FILTER_ACCELERATION_CALCULATOR_H

