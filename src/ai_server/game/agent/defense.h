#ifndef AI_SERVER_GAME_ACTION_DEFENSE_H
#define AI_SERVER_GAME_ACTION_DEFENSE_H

#include <vector>
#include <memory>

#include "ai_server/game/agent/base.h"

namespace ai_server{
	namespace game{
		namespace action{
			class defense:public base{
				public:
				defense(const model::world& world,bool is_yellow,unsigned int keeper_id,const std::vector<unsigned int>& ids);
				std::vector<std::shared_ptr<action::base>> execute() override;
				private:
					std::vector<unsigned int>& ids_;
					unsigned int keeper_id_;
					double x_=0;
					double y_=0;
			}

		}
	}
}
#endif//AI_SERVER_GAME_ACTION_DEFENSE_H
