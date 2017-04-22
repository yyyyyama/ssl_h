#include "ai_server/game/agent/defense.h"

namespace ai_server{
	namespace game{
		namespace action{
			defense::defense(const model::world& world,bool is_yellow,unsigned int keeper_id,const std::vector<unsigned int>& ids):keeper_id_(keeper_id),ids_(ids){};
				std::vector<std::shared_ptr<action::base>> defens::execute(){
				}
		}
	}
}
#endif//AI_SERVER_GAME_ACTION_DEFENSE_H
