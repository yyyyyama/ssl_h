#include "halt.h"

namespace ai_server{
namespace game{
namespace agent{

halt::halt(const model::world& world, bool is_yellow, 
            const std::vector<unsigned int>& ids)
            : base(world, is_yellow), ids_(ids){
}

std::vector<std::shared_ptr<action::base>> halt::execute(){
    std::vector<std::shared_ptr<action::base>> exe;
    std::shared_ptr<action::no_operation> nop;
    for (auto it = ids_.begin(); it != ids_.end(); ++it) {
        nop = std::make_shared<action::no_operation>(world_, is_yellow_, *it);
        exe.push_back(nop);
    }

    return exe;
}

}
}
}