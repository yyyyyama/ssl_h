#include "no_operation.h"

namespace ai_server{
namespace game{
namespace action{

model::command no_operation::execute(){
    model::command command{id_};
    return command;
}

bool no_operation::finished() const {
    return true;
}

}
}
}
