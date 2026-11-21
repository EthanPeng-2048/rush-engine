module;
#include <vector>

module rush.engine.physics;
import rush.engine.events;

namespace apex {

rush::EventBus<ContactEvent>& CollisionBus() {
    static rush::EventBus<ContactEvent> bus;
    return bus;
}

}  // namespace apex
