#include "core.h"
#include <chrono>

using namespace std;

static inline double now_seconds() {
    return chrono::duration_cast<chrono::duration<double>>(chrono::high_resolution_clock::now().time_since_epoch()).count();
}

bool BaseMessage::is_expired() const {
    if (!creation.has_value()) return false;
    return creation.value() + lifetime < now_seconds();
}

ChannelSpec RequestSpec::cancel_channel() const { return ChannelSpec{"cancel-" + base_channel, 1024}; }
ChannelSpec RequestSpec::request_channel() const { return ChannelSpec{"request-" + base_channel, 1024}; }
ChannelSpec RequestSpec::response_channel(const NodeID& node_id) const { return ChannelSpec{"response-" + base_channel + "-" + node_id.name, 1024}; }
