#ifndef IPC_PUBSUB_H
#define IPC_PUBSUB_H

#include "core.h"
#include <functional>
#include <optional>
#include <thread>
#include <atomic>
#include <chrono>

class Publisher {
public:
    Publisher(const NodeID& node_id, const ChannelSpec& channel);
    ~Publisher();
    void publish(BaseMessage& msg);
    void close();

private:
    NodeID _node_id;
    ChannelSpec _channel;
    int _shm_fd = -1;
    void* _shm_ptr = nullptr;
    int _shm_id = -1;
    int _lock_fd = -1;
};

class Subscriber {
public:
    Subscriber(const NodeID& node_id, const ChannelSpec& channel, std::function<void(const BaseMessage&)> callback = nullptr);
    ~Subscriber();
    std::optional<BaseMessage> wait_for_message();
    void listen();
    void close();

private:
    NodeID _node_id;
    ChannelSpec _channel;
    std::function<void(const BaseMessage&)> _callback;
    int _shm_fd = -1;
    void* _shm_ptr = nullptr;
    int _shm_id = -1;
    int _lock_fd = -1;
    std::optional<BaseMessage> _last_msg;
    std::atomic<bool> _running{false};
    std::thread _listen_thread;
     std::optional<BaseMessage> get_msg();
};

#endif // IPC_PUBSUB_H

