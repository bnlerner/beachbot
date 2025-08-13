#ifndef IPC_CORE_H
#define IPC_CORE_H

#include <string>
#include <optional>

struct NodeID {
    std::string name;
    explicit NodeID(const std::string& n) : name(n) {}
    bool operator==(const NodeID& other) const { return name == other.name; }
    bool operator<(const NodeID& other) const { return name < other.name; }
};

struct BaseMessage {
    std::optional<NodeID> origin;
    std::optional<double> creation;
    double lifetime = 0.5;
    std::string data; // generic payload for simple IPC
    virtual ~BaseMessage() = default;

    bool is_expired() const;
    bool operator==(const BaseMessage& other) const {
        return origin.has_value() == other.origin.has_value() &&
               (!origin.has_value() || origin->name == other.origin->name) &&
               creation == other.creation &&
               data == other.data;
    }
};

struct ChannelSpec {
    std::string channel;
    size_t msg_size = 1024;
    std::string name() const { return channel; }
};

struct RequestSpec {
    std::string base_channel;
    ChannelSpec cancel_channel() const;
    ChannelSpec request_channel() const;
    ChannelSpec response_channel(const NodeID& node_id) const;
};

struct Request : BaseMessage {};

struct RequestResponse : BaseMessage {
    Request request;
    std::string result;
    bool cancelled = false;
};

struct RequestCancel : BaseMessage {
    Request request;
};

#endif // IPC_CORE_H
