#ifndef IPC_REQUEST_H
#define IPC_REQUEST_H

#include "core.h"
#include "pubsub.h"
#include <memory>
#include <future>
#include <map>

class RequestExecutable {
public:
    RequestExecutable(const Request& req, std::function<std::future<std::string>(const Request&)> func);
    std::future<std::string> execute();
    void cancel();
    Request request;

private:
    std::function<std::future<std::string>(const Request&)> func;
    std::thread execution_thread;
};

class RequestServer {
public:
    RequestServer(const NodeID& node_id, const RequestSpec& req_spec, std::function<std::future<std::string>(const Request&)> req_func);
    void start();
    void close();

private:
    NodeID node_id;
    RequestSpec request_spec;
    std::function<std::future<std::string>(const Request&)> request_function;
    std::unique_ptr<Subscriber> request_sub;
    std::unique_ptr<Subscriber> cancel_sub;
    std::map<NodeID, Publisher> response_publishers;
    std::unique_ptr<RequestExecutable> cur_request;
    void process_request(const BaseMessage& base_msg);
};

class RequestClient {
public:
    RequestClient(const NodeID& node_id, const RequestSpec& req_spec);
    std::future<RequestResponse> send(Request msg);
    void close();

private:
    NodeID node_id;
    RequestSpec request_spec;
    Publisher request_pub;
    Publisher cancel_pub;
    std::unique_ptr<Subscriber> response_sub;
};

#endif // IPC_REQUEST_H
