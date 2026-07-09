#include <iostream>
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>
#include <map>
#include <functional>
#include <stdexcept>
#include <utility>
#include "request.h"

using namespace std;

// Helper for logging (simplified)
void log_info(const string& msg) { cout << "[INFO] " << msg << endl; }
void log_error(const string& msg) { cerr << "[ERROR] " << msg << endl; }

RequestExecutable::RequestExecutable(const Request& req, function<future<string>(const Request&)> func)
    : request(req), func(std::move(func)) {}

future<string> RequestExecutable::execute() {
    promise<string> prom;
    future<string> fut = prom.get_future();

    execution_thread = thread([this, prom = std::move(prom)]() mutable {
        try {
            auto result = func(request).get();
            prom.set_value(result);
        } catch (const exception& e) {
            log_error("Error executing request: " + string(e.what()));
            prom.set_exception(current_exception());
        }
    });

    return fut;
}

void RequestExecutable::cancel() {
    if (execution_thread.joinable()) {
        execution_thread.detach();
    }
}

RequestServer::RequestServer(const NodeID& node_id, const RequestSpec& req_spec, function<future<string>(const Request&)> req_func)
    : node_id(node_id), request_spec(req_spec), request_function(std::move(req_func)) {
    request_sub = make_unique<Subscriber>(node_id, request_spec.request_channel(), [this](const BaseMessage& msg) {
        this->process_request(msg);
    });
    cancel_sub = make_unique<Subscriber>(node_id, request_spec.cancel_channel(), [](const BaseMessage& /*msg*/) {
        // No-op for now
    });
}

void RequestServer::start() {
    request_sub->listen();
    cancel_sub->listen();
}

void RequestServer::close() {
    request_sub->close();
    cancel_sub->close();
    for (auto& [_, pub] : response_publishers) {
        pub.close();
    }
}

void RequestServer::process_request(const BaseMessage& base_msg) {
        
        Request msg;
        msg.origin = base_msg.origin;
        msg.creation = base_msg.creation;
        msg.data = base_msg.data;
        cur_request = make_unique<RequestExecutable>(msg, request_function);
        log_info("Executing request from " + (msg.origin ? msg.origin->name : std::string("unknown")));

        auto fut = cur_request->execute();
        thread([this, fut = std::move(fut), msg]() mutable {
            try {
                string result = fut.get();
                RequestResponse response;
                response.request = msg;
                response.result = result;
                response.data = result;
                response.cancelled = false;

                NodeID sender = msg.origin ? *msg.origin : NodeID{"unknown"};
                response_publishers.try_emplace(sender, node_id, request_spec.response_channel(sender));
                response_publishers.at(sender).publish(response);
                log_info("Finished request with result: " + result);
            } catch (const exception& e) {
                // Handle cancelled or error
                RequestResponse response;
                response.request = msg;
                response.result = "";
                response.data = "";
                response.cancelled = true;
                // Publish cancelled response
                NodeID sender = msg.origin ? *msg.origin : NodeID{"unknown"};
                response_publishers.try_emplace(sender, node_id, request_spec.response_channel(sender));
                response_publishers.at(sender).publish(response);
            }
        }).detach();
}

// Optional: not wired into cancel channel yet
// void RequestServer::cancel_request(const RequestCancel& msg) {
//     if (cur_request && cur_request->request == msg.request) {
//         cur_request->cancel();
//     }
// }

RequestClient::RequestClient(const NodeID& node_id, const RequestSpec& req_spec)
    : node_id(node_id), request_spec(req_spec),
      request_pub(node_id, req_spec.request_channel()),
      cancel_pub(node_id, req_spec.cancel_channel()) {
    response_sub = make_unique<Subscriber>(node_id, req_spec.response_channel(node_id), nullptr);
}

future<RequestResponse> RequestClient::send(Request msg) {
        promise<RequestResponse> prom;
        future<RequestResponse> fut = prom.get_future();

        log_info("Sending request from " + node_id.name + " to " + request_spec.base_channel);
        // Put request payload into data so the server can echo or process
        msg.data = "request";
        request_pub.publish(msg);

        // Start a thread to wait for response
        thread([this, prom = std::move(prom), msg]() mutable {
            while (true) {
                auto opt_resp = response_sub->wait_for_message();
                if (opt_resp) {
                    // Our pubsub delivers BaseMessage; here we expect a RequestResponse-like content in data.
                    // For this simple IPC, accept any response and return.
                    RequestResponse response;
                    response.request = msg;
                    response.result = opt_resp->data;
                    response.cancelled = false;
                    prom.set_value(response);
                        return;
                }
                // Check for cancellation or timeout (add if needed)
                this_thread::sleep_for(chrono::milliseconds(10));
            }
        }).detach();

    return fut;
}

void RequestClient::close() {
    response_sub->close();
}
