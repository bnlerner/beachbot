#include <iostream>
#include <thread>
#include <chrono>
#include <future>
#include <cassert>
#include "../ipc/request.h"  // Adjust path as needed
#include "../ipc/registry.h"
#include "../ipc/core.h"

using namespace std;

// Simple request handler that sleeps and returns a result
future<string> simple_handler(const Request& req) {
    promise<string> prom;
    thread([prom = std::move(prom), req]() mutable {
        // Simulate work: sleep 100ms
        this_thread::sleep_for(chrono::milliseconds(100));
        prom.set_value("Processed request from " + (req.origin ? req.origin->name : std::string("unknown")));
    }).detach();
    return prom.get_future();
}

int main() {
    // Test setup
    NodeID server_id("test_server");
    NodeID client_id("test_client");
    RequestSpec nav_spec = Requests::NAVIGATE;

    // Start server in a thread
    thread server_thread([&]() {
        RequestServer server(server_id, nav_spec, simple_handler);
        server.start();
        // Run for a while to handle requests
        this_thread::sleep_for(chrono::seconds(1));
        server.close();
    });

    // Give server time to start
    this_thread::sleep_for(chrono::milliseconds(50));

    // Client sends request
    RequestClient client(client_id, nav_spec);
    Request req;
    req.data = "payload";

    auto fut = client.send(req);
    auto response = fut.get();  // Wait for response

    // Verify
    assert(response.result == "Processed request from test_client");
    assert(!response.cancelled);
    cout << "Test passed: Received expected response." << endl;

    client.close();

    // Join server thread
    server_thread.join();

    return 0;
}
