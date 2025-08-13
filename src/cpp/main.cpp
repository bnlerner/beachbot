#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/wait.h>
#include <chrono>
#include "ipc/pubsub.h"
#include "ipc/core.h"

using namespace std;

int main() {
    ChannelSpec channel{"test_channel", 1024};

    pid_t pid = fork();

    if (pid == 0) {
        // Child: Subscriber
        NodeID node_id{"subscriber"};
        auto callback = [](const BaseMessage& msg) {
            cout << "Received: " << msg.data << endl;
        };
        Subscriber sub(node_id, channel, callback);
        sub.listen();

        // Run for a while to receive messages
        this_thread::sleep_for(chrono::seconds(2));  // Adjust as needed
        sub.close();
    } else if (pid > 0) {
        // Parent: Publisher
        NodeID node_id{"publisher"};
        Publisher pub(node_id, channel);

        double total_time = 0.0;
        for (int i = 1; i <= 10; ++i) {
            BaseMessage msg;
            msg.data = "Message " + to_string(i);

            auto start = chrono::high_resolution_clock::now();
            pub.publish(msg);
            auto end = chrono::high_resolution_clock::now();

            total_time += chrono::duration<double>(end - start).count();
            this_thread::sleep_for(chrono::milliseconds(100));  // Simulate delay
        }

        cout << "Average publish time: " << (total_time / 10) << " seconds" << endl;

        // Wait for child
        waitpid(pid, nullptr, 0);
        pub.close();
    } else {
        cerr << "Fork failed" << endl;
        return 1;
    }

    return 0;
}
