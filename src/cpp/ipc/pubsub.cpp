#include <iostream>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <functional>
#include <optional>
#include <fstream>
#include <atomic>
#include <sys/file.h>
#include <cerrno>
#include "core.h"
#include "pubsub.h"

using namespace std;

// Constants
const string LOCK_DIR = "/tmp/";  // Adjust for macOS
const string LOCK_SUFFIX = "-lock";
const chrono::duration<double> POLL_INTERVAL(0.001);

// Forward decl helpers
static int get_shm_id(int shm_fd);
static void close_shm(int shm_fd, const std::string& shm_name);
static int init_shm(ChannelSpec& channel);
static int gen_lock(const std::string& shm_name);

// Publisher implementation
Publisher::Publisher(const NodeID& node_id, const ChannelSpec& channel)
    : _node_id(node_id), _channel(channel) {
    _shm_fd = init_shm(_channel);
    _shm_ptr = mmap(nullptr, _channel.msg_size, PROT_READ | PROT_WRITE, MAP_SHARED, _shm_fd, 0);
    if (_shm_ptr == MAP_FAILED) throw runtime_error("mmap failed");
    _shm_id = get_shm_id(_shm_fd);
    _lock_fd = gen_lock(_channel.name());
}

Publisher::~Publisher() {
    if (_shm_ptr) munmap(_shm_ptr, _channel.msg_size);
    if (_shm_fd != -1) ::close(_shm_fd);
}

void Publisher::publish(BaseMessage& msg) {
    flock(_lock_fd, LOCK_EX);
    int cur_shm_id = get_shm_id(_shm_fd);
    if (cur_shm_id == -1) {
        flock(_lock_fd, LOCK_UN);
        throw runtime_error("SHM is closed");
    }
    if (cur_shm_id != _shm_id) {
        flock(_lock_fd, LOCK_UN);
        throw runtime_error("SHM ID changed");
    }
    // Serialize
    msg.origin = _node_id;
    msg.creation = chrono::duration_cast<chrono::duration<double>>(chrono::high_resolution_clock::now().time_since_epoch()).count();
    string serialized;
    double creation_val = msg.creation.value_or(0.0);
    serialized.append(reinterpret_cast<const char*>(&creation_val), sizeof(double));
    size_t len = msg.origin.has_value() ? msg.origin->name.size() : 0;
    serialized.append(reinterpret_cast<const char*>(&len), sizeof(size_t));
    if (len > 0) serialized += msg.origin->name;
    len = msg.data.size();
    serialized.append(reinterpret_cast<const char*>(&len), sizeof(size_t));
    serialized += msg.data;
    if (serialized.size() > _channel.msg_size) {
        flock(_lock_fd, LOCK_UN);
        throw runtime_error("Message too large");
    }
    memcpy(_shm_ptr, serialized.data(), serialized.size());
    memset(static_cast<char*>(_shm_ptr) + serialized.size(), 0, _channel.msg_size - serialized.size());
    flock(_lock_fd, LOCK_UN);
}

void Publisher::close() {
    flock(_lock_fd, LOCK_EX);
    if (_shm_ptr) munmap(_shm_ptr, _channel.msg_size);
    close_shm(_shm_fd, _channel.name());
    if (_lock_fd != -1) ::close(_lock_fd);
    flock(_lock_fd, LOCK_UN);
}

// Subscriber implementation
Subscriber::Subscriber(const NodeID& node_id, const ChannelSpec& channel, function<void(const BaseMessage&)> callback)
    : _node_id(node_id), _channel(channel), _callback(std::move(callback)), _running(true) {
    _shm_fd = init_shm(_channel);
    _shm_ptr = mmap(nullptr, _channel.msg_size, PROT_READ | PROT_WRITE, MAP_SHARED, _shm_fd, 0);
    if (_shm_ptr == MAP_FAILED) throw runtime_error("mmap failed");
    _shm_id = get_shm_id(_shm_fd);
    _lock_fd = gen_lock(_channel.name());
    _last_msg = get_msg();
}

Subscriber::~Subscriber() {
    if (_shm_ptr) munmap(_shm_ptr, _channel.msg_size);
    if (_shm_fd != -1) ::close(_shm_fd);
}

optional<BaseMessage> Subscriber::wait_for_message() {
    while (_running) {
        flock(_lock_fd, LOCK_EX);
        int cur_shm_id = get_shm_id(_shm_fd);
        if (cur_shm_id == -1) {
            _running = false;
            flock(_lock_fd, LOCK_UN);
            return nullopt;
        }
        if (cur_shm_id != _shm_id) {
            flock(_lock_fd, LOCK_UN);
            throw runtime_error("SHM ID changed");
        }
        auto msg = get_msg();
        flock(_lock_fd, LOCK_UN);
        if (!msg || (_last_msg.has_value() && *msg == *_last_msg)) {
            this_thread::sleep_for(POLL_INTERVAL);
        } else {
            _last_msg = msg;
            return msg;
        }
    }
    return nullopt;
}

void Subscriber::listen() {
    if (!_callback) throw runtime_error("Callback required for listen");
    _listen_thread = thread([this]() {
        while (auto msg = wait_for_message()) {
            _callback(*msg);
            this_thread::sleep_for(chrono::duration<double>(0.0));
        }
    });
}

void Subscriber::close() {
    _running = false;
    if (_listen_thread.joinable()) _listen_thread.join();
    flock(_lock_fd, LOCK_EX);
    if (_shm_ptr) munmap(_shm_ptr, _channel.msg_size);
    close_shm(_shm_fd, _channel.name());
    if (_lock_fd != -1) ::close(_lock_fd);
    flock(_lock_fd, LOCK_UN);
}

optional<BaseMessage> Subscriber::get_msg() {
    try {
        char* buf = static_cast<char*>(_shm_ptr);
        BaseMessage msg;
        size_t offset = 0;
        double creation_val = 0.0;
        if (offset + sizeof(double) > _channel.msg_size) return nullopt;
        memcpy(&creation_val, buf + offset, sizeof(double));
        if (creation_val == 0.0) return nullopt; // treat empty
        msg.creation = creation_val;
        offset += sizeof(double);
        size_t len;
        if (offset + sizeof(size_t) > _channel.msg_size) return nullopt;
        memcpy(&len, buf + offset, sizeof(size_t));
        offset += sizeof(size_t);
        if (len > 0 && len < _channel.msg_size && offset + len <= _channel.msg_size) {
            msg.origin = NodeID{std::string(buf + offset, len)};
        }
        offset += len;
        if (offset + sizeof(size_t) > _channel.msg_size) return nullopt;
        memcpy(&len, buf + offset, sizeof(size_t));
        offset += sizeof(size_t);
        if (len > 0 && len < _channel.msg_size && offset + len <= _channel.msg_size) {
            msg.data.assign(buf + offset, len);
        } else {
            return nullopt;
        }
        return msg;
    } catch (...) {
        return nullopt;
    }
}

// Helper functions
static int get_shm_id(int shm_fd) {
    struct stat st;
    if (fstat(shm_fd, &st) == -1) return -1;
    return st.st_ino;
}

static void close_shm(int shm_fd, const string& shm_name) {
    ::close(shm_fd);
    std::string path = LOCK_DIR + std::string("ipc_shm_") + shm_name;
    ::unlink(path.c_str());
}

static int init_shm(ChannelSpec& channel) {
    std::string path = LOCK_DIR + std::string("ipc_shm_") + channel.name();
    int fd = ::open(path.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd == -1) throw runtime_error("open shm file failed");
    if (ftruncate(fd, static_cast<off_t>(channel.msg_size)) == -1) {
        struct stat st;
        if (fstat(fd, &st) == 0 && st.st_size > 0) {
            channel.msg_size = static_cast<size_t>(st.st_size);
        } else {
            ::close(fd);
            ::unlink(path.c_str());
            throw runtime_error("ftruncate failed on shm file");
        }
    }
    return fd;
}

static int gen_lock(const string& shm_name) {
    string lock_path = LOCK_DIR + shm_name + LOCK_SUFFIX;
    int fd = open(lock_path.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd == -1) throw runtime_error("open lock failed");
    return fd;
}
