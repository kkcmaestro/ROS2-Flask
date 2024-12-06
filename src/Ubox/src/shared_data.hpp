#ifndef SHARED_DATA_HPP
#define SHARED_DATA_HPP

#include <mutex>
#include <iostream>
#include <chrono>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

// Shared data structure to be used by both scripts
struct SharedData {
    double target_left_wheel_rpm;
    double target_right_wheel_rpm;
    double input_linear_velocity;
    double input_angular_velocity;
    std::int64_t timestamp_ms;

    // We will not use std::mutex here, as we rely on mmap for synchronization
};

// Constants for shared memory
const char* SHARED_MEMORY_NAME = "/shared_data";  // Name of shared memory object
const size_t SHARED_MEMORY_SIZE = sizeof(SharedData);  // Size of shared memory object

#endif  // SHARED_DATA_HPP
