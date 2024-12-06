#include <iostream>
#include <chrono>
#include <cstring>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include "shared_data.h"

constexpr const char *SHARED_MEM_NAME = "/shared_data";
constexpr size_t SHARED_MEM_SIZE = sizeof(SharedData);

int main() {
    // Create shared memory
    int shm_fd = shm_open(SHARED_MEM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open failed");
        return 1;
    }

    // Resize shared memory
    if (ftruncate(shm_fd, SHARED_MEM_SIZE) == -1) {
        perror("ftruncate failed");
        return 1;
    }

    // Map shared memory
    void *ptr = mmap(nullptr, SHARED_MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap failed");
        return 1;
    }

    // Cast to shared data structure
    SharedData *shared_data = static_cast<SharedData *>(ptr);

    while (true) {
        // Simulate data update
        auto now = std::chrono::system_clock::now();
        auto epoch = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch();
        std::int64_t timestamp_ms = epoch.count();

        {
            // Lock mutex and update shared data
            std::lock_guard<std::mutex> lock(shared_data->mutex);
            shared_data->target_left_wheel_rpm = 50.0;
            shared_data->target_right_wheel_rpm = 55.0;
            shared_data->input_linear_velocity = 1.5; // m/s
            shared_data->input_angular_velocity = 0.2; // rad/s
            shared_data->timestamp_ms = timestamp_ms;
        }

        std::cout << "Script A: Data updated\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
    }

    // Cleanup
    munmap(ptr, SHARED_MEM_SIZE);
    shm_unlink(SHARED_MEM_NAME);

    return 0;
}
