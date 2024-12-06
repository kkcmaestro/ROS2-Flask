#include <iostream>
#include <chrono>
#include <thread>
#include "shared_data.hpp"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>  // Include JSONCPP library


constexpr const char* FLASK_URL = "http://127.0.0.1:5000/send_shared_data"; 

class SharedDataSubscriber {
public:
    SharedDataSubscriber(SharedData* shared_data)
        : shared_data_(shared_data) {}

    void fetchAndSendData() {
        while (true) {
            // Read data from shared memory
            double left_rpm = shared_data_->target_left_wheel_rpm;
            double right_rpm = shared_data_->target_right_wheel_rpm;
            double linear_velocity = shared_data_->input_linear_velocity;
            double angular_velocity = shared_data_->input_angular_velocity;
            std::int64_t timestamp_ms = shared_data_->timestamp_ms;

            // Create a JSON object using JSONCPP
            Json::Value jsonData;
            jsonData["target_left_wheel_rpm"] = left_rpm;
            jsonData["target_right_wheel_rpm"] = right_rpm;
            jsonData["input_linear_velocity"] = linear_velocity;
            jsonData["input_angular_velocity"] = angular_velocity;
            jsonData["timestamp_ms"] = timestamp_ms;

            // Convert the JSON object to a string
            Json::StreamWriterBuilder writer;
            std::string jsonString = Json::writeString(writer, jsonData);

            // Send the data to the Flask server via POST request
            sendPostRequest(jsonString);

            // Print the fetched data
            std::cout << "Sent Data to Flask: " << jsonString << std::endl;

            // Sleep for 100ms 
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    SharedData* shared_data_;

    
    void sendPostRequest(const std::string& jsonString) {
        CURL* curl;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl = curl_easy_init();

        if (curl) {
            
            curl_easy_setopt(curl, CURLOPT_URL, FLASK_URL);

            // Set the HTTP request to POST
            curl_easy_setopt(curl, CURLOPT_POST, 1L);

            // Set the JSON data as the POST body
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonString.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, jsonString.size());

            
            struct curl_slist* headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            
            res = curl_easy_perform(curl);

            if (res != CURLE_OK) {
                std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            }

            
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }

        curl_global_cleanup();
    }
};

int main() {
    // Open shared memory object
    int shm_fd = shm_open(SHARED_MEMORY_NAME, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("Failed to open shared memory");
        return -1;
    }

    // Map the shared memory object into memory
    SharedData* shared_data = static_cast<SharedData*>(mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (shared_data == MAP_FAILED) {
        perror("Failed to map shared memory");
        return -1;
    }

    // Initialize the subscriber and fetch/send data
    SharedDataSubscriber subscriber(shared_data);
    subscriber.fetchAndSendData();

    // Unmap and close the shared memory when done
    munmap(shared_data, SHARED_MEMORY_SIZE);
    close(shm_fd);

    return 0;
}


