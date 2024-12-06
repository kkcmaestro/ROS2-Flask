from flask import Flask, request, jsonify
import matplotlib.pyplot as plt
import threading
import time

app = Flask(__name__)


received_data = []

@app.route('/send_shared_data', methods=['POST'])
def receive_data():
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "Invalid data"}), 400

       
        target_left_wheel_rpm = data.get('target_left_wheel_rpm')
        target_right_wheel_rpm = data.get('target_right_wheel_rpm')
        input_linear_velocity = data.get('input_linear_velocity')
        input_angular_velocity = data.get('input_angular_velocity')
        timestamp_ms = data.get('timestamp_ms')

        
        received_data.append({
            'target_left_wheel_rpm': target_left_wheel_rpm,
            'target_right_wheel_rpm': target_right_wheel_rpm,
            'input_linear_velocity': input_linear_velocity,
            'input_angular_velocity': input_angular_velocity,
            'timestamp_ms': timestamp_ms
        })

        print(f"Received data: {data}")
        return jsonify({"status": "Data received successfully"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/get_received_data', methods=['GET'])
def get_received_data():
    return jsonify(received_data)

def plot_data_periodically():
    plt.ion()  
    fig, ax = plt.subplots()

    while True:
        if received_data:
            
            timestamps = [entry['timestamp_ms'] for entry in received_data]
            left_rpms = [entry['target_left_wheel_rpm'] for entry in received_data]
            right_rpms = [entry['target_right_wheel_rpm'] for entry in received_data]

            # Clear and update the plot
            ax.clear()
            ax.plot(timestamps, left_rpms, label="Left Wheel RPM", marker='o')
            ax.plot(timestamps, right_rpms, label="Right Wheel RPM", marker='x')
            ax.set_xlabel('Timestamp (ms)')
            ax.set_ylabel('RPM')
            ax.set_title('Wheel RPM vs Time')
            ax.legend()
            ax.grid(True)

            # Redraw the plot
            plt.pause(1)  

        time.sleep(1)  

if __name__ == '__main__':
    
    threading.Thread(target=plot_data_periodically, daemon=True).start()

    app.run(debug=True, host='0.0.0.0', port=5000)
