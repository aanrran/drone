import cv2
import pygame
import tkinter as tk
import socket
from threading import Thread
from PIL import Image, ImageTk
import struct
import crc8
import csv


class VideoStreamApp:
    def __init__(self, master, stream_url, tcp_ip, tcp_port):
        self.master = master
        self.stream_url = stream_url
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.threshold = 0.1  # Sensitivity threshold

        # Set up the main window
        self.master.title("Video Stream with Joystick Status")
        self.master.geometry("800x600")

        # Create a label to display the video stream
        self.video_label = tk.Label(self.master)
        self.video_label.pack()

        # Create a label to display the joystick status
        self.status_label = tk.Label(self.master, text="Joystick Status", font=("Helvetica", 16))
        self.status_label.pack()

        # Open a video capture from the stream URL
        self.cap = cv2.VideoCapture(self.stream_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)  # Set buffer size to minimum

        # Initialize Pygame for joystick support
        pygame.init()
        pygame.joystick.init()

        # Check if a joystick is connected and initialize it
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

        # Set up a TCP socket for sending joystick data
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_to_server()

        # Send CSV data a few times to initialize the drone
        for _ in range(3):  # Adjust the number of times as needed
            self.send_csv_data()

        # Start the frame update and joystick update using `after`
        self.master.after(0, self.update_frame)
        self.master.after(0, self.update_joystick_status)

    def connect_to_server(self):
        """Attempt to connect to the TCP server with retry logic."""
        attempts = 0
        while attempts < 5:
            try:
                self.socket.connect((self.tcp_ip, self.tcp_port))
                print("Connected to server.")
                return
            except ConnectionRefusedError:
                attempts += 1
                print(f"Connection attempt {attempts} failed. Retrying in 2 seconds...")
                self.master.after(2000, self.connect_to_server)
        print("Failed to connect to server after multiple attempts.")
        self.master.quit()  # Exit the application if connection fails

    def update_frame(self):
        """Update the video frame from the stream."""
        ret, frame = self.cap.read()
        if ret:
            # Rotate the frame by 90 degrees counter-clockwise
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # Convert the frame to RGB
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)

        self.master.after(10, self.update_frame)  # Schedule the next frame update

    def update_joystick_status(self):
        """Update the joystick status and send data if there's any movement."""
        if self.joystick:
            pygame.event.pump()
            x_axis_1 = -self.joystick.get_axis(0)  # First joystick X-axis (invert sign)
            y_axis_1 = -self.joystick.get_axis(1)  # First joystick Y-axis (invert sign)
            x_axis_2 = -self.joystick.get_axis(2)  # Second joystick X-axis (invert sign)
            y_axis_2 = -self.joystick.get_axis(3)  # Second joystick Y-axis (invert sign)

            # Scale the joystick values to range from -10 to 10
            x_axis_1_int = int(x_axis_1 * 10)
            y_axis_1_int = int(y_axis_1 * 10)
            x_axis_2_int = int(x_axis_2 * 10)
            y_axis_2_int = int(y_axis_2 * 10)

            # Update the status label with the joystick readings
            self.status_label.config(
                text=f"Joystick 1: X={x_axis_1_int}, Y={y_axis_1_int} | Joystick 2: X={x_axis_2_int}, Y={y_axis_2_int}")

            # Check if any axis exceeds the threshold
            if (abs(x_axis_1) > self.threshold or abs(y_axis_1) > self.threshold or
                    abs(x_axis_2) > self.threshold or abs(y_axis_2) > self.threshold):
                # Pack data as 8-bit integers
                packed_data = struct.pack('4b', x_axis_1_int, y_axis_1_int, x_axis_2_int, y_axis_2_int)

                # Calculate CRC-8
                crc = crc8.crc8()
                crc.update(packed_data)
                crc_value = crc.digest()[0]
                message = packed_data + struct.pack('B', crc_value)

                # Send the data
                self.socket.sendall(message)
                print(f"Sent: {x_axis_1_int}, {y_axis_1_int}, {x_axis_2_int}, {y_axis_2_int}, CRC: {crc_value}")

            if self.joystick.get_button(2):  # X button 
                print("X button pressed.")
                self.command_drone_restart()
            if self.joystick.get_button(1):  # B button
                print("B button pressed.")
                self.send_csv_data()

        else:
            # Update the status label if no joystick is connected
            self.status_label.config(text="No joystick connected")

        self.master.after(50, self.update_joystick_status)  # Schedule the next joystick update

    def command_drone_restart(self):
        """Send a command to the drone to restart."""
        command = b'RESTART'
        self.socket.sendall(command)
        print("Sent: RESTART")

    def send_csv_data(self):
        """Read from a CSV file and send the data to the drone."""
        filename = 'parameters.csv'  # Replace with your CSV file path
        try:
            with open(filename, mode='r') as file:
                reader = csv.DictReader(file)  # Use DictReader to read the header row
                for row in reader:
                    try:
                        data = [
                            float(row['Kp_roll']),
                            float(row['Ki_roll']),
                            float(row['Kd_roll']),
                            float(row['Kp_pitch']),
                            float(row['Ki_pitch']),
                            float(row['Kd_pitch']),
                            float(row['Kp_yaw']),
                            float(row['Ki_yaw']),
                            float(row['Kd_yaw']),
                            float(row['integral_max'])
                        ]
                        packed_data = struct.pack('10f', *data)

                        # Calculate CRC-8
                        crc = crc8.crc8()
                        crc.update(packed_data)
                        crc_value = crc.digest()[0]
                        message = packed_data + struct.pack('B', crc_value)

                        # Send the data
                        self.socket.sendall(message)
                        print(f"Sent CSV data: {data}, CRC: {crc_value}")
                    except KeyError as e:
                        print(f"Missing parameter in CSV: {e}")
                    except ValueError as e:
                        print(f"Invalid data in CSV: {e}")
        except FileNotFoundError:
            print(f"CSV file {filename} not found.")

    def close(self):
        """Close the application, releasing the resources."""
        self.cap.release()  # Release the video capture
        self.socket.close()  # Close the TCP socket
        self.master.destroy()  # Close the GUI window


if __name__ == "__main__":
    root = tk.Tk()
    stream_url = "http://192.168.50.149/"  # Replace with your ESP32 stream URL
    tcp_ip = "192.168.50.149"  # Replace with your ESP32 IP address
    tcp_port = 3333  # Replace with your TCP port

    # Create and run the video stream application
    app = VideoStreamApp(root, stream_url, tcp_ip, tcp_port)
    root.protocol("WM_DELETE_WINDOW", app.close)  # Ensure proper cleanup on close
    root.mainloop()
