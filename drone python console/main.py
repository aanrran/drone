import cv2
import pygame
from PIL import Image, ImageTk
import tkinter as tk
import socket
import threading

class VideoStreamApp:
    def __init__(self, master, stream_url, tcp_ip, tcp_port):
        self.master = master
        self.stream_url = stream_url
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.threshold = 0.01  # Sensitivity threshold
        self.brightness_threshold = 100  # Adjust this value based on your needs
        self.frame_counter = 0

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
        self.update_frame()

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
        self.socket.connect((self.tcp_ip, self.tcp_port))

        # Start the joystick polling in a separate thread
        self.joystick_thread = threading.Thread(target=self.poll_joystick)
        self.joystick_thread.daemon = True
        self.joystick_thread.start()

    def update_frame(self):
        """Update the video frame from the stream."""
        ret, frame = self.cap.read()
        if ret:
            # Convert the frame to RGB and update the label
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)

        self.master.after(5, self.update_frame)

    def poll_joystick(self):
        """Poll the joystick for input and update the status label and socket."""
        while True:
            if self.joystick:
                pygame.event.pump()
                x_axis_1 = self.joystick.get_axis(0)  # First joystick X-axis
                y_axis_1 = self.joystick.get_axis(1)  # First joystick Y-axis
                x_axis_2 = self.joystick.get_axis(2)  # Second joystick X-axis
                y_axis_2 = self.joystick.get_axis(3)  # Second joystick Y-axis

                # Only send joystick data if any axis value is above the threshold
                if (abs(x_axis_1) > self.threshold or abs(y_axis_1) > self.threshold or
                        abs(x_axis_2) > self.threshold or abs(y_axis_2) > self.threshold):
                    message = f"{x_axis_1} {y_axis_1} {x_axis_2} {y_axis_2}\n"
                    self.socket.sendall(message.encode())

                # Update the status label with the current joystick positions
                status_text = f"Joystick 1 - X: {x_axis_1:.2f}, Y: {y_axis_1:.2f}\n"
                status_text += f"Joystick 2 - X: {x_axis_2:.2f}, Y: {y_axis_2:.2f}"
                self.master.after(0, self.status_label.config, {'text': status_text})

            pygame.time.wait(10)  # Wait for 10 milliseconds before polling again

    def close(self):
        """Close the application, releasing the resources."""
        self.cap.release()  # Release the video capture
        self.socket.close()  # Close the TCP socket
        self.master.destroy()  # Close the GUI window


if __name__ == "__main__":
    root = tk.Tk()
    stream_url = "http://192.168.50.141/"  # Replace with your ESP32 stream URL
    tcp_ip = "192.168.50.141"  # Replace with your ESP32 IP address
    tcp_port = 3333  # Replace with your TCP port

    # Create and run the video stream application
    app = VideoStreamApp(root, stream_url, tcp_ip, tcp_port)
    root.protocol("WM_DELETE_WINDOW", app.close)  # Ensure proper cleanup on close
    root.mainloop()
