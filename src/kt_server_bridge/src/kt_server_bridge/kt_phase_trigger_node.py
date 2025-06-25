#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
import threading

class KtPhaseTriggerNode(Node):
    def __init__(self):
        super().__init__('KtPhaseTriggerNode')
        
        # Create publisher for string messages
        self.publisher = self.create_publisher(String, 'demo_phase', 10)
        
        # Button values mapping with your specified values
        self.button_values = {
            'Pre Manual': 'pre_manual',
            'Manual Started': 'manual_started', 
            'Manual Stopped': 'manual_stopped',
            'Auto Started': 'auto_started',
            'Auto Stopped': 'auto_stopped'
        }
        
        # Current message value (default to first button)
        self.current_message = list(self.button_values.values())[0]
        
        # Initialize button tracking attributes BEFORE init_gui()
        self.selected_button = None
        self.last_bg_color = None
        
        # Create timer for publishing at 1Hz
        self.timer = self.create_timer(1.0, self.publish_message)
        
        # Initialize GUI
        self.init_gui()
        
    def init_gui(self):
        """Initialize the Tkinter GUI"""
        self.root = tk.Tk()
        self.root.title("KT Phase Trigger Controller")
        self.root.geometry("500x350")
        
        # Create title label
        title_label = tk.Label(self.root, text="KT Phase Trigger Controller", 
                              font=("Arial", 16, "bold"))
        title_label.pack(pady=10)
        
        # Create buttons frame
        buttons_frame = tk.Frame(self.root)
        buttons_frame.pack(pady=20)
        
        # Create buttons
        self.buttons = {}
        for i, (button_name, value) in enumerate(self.button_values.items()):
            btn = tk.Button(
                buttons_frame,
                text=f"{button_name}\n({value})",
                width=15,
                height=3,
                font=("Arial", 10),
                bg="lightgray",
                fg="black",
                relief="raised",
                command=lambda v=value, b=button_name: self.button_clicked(v, b)
            )
            # Arrange buttons in 1 column with 1 row
            # btn.grid(row=i, column=0, padx=5, pady=5)
            if i < 3:
                btn.grid(row=0, column=i, padx=5, pady=5)
            else:
                btn.grid(row=1, column=i-3, padx=5, pady=5)
            self.buttons[button_name] = btn
        
        # Highlight the first button by default
        first_button_name = list(self.button_values.keys())[0]
        self.highlight_button(self.buttons[first_button_name])
        
        # Status label
        self.status_label = tk.Label(self.root, 
                                   text=f"Current Mode: {self.current_message}",
                                   font=("Arial", 12),
                                   fg="blue")
        self.status_label.pack(pady=15)
        
        # Info label
        info_label = tk.Label(self.root, 
                            text="Publishing at 1Hz to /demo_phase topic",
                            font=("Arial", 10),
                            fg="gray")
        info_label.pack(pady=5)
        
        # Set up window close protocol
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def button_clicked(self, value, button_name):
        """Handle button click events"""
        self.current_message = value
        self.highlight_button(self.buttons[button_name])
        self.status_label.config(text=f"Current Mode: {self.current_message}")
        self.get_logger().info(f'Phase changed: {button_name} -> {value}')
        
    def highlight_button(self, button):
        """Highlight the selected button and reset previous selection"""
        # Reset previous button color
        if self.selected_button is not None:
            self.selected_button.config(bg="lightgray")
        
        # Highlight new button
        self.selected_button = button
        button.config(bg="orange")
        
    def publish_message(self):
        """Publish the current message at 1Hz"""
        msg = String()
        msg.data = self.current_message
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing phase: "{msg.data}"')
        
    def on_closing(self):
        """Handle window closing"""
        self.get_logger().info('Shutting down KT Phase Trigger GUI...')
        self.destroy_node()
        rclpy.shutdown()
        self.root.destroy()
        
    def run_gui(self):
        """Run the Tkinter main loop"""
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = KtPhaseTriggerNode()
    
    # Run ROS2 spinning in a separate thread
    def spin_ros():
        try:
            rclpy.spin(node)
        except:
            pass
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    # Run the GUI in the main thread
    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
