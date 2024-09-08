#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
import spacy
import speech_recognition as sr
import pyttsx3

class CommandSender(Node):

    def __init__(self):
        super().__init__('command_sender')
        
        self.publisher_ = self.create_publisher(
            String,
            'goal_command',
            QoSProfile(depth=10)
        )

        # Load the trained model
        self.nlp = spacy.load("../model/textcat_model")

        # Speech recognition setup
        self.r = sr.Recognizer()
        self.engine = pyttsx3.init()

        self.locations = {
            "OFFICE": (2.069616, 0.102869, 0.0),
            "LIBRARY": (0.53185, 2.01191, 0.0),
            "GYM": (-0.38, -1.76, 0.0),
            # Add more locations here
        }

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command to goal_command topic: {command}")

    def classify_text(self, text):
        doc = self.nlp(text)
        return max(doc.cats, key=doc.cats.get)

    def SpeakText(self, command):
        try:
            self.engine.say(command)
            self.engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"Error speaking text: {e}")

    def listen_and_process(self):
        try:
            with sr.Microphone() as source2:
                self.get_logger().info("Adjusting for ambient noise...")
                self.r.adjust_for_ambient_noise(source2, duration=0.3)  # Reduced duration
                self.get_logger().info("Listening...")
                audio2 = self.r.listen(source2, timeout=5, phrase_time_limit=5)  # Added time limits
                self.get_logger().info("Processing audio...")
                MyText = self.r.recognize_google(audio2,language="hi-IN")
                MyText = MyText.lower()
                self.get_logger().info(f"Did you say: {MyText}")
                self.SpeakText(MyText)
                command = self.classify_text(MyText)
                self.get_logger().info(f"Command recognized: {command}")
                self.process_command(command)
        except sr.WaitTimeoutError:
            self.get_logger().info("Listening timed out, please try again.")
        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results; {e}")
        except sr.UnknownValueError:
            self.get_logger().error("Speech recognition could not understand the audio")

    def process_command(self, command):
        if command.startswith("GO TO"):
            location_name = command.split("GO TO")[1].strip()
            if location_name in self.locations:
                x, y, theta = self.locations[location_name]
                full_command = f"tb1 {x} {y} {theta}"
                self.send_command(full_command)
            else:
                self.get_logger().info(f"Unknown location: {location_name}")
        else:
            self.get_logger().info("Invalid command format. Use 'go to <location>'")

def main(args=None):
    rclpy.init(args=args)
    node = CommandSender()

    try:
        while rclpy.ok():
            node.listen_and_process()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

