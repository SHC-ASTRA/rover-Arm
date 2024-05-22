import rclpy
from rclpy.node import Node
import serial
import sys
import threading
import glob
import time
import atexit
import signal
from std_msgs.msg import String
from interfaces_pkg.msg import ControllerState
from interfaces_pkg.msg import ArmState

serial_pub = None
thread = None

class SerialRelay(Node):
    def __init__(self):
        # Initialize node with name
        super().__init__("arm_relay")

        # Create publishers
        self.output_publisher = self.create_publisher(String, '/astra/arm/feedback', 10)
        self.state_publisher = self.create_publisher(ArmState, '/astra/arm/state', 10)

        # Create subscriber
        self.subscriber = self.create_subscription(ControllerState, '/astra/arm/control', self.send_controls, 10)

        # Loop through all serial devices on the computer to check for the MCU
        self.port = None
        ports = SerialRelay.list_serial_ports()
        for port in ports:
            try:
                ser = serial.Serial(port, timeout=1.0)
                ser.write(b"arm,ping\n")
                response = ser.read_until("\n")
                if b"pong" in response:
                    self.port = port
                    print(f"Found MCU at {self.port}!")
                    break
            except:
                pass
        
        if self.port is None:
            print("Unable to find MCU... please make sure it is connected.")
            sys.exit(1)
        
        self.ser = serial.Serial(self.port, 115200)
        atexit.register(self.cleanup)

    def run(self):
        global thread
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()
        
        try:
            while rclpy.ok():
                if self.ser.in_waiting:
                    self.read_mcu()
                else:
                    time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def read_mcu(self):
        try:
            output = str(self.ser.readline(), "utf8")
            if output:
                print(f"[MCU] {output}", end="")
                msg = String()
                msg.data = output
                self.output_publisher.publish(msg)
        except serial.SerialException:
            pass

    def send_controls(self, msg):
        command = ""
        ef_cmd = ""
        if(msg.b):
            command = "arm,stop\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
            return 
        
        if(msg.plus):
            command = "arm,endEffect,laser,1\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
        elif(msg.minus):
            command = "arm,endEffect,laser,0\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
        
        if(msg.rb):
            ef_cmd = "arm,endEffect,ctrl,"
            if(msg.lt >= 0.5):
                ef_cmd += "-1,"
            elif(msg.rt >= 0.5):
                ef_cmd += "1,"
            else:
                ef_cmd += "0,"

            if(msg.rs_x < 0):
                ef_cmd += "-1,"
            elif(msg.rs_x > 0):
                ef_cmd += "1,"
            else:
                ef_cmd += "0,"
            if(msg.ls_x < 0):
                ef_cmd += "-1"
            elif(msg.ls_x > 0):
                ef_cmd += "1"
            else:
                ef_cmd += "0"

            command = ef_cmd + "\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
            return
        else:
            ef_cmd = "arm,endEffect,ctrl,"
            if(msg.lt >= 0.5):
                ef_cmd += "-1,0,0"
            elif(msg.rt >= 0.5):
                ef_cmd += "1,0,0"
            else:
                ef_cmd += "0,0,0"
            command = ef_cmd + "\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
        
        if(msg.lb):
            #self.ser.write(bytes("arm,setMode,manual\n", "utf8"))
            command = "arm,man,0.25,"
            if(msg.d_left):
                command += "-1,"
            elif(msg.d_right):
                command += "1,"
            else:
                command += "0,"
            if(msg.ls_x < -0.5):
                command += "-1,"
            elif(msg.ls_x > 0.5):
                command += "1,"
            else:
                command += "0,"
            if(msg.ls_y < -0.5):
                command += "1,"
            elif(msg.ls_y > 0.5):
                command += "-1,"
            else:
                command += "0,"
            if(msg.rs_y < -0.5):
                command += "1"
            elif(msg.rs_y > 0.5):
                command += "-1"
            else:
                command += "0"

            command += "\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
            return
        else:
            #self.ser.write(bytes("arm,setMode,manual\n", "utf8"))
            command = "arm,man,0.15,"
            if(msg.d_left):
                command += "-1,"
            elif(msg.d_right):
                command += "1,"
            else:
                command += "0,"
            if(msg.ls_x < -0.5):
                command += "-1,"
            elif(msg.ls_x > 0.5):
                command += "1,"
            else:
                command += "0,"
            if(msg.ls_y < -0.5):
                command += "1,"
            elif(msg.ls_y > 0.5):
                command += "-1,"
            else:
                command += "0,"
            if(msg.rs_y < -0.5):
                command += "1"
            elif(msg.rs_y > 0.5):
                command += "-1"
            else:
                command += "0"

            command += "\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
            return

    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/tty[A-Za-z]*")

    def cleanup(self):
        print("Cleaning up...")
        if self.ser.is_open:
            self.ser.close()

def myexcepthook(type, value, tb):
    print("Uncaught exception:", type, value)
    if serial_pub:
        serial_pub.cleanup()

def main(args=None):
    rclpy.init(args=args)
    sys.excepthook = myexcepthook

    global serial_pub
    serial_pub = SerialRelay()
    serial_pub.run()

if __name__ == '__main__':
    signal.signal(signal.SIGTSTP, lambda signum, frame: sys.exit(0))  # Catch Ctrl+Z and exit cleanly
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))  # Catch termination signals and exit cleanly
    main()
