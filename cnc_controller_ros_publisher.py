#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import threading
import time

class CNCCommandPublisher(Node):
    def __init__(self):
        super().__init__('cnc_command_publisher')
        
        # Create publisher for CNC commands
        self.publisher_ = self.create_publisher(String, 'cnc_commands', 10)
        
        self.get_logger().info('CNC Command Publisher started')
        self.get_logger().info('Available commands:')
        self.print_help()

    def publish_command(self, command):
        """Publish a command to the CNC controller."""
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published command: "{command}"')

    def print_help(self):
        """Print available commands."""
        help_text = """
            Available commands:
            s           - Query status
            u           - Unlock CNC
            mh          - Move to machine home
            h           - Go to work home
            sethome     - Set current position as work home
            p           - Play sound
            gp          - Get current position
            a x,y,z     - Absolute move (e.g., 'a 10,5,2')
            r x,y,z     - Relative move (e.g., 'r 1,0,0')
            j           - Enter jog mode
            test        - Run CNC test only
            test e.csv a.csv - Run CNC test + inductance script with files
            test l,s,w,z - Test mode with parameters (e.g., 'test 4,10,2,0')
            rotate test - Test rotation only
            kill_test   - Kill running inductance test script
            feed rate   - Set feed rate (e.g., 'feed 800')
            help        - Show this help
            quit        - Exit publisher
        """
        print(help_text)

    def interactive_mode(self):
        """Run in interactive mode where user can type commands."""
        print("\n=== CNC Command Publisher - Interactive Mode ===")
        print("Type commands to send to CNC controller (type 'help' for commands, 'quit' to exit):")
        
        while rclpy.ok():
            try:
                command = input("\nCNC> ").strip()
                
                if not command:
                    continue
                    
                
                if command.lower() in ['quit', 'exit', 'q']:
                    self.get_logger().info('Exiting interactive mode')
                    break
                    
                if command.lower() == 'help':
                    self.print_help()
                    continue
                    
                self.publish_command(command)
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                self.get_logger().error(f'Error in interactive mode: {e}')

    def batch_mode(self, commands):
        """Send a list of commands in batch mode."""
        self.get_logger().info(f'Running batch mode with {len(commands)} commands')
        
        for i, command in enumerate(commands, 1):
            if not rclpy.ok():
                break
                
            self.get_logger().info(f'Batch command {i}/{len(commands)}: {command}')
            self.publish_command(command)
            
            # Small delay between commands to avoid overwhelming the subscriber
            time.sleep(0.1)
        
        self.get_logger().info('Batch mode completed')

class CNCSequencePublisher(CNCCommandPublisher):
    """Extended publisher with predefined sequences and autonomous operation capabilities."""
    
    def __init__(self):
        super().__init__()
        self.get_logger().info('CNC Sequence Publisher initialized with predefined sequences')

    def sequence_startup(self):
        """Startup sequence to prepare CNC machine."""
        sequence = [
            "u",        # Unlock
            "mh",       # Move to machine home
            "feed 500", # Set moderate feed rate
            "s"         # Check status
        ]
        self.get_logger().info('Running startup sequence')
        self.batch_mode(sequence)

    def sequence_calibration_test(self):
        """Run a calibration test sequence."""
        sequence = [
            "sethome",      # Set current position as home
            "feed 200",     # Set slow feed rate for testing
            "r 1,0,0",      # Move +1mm in X
            "r -1,0,0",     # Return to start X
            "r 0,1,0",      # Move +1mm in Y
            "r 0,-1,0",     # Return to start Y
            "r 0,0,1",      # Move +1mm in Z
            "r 0,0,-1",     # Return to start Z
            "p"             # Play completion sound
        ]
        self.get_logger().info('Running calibration test sequence')
        self.batch_mode(sequence)

    def sequence_square_pattern(self, size=10):
        """Draw a square pattern."""
        sequence = [
            "sethome",           # Set starting point
            "feed 300",          # Set moderate speed
            f"r {size},0,0",     # Move right
            f"r 0,{size},0",     # Move up
            f"r -{size},0,0",    # Move left
            f"r 0,-{size},0",    # Move down (back to start)
            "p"                  # Play completion sound
        ]
        self.get_logger().info(f'Running square pattern sequence (size: {size}mm)')
        self.batch_mode(sequence)

    def sequence_rotate_test(self):
        """Test rotation functionality."""
        sequence = [
            "rotate test",  # Run rotation test
            "p"            # Play completion sound
        ]
        self.get_logger().info('Running rotation test sequence')
        self.batch_mode(sequence)

    def sequence_inductance_test(self, evm_file, ati_file):
        """Run inductance test with CNC rotation and bash script."""
        sequence = [
            "sethome",                    # Set home position
            f"test {evm_file} {ati_file}" # Run CNC test + inductance script
        ]
        self.get_logger().info(f'Running inductance test sequence with files: {evm_file}, {ati_file}')
        self.batch_mode(sequence)
        """Run autonomous scanning pattern."""
        sequence = [
            "sethome",                                    # Set home position
            "feed 100",                                   # Set slow scan speed
            f"test {length},{segments},0.5,{z_offset}",   # Run test pattern
            "h",                                          # Return to home
            "p"                                          # Play completion sound
        ]
        self.get_logger().info(f'Running autonomous scan pattern')
        self.batch_mode(sequence)

def main(args=None):
    rclpy.init(args=args)
    
    # Check command line arguments
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        
        if mode == 'sequence':
            # Sequence mode with predefined operations
            node = CNCSequencePublisher()
            
            if len(sys.argv) > 2:
                sequence_type = sys.argv[2].lower()
                
                if sequence_type == 'startup':
                    node.sequence_startup()
                elif sequence_type == 'calibration':
                    node.sequence_calibration_test()
                elif sequence_type == 'square':
                    size = float(sys.argv[3]) if len(sys.argv) > 3 else 10
                    node.sequence_square_pattern(size)
                elif sequence_type == 'rotate':
                    node.sequence_rotate_test()
                elif sequence_type == 'scan':
                    length = float(sys.argv[3]) if len(sys.argv) > 3 else 20
                    segments = int(sys.argv[4]) if len(sys.argv) > 4 else 10
                    z_offset = float(sys.argv[5]) if len(sys.argv) > 5 else 0
                    node.autonomous_scan_pattern(length, segments, z_offset)
                elif sequence_type == 'inductance':
                    if len(sys.argv) > 4:
                        evm_file = sys.argv[3]
                        ati_file = sys.argv[4]
                        node.sequence_inductance_test(evm_file, ati_file)
                    else:
                        node.get_logger().error('Inductance test requires two files: evm_file.csv ati_file.csv')
                else:
                    node.get_logger().error(f'Unknown sequence type: {sequence_type}')
                    node.get_logger().info('Available sequences: startup, calibration, square, rotate, scan, inductance')
            else:
                node.get_logger().error('Sequence type required. Use: startup, calibration, square, rotate, scan, inductance')
                
        elif mode == 'batch':
            # Batch mode - send commands from command line
            node = CNCCommandPublisher()
            commands = sys.argv[2:]  # All remaining arguments are commands
            if commands:
                node.batch_mode(commands)
            else:
                node.get_logger().error('No commands provided for batch mode')
                
        elif mode == 'single':
            # Single command mode
            node = CNCCommandPublisher()
            if len(sys.argv) > 2:
                command = ' '.join(sys.argv[2:])  # Join all remaining args as single command
                node.publish_command(command)
            else:
                node.get_logger().error('No command provided for single mode')
                
        else:
            print("Unknown mode. Available modes:")
            print("  interactive (default) - Interactive command prompt")
            print("  single <command>     - Send single command")
            print("  batch <cmd1> <cmd2>  - Send multiple commands")
            print("  sequence <type>      - Run predefined sequence")
            return
    else:
        # Interactive mode (default)
        node = CNCCommandPublisher()
        
        # Run interactive mode in a separate thread
        interactive_thread = threading.Thread(target=node.interactive_mode)
        interactive_thread.daemon = True
        interactive_thread.start()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    
    # Cleanup
    try:
        node.destroy_node()
        rclpy.shutdown()
    except:
        pass

if __name__ == '__main__':
    main()