import serial
import time
import random
import matplotlib.pyplot as plt
import numpy as np

def connect_to_arduino(port, baud_rate=115200):
    """Connect to the Arduino."""
    try:
        ser = serial.Serial(port, baud_rate, timeout=0.1)  # Short timeout for rapid commands
        print(f"Connected to {port} at {baud_rate} baud")
        time.sleep(2)  # Allow time for the Arduino to reset
        return ser
    except Exception as e:
        print(f"Error connecting to {port}: {e}")
        return None

def rapid_command_test(ser, num_commands=100, delay_between_commands=0.01):
    """Send many commands rapidly to test responsiveness."""
    print(f"\nStarting rapid command test with {num_commands} commands")
    print(f"Delay between commands: {delay_between_commands}s")
    
    # Initialize data collection
    command_times = []
    response_times = []
    success_count = 0
    
    # Run the test
    for i in range(num_commands):
        # Generate random angles
        pan = random.randint(0, 180)
        tilt = random.randint(0, 180)
        
        # Create command
        command = f"PT:{pan},{tilt}\n"
        
        # Record time and send command
        start_time = time.time()
        ser.write(command.encode())
        
        # Try to read response with timeout
        response = ser.readline().decode('utf-8', errors='replace').strip()
        end_time = time.time()
        
        # Record timing
        command_time = end_time - start_time
        command_times.append(command_time)
        
        if response:
            success_count += 1
            response_times.append(command_time)
            
        # Print progress
        if (i + 1) % 10 == 0:
            print(f"Completed {i + 1}/{num_commands} commands")
            
        # Wait specified delay before next command
        time.sleep(delay_between_commands)
    
    # Calculate statistics
    success_rate = (success_count / num_commands) * 100
    avg_response_time = np.mean(response_times) if response_times else 0
    
    print("\nTest Results:")
    print(f"Commands sent: {num_commands}")
    print(f"Successful responses: {success_count}")
    print(f"Success rate: {success_rate:.2f}%")
    print(f"Average response time: {avg_response_time*1000:.2f}ms")
    
    return command_times, success_rate, avg_response_time

def step_response_test(ser):
    """Test step response of servo by moving from one extreme to another."""
    print("\nStarting step response test")
    
    # Move to starting position and wait for stabilization
    ser.write(b"PT:0,90\n")
    time.sleep(1)
    
    # Prepare to measure
    ser.flushInput()
    
    # Send command to move to opposite position and time it
    print("Moving from 0° to 180° pan")
    start_time = time.time()
    ser.write(b"PT:180,90\n")
    
    # Wait for response
    response = ser.readline().decode('utf-8', errors='replace').strip()
    
    # Read for a while to catch any delayed responses
    additional_time = 0
    while additional_time < 2:  # Wait up to 2 seconds for more data
        if ser.in_waiting:
            more_data = ser.readline().decode('utf-8', errors='replace').strip()
            if more_data:
                response += " / " + more_data
        time.sleep(0.1)
        additional_time += 0.1
    
    print(f"Arduino response: {response}")
    print(f"Measured time for 180° movement: {(time.time() - start_time):.2f} seconds")
    
    # Return to center
    time.sleep(1)
    ser.write(b"PT:90,90\n")

def plot_results(command_times):
    """Plot the timing results."""
    plt.figure(figsize=(10, 6))
    plt.plot(command_times, 'b-')
    plt.xlabel('Command Number')
    plt.ylabel('Response Time (seconds)')
    plt.title('Arduino Servo Command Response Times')
    plt.grid(True)
    plt.savefig('servo_response_times.png')
    print("\nPlot saved as 'servo_response_times.png'")

def main():
    """Main test function."""
    port = input("Enter Arduino port (e.g., COM3, /dev/ttyUSB0): ")
    ser = connect_to_arduino(port)
    if not ser:
        return
    
    try:
        # Test menu
        print("\nPTZ Camera Control Test Menu:")
        print("1: Rapid command test (many commands quickly)")
        print("2: Step response test (measure full movement time)")
        print("3: Run both tests")
        print("4: Exit")
        
        choice = input("\nEnter test number (1-4): ")
        
        if choice == '1':
            num_commands = int(input("Enter number of commands to send (default 100): ") or "100")
            delay = float(input("Enter delay between commands in seconds (default 0.01): ") or "0.01")
            command_times, success_rate, avg_time = rapid_command_test(ser, num_commands, delay)
            plot_results(command_times)
            
        elif choice == '2':
            step_response_test(ser)
            
        elif choice == '3':
            # Run both tests
            num_commands = int(input("Enter number of commands for rapid test (default 100): ") or "100")
            delay = float(input("Enter delay between commands in seconds (default 0.01): ") or "0.01")
            command_times, success_rate, avg_time = rapid_command_test(ser, num_commands, delay)
            plot_results(command_times)
            time.sleep(1)  # Pause between tests
            step_response_test(ser)
            
        elif choice == '4':
            print("Exiting...")
            
        else:
            print("Invalid choice. Exiting...")
            
    except KeyboardInterrupt:
        print("\nTest terminated by user")
    except Exception as e:
        print(f"\nError during test: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()