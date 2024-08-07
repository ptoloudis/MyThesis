import struct
import socket
import time

# Define the structure format
# 'H' stands for uint16 and 'I' stands for uint32
# The format string 'HHI16H' corresponds to:
# 2 uint16 (magic, frame_rate)
# 1 uint32 (frame_count)
# 16 uint16 (pwm array)
struct_format = 'HHI16H'

# Example data for the structure
magic = 18458
frame_rate = 500
frame_count = 0
pwm_value = 1100  # Initial PWM value

# Define the target address and port
UDP_IP = "127.0.0.1"  # localhost
UDP_PORT = 9002       # Port to send to

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

interval = 1.0 / frame_rate

try:
    while pwm_value <= 2000:
        for _ in range(5):  # Send the same PWM for 5 times
            # Record the start time
            start_time = time.time()

            # Create the PWM array with the current PWM value
            pwm = [pwm_value] * 16

            # Create the binary data for the structure
            packed_data = struct.pack(struct_format, magic, frame_rate, frame_count, *pwm)

            # Send the message
            sock.sendto(packed_data, (UDP_IP, UDP_PORT))

            # Calculate the time taken to send the message
            elapsed_time = time.time() - start_time

            # Sleep for the remainder of the interval, if necessary
            time_to_sleep = interval - elapsed_time
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

            frame_count += 1
        
        # Increment the PWM value
        pwm_value += 1

    while 1:
        
        # Record the start time
        start_time = time.time()

        # Create the PWM array with the current PWM value
        pwm = [1500] * 16

        # Create the binary data for the structure
        packed_data = struct.pack(struct_format, magic, frame_rate, frame_count, *pwm)

        # Send the message
        sock.sendto(packed_data, (UDP_IP, UDP_PORT))

        # Calculate the time taken to send the message
        elapsed_time = time.time() - start_time

        # Sleep for the remainder of the interval, if necessary
        time_to_sleep = interval - elapsed_time
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)

        frame_count += 1

except KeyboardInterrupt:
    print("UDP Client stopped.")
finally:
    sock.close()
