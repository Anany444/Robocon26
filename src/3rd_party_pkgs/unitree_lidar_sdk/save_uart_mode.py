import serial
import struct
import time
import argparse

def crc32(buf):
    crc = 0xFFFFFFFF
    for b in buf:
        crc ^= b
        for i in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return ~crc & 0xFFFFFFFF

def main():
    parser = argparse.ArgumentParser(description="Set Unitree LiDAR to UART mode and save it permanently.")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port of the LiDAR (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=4000000, help="Baud rate (default: 4000000)")
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud...")
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"Failed to open port {args.port}: {e}")
        return

    # 1. Packet to set LidarWorkMode to 8 (Serial)
    # LIDAR_WORK_MODE_CONFIG_PACKET_TYPE = 107
    header1 = struct.pack('<4BII', 0x55, 0xAA, 0x05, 0x0A, 107, 28)
    data_mode = struct.pack('<I', 8)  # mode = 8 (Serial)
    crc1 = crc32(header1 + data_mode)
    tail1 = struct.pack('<II2B2B', crc1, 0, 0, 0, 0x00, 0xFF)
    mode_packet = header1 + data_mode + tail1

    # 2. Packet to send CMD_PARAM_SAVE
    # LIDAR_COMMAND_PACKET_TYPE = 2000
    # CMD_PARAM_SAVE = 2
    header2 = struct.pack('<4BII', 0x55, 0xAA, 0x05, 0x0A, 2000, 32)
    data_cmd = struct.pack('<II', 2, 0)
    crc2 = crc32(header2 + data_cmd)
    tail2 = struct.pack('<II2B2B', crc2, 0, 0, 0, 0x00, 0xFF)
    save_packet = header2 + data_cmd + tail2

    print("Sending Work Mode = 8 (UART) command...")
    ser.write(mode_packet)
    time.sleep(0.5) 
    
    print("Sending Save Configuration command...")
    ser.write(save_packet)
    time.sleep(0.5)

    ser.close()
    print("Done! The LiDAR should now default to UART mode across power cycles.")

if __name__ == "__main__":
    main()
