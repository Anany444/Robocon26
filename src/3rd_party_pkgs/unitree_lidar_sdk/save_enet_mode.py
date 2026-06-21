import socket
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
    parser = argparse.ArgumentParser(description="Set Unitree LiDAR to ENET mode and save it permanently via UDP.")
    parser.add_argument("--ip", type=str, default="192.168.1.62", help="LiDAR IP address (default: 192.168.1.62)")
    parser.add_argument("--port", type=int, default=6101, help="LiDAR UDP Port (default: 6101)")
    args = parser.parse_args()

    print(f"Connecting to LiDAR at {args.ip}:{args.port} via UDP...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except Exception as e:
        print(f"Failed to create UDP socket: {e}")
        return

    # 1. Packet to set LidarWorkMode to 0 (ENET)
    # LIDAR_WORK_MODE_CONFIG_PACKET_TYPE = 107
    header1 = struct.pack('<4BII', 0x55, 0xAA, 0x05, 0x0A, 107, 28)
    data_mode = struct.pack('<I', 0)  # mode = 0 (ENET)
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

    print("Sending Work Mode = 0 (ENET) command...")
    sock.sendto(mode_packet, (args.ip, args.port))
    time.sleep(0.5) 
    
    print("Sending Save Configuration command...")
    sock.sendto(save_packet, (args.ip, args.port))
    time.sleep(0.5)

    sock.close()
    print("Done! The LiDAR should now be saved to ENET mode.")

if __name__ == "__main__":
    main()
