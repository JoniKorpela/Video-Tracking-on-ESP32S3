import socket
import cv2
import numpy as np
import struct
import targetSelector
import time

# Set the HOST variable to the IP address your router gives to your ESP32
HOST = 'SET ESP32 IP'
PORT = 8001

TARGET_WIDTH = 1920
TARGET_HEIGHT = 1080


def recv_dims(sock, size):
        if (sock.recv(1, socket.MSG_PEEK)[0] == 4):
            sock.recv(1)
            buf = bytearray()
            while len(buf) < size:
                data = sock.recv(size - len(buf))
                buf.extend(data)
            return buf
        else:
            return None

def recv_box(sock, size):
        if (sock.recv(1, socket.MSG_PEEK)[0] == 2):
            sock.recv(1)
            buf = bytearray()
            while len(buf) < size:
                data = sock.recv(size - len(buf))
                buf.extend(data)
            return buf
        else:
            return None

def recv_frame(sock, size):
    if (sock.recv(1, socket.MSG_PEEK)[0] == 3):
        sock.recv(1)
        buf = bytearray()
        while len(buf) < size:
            data = sock.recv(size - len(buf))
            buf.extend(data)
        return buf
    else:
        return None
    
def recv_cmd_disconnect(sock, size):
    if (sock.recv(1, socket.MSG_PEEK)[0] == 1):
        sock.recv(1)
        buf = bytearray()
        while len(buf) < size:
            data = sock.recv(size - len(buf))
            buf.extend(data)
        return buf
    else:
        return None


def send(sock, data):
    total_sent = 0
    while total_sent < len(data):
        sent = sock.send(data[total_sent:])
        if sent == 0:
            raise RuntimeError("Problem in send")
        total_sent += sent


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(10)
        print(f"Connecting to {HOST}:{PORT}...")
        sock.connect((HOST, PORT))
        print("Connected. Receiving frame dimensions..")

        try:
            dimensions = recv_dims(sock, 4)
            width, height = struct.unpack('!HH', dimensions)
            print(f"Received dimensions: width={width}, height={height}")

        except Exception as e:
            print(f"Error: {e}")

        box_size = 8
        size = width * height
        scale_w = TARGET_WIDTH / width
        scale_h = TARGET_HEIGHT / height
        scale = min(scale_w, scale_h)
        new_width = int(width * scale)
        new_height = int(height * scale)

        print("Frame dimensions received. Receiving frames..")

        window_name = "Tracker"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        target_selector = targetSelector.TargetSelector(window_name, scale)
        box_data = False
        x_min = None
        x_max = None
        y_min = None
        y_max = None 

        frame_count = 0
        start_time = time.time()
        fps_text = None

        try:
            while True:
                if target_selector.get_state() == "READY":
                    print("Sending the target box")
                    target_box = target_selector.get_target_box()
                    print(target_box)
                    box = [2, target_box.x_min, target_box.y_min,
                           target_box.x_max, target_box.y_max]
                    box_bytes = struct.pack('!BHHHH', *box)
                    send(sock, box_bytes)
                    target_selector.set_state("TRACK")

                ret = recv_cmd_disconnect(sock, 1)
                if ret is not None and ret[0] == 2:
                    print("Received disconnect ACK")
                    time.sleep(0.2)
                    break
                    
                box_data = recv_box(sock, box_size)
                # print(box_data)
                if box_data:
                    try:
                        x_min, y_min, x_max, y_max = struct.unpack(
                            '!HHHH', box_data)
                    except Exception as e:
                        print(f"Error: {e}")
                    target_selector.target_box.x_min = x_min
                    target_selector.target_box.x_max = x_max
                    target_selector.target_box.y_min = y_min
                    target_selector.target_box.y_max = y_max

                frame_data = recv_frame(sock, size)

                if frame_data:
                    try:
                        img = np.frombuffer(
                            frame_data, dtype=np.uint8).reshape((height, width))
                    except ValueError as ve:
                        print(f"Image reshape failed: {ve}")
                        continue

                    frame_count += 1
                    current_time = time.time()
                    elapsed = current_time - start_time

                    if elapsed >= 10.0:
                        fps = frame_count / elapsed
                        fps_text = f"{fps:.2f}"
                        frame_count = 0
                        start_time = current_time

                    target_selector.draw_box(img)
                    
                    if fps_text:
                        cv2.putText(img, fps_text, (5, 20), cv2.FONT_HERSHEY_DUPLEX,
                            0.5, (255, 255, 255), 1, cv2.LINE_8)

                    resized_img = cv2.resize(
                        img, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
                    cv2.imshow(window_name, resized_img)
                    cv2.imshow("Raw", img)

                

                key = cv2.waitKey(1) & 0xFF

                if (key == ord('q')):
                    header = [1]
                    header_bytes = struct.pack('!B', *header)
                    send(sock, header_bytes)
                    cmd = [2]
                    cmd_bytes = struct.pack('!B', *cmd)
                    send(sock, cmd_bytes)
                    print("Sent stop cmd")
                elif (key == ord('t')):
                    if (target_selector.get_state() == "TRACK"):
                        header = [1]
                        header_bytes = struct.pack('!B', *header)
                        send(sock, header_bytes)
                        cmd = [1]
                        cmd_bytes = struct.pack('!B', *cmd)
                        send(sock, cmd_bytes)
                        print("Sent stop track cmd")
                        target_selector.set_state("INIT")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            cv2.destroyAllWindows()
            sock.shutdown(socket.SHUT_WR)
            sock.close()
            print("Socket closed")


if __name__ == "__main__":
    main()
