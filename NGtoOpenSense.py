from ast import If
import osc_decoder
import socket

def readIMU(l, q):
    # Send /identify message to strobe all LEDs.  The OSC message is constructed
    # from raw bytes as per the OSC specification.  The IP address must be equal to
    # the IP address of the target NGIMU.
    send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # send_socket.sendto(bytes("/identify\0\0\0,\0\0\0", "utf-8"), ("192.168.50.205", 9000)) #For 8205
    # send_socket.sendto(bytes("/identify\0\0\0,\0\0\0", "utf-8"), ("192.168.50.27", 9000))
    # Array of UDP ports to listen to, one per NGIMU.  These ports must be equal to
    # the UDP Send Port in the NGIMU settings.  The UDP Send IP Address setting
    # must be the computer's IP address.  Both these settings are changed
    # automatically when connecting to the NGIMU using the NGIMU GUI.
    receive_ports = [8101, 8102] #This number is very important for recognizing sensors
    receive_sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(len(receive_ports))]

    IMU_list = [None]*len(receive_ports)

    index = 0
    for receive_socket in receive_sockets:
        receive_socket.bind(("", receive_ports[index]))
        index = index + 1
        receive_socket.setblocking(False)

    while True:
        for udp_socket in receive_sockets:
            idx = receive_sockets.index(udp_socket)
            try:
                data, addr = udp_socket.recvfrom(2048)
            except socket.error:
                pass
            else:
                message = osc_decoder.decode(data)[0]
                message.append(udp_socket.getsockname()[1])
                IMU_list[idx] = message
                if(idx == (len(receive_ports)-1))&(None not in IMU_list):
                    l.acquire()
                    q.put(IMU_list)
                    l.release()