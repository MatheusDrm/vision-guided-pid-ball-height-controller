from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import socket



north, east, south, west = "Up", "Right", "Down", "Left"

defaultGreenLower = (24, 36, 30)
defaultGreenUpper = (84, 255, 255)

defaultRedLower = (0, 145, 106)
defaultRedUpper = (38, 255, 255)

defaultBlueLower = (78, 75, 0)
defaultBlueUpper = (140, 255, 152)

windowTitle = "Tracking & Threshold"
hsvStr = "HSV"
border_inferior_y = 25

x, y = 0, 0
mask = []


def start_client():
    # Configurações do cliente
    host = '192.168.43.158'  # Endereço IP do servidor
    port = 5002         # Porta do servidor

    # Cria um soquete (socket) TCP/IP
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Conecta o soquete ao servidor
    client_socket.connect((host, port))

    return client_socket



def get_arguments():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to the (optional) video file")
    ap.add_argument("-bf", "--buffer", type=int, default=64, help="max buffer size")
    ap.add_argument("-t", "--threshold", help="Direction_detection. Lower = more sensitive. Default: 20",
                    default=20, type=int)

    group = ap.add_mutually_exclusive_group()
    group.add_argument("-r", "--green", help="track red objects", action='store_true')
    group.add_argument("-b", "--blue", help="track blue objects", action='store_true')
    return vars(ap.parse_args())


def callback(value):
    pass


def setup_trackbars(lowerBound, upperBound):
    cv2.namedWindow(windowTitle, 0)

    for i in ["MIN", "MAX"]:
        v = lowerBound if i == "MIN" else upperBound
        for j, k in zip(hsvStr, v):
            cv2.createTrackbar("%s_%s" % (j, i), windowTitle, k, 255, callback)


def get_trackbar_values():
    values = []

    for i in ["MIN", "MAX"]:
        for j in hsvStr:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), windowTitle)
            values.append(v)

    return values


def updatePosition(pts, counter, direction_detection, dX, dY):
    direction = ""
    
    if counter >= 10 and pts[-10] is not None and pts[0] is not None:
        dX = pts[-10][0] - pts[0][0]
        dY = pts[-10][1] - pts[0][1]
        (dirX, dirY) = ("", "")

        if np.abs(dX) > direction_detection:
            dirX = west if np.sign(dX) == 1 else east
        if np.abs(dY) > direction_detection:
            dirY = north if np.sign(dY) == 1 else south

        if dirX != "" and dirY != "":
            direction = "{}-{}".format(dirY, dirX)
        else:
            direction = dirX if dirX != "" else dirY

    return dX, dY, direction



def process_frame(frame, args):
    frame = imutils.resize(frame, width=600)
    frame = cv2.flip(frame, 1)
    overlay = frame.copy()

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsvFrame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    h_min, s_min, v_min, h_max, s_max, v_max = get_trackbar_values()

    org_mask = cv2.inRange(hsvFrame, (h_min, s_min, v_min), (h_max, s_max, v_max))
    mask = cv2.erode(org_mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 10:
            cv2.circle(overlay, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(overlay, center, 5, (0, 0, 255), -1)

    return overlay, center, mask




def display_info_local(overlay, dX, dY, direction, frame, mask, args, pts, client):
    thickness = int(np.sqrt(args["buffer"] / float(len(pts) + 1)) * 2.5)

    # Se houver pelo menos dois pontos e ambos não forem None
    if len(pts) > 1 and pts[-1] is not None and pts[-2] is not None:

        cv2.putText(overlay, "dx: {}, dy: {}".format(dX, dY), (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (0, 0, 255), 1)
        
        # Verifica se o último ponto não é None antes de acessar suas coordenadas
        if pts[-1] is not None:
            cv2.putText(overlay, "X: {}, Y: {}".format(int(pts[-1][0]), int(pts[-1][1])), (10, frame.shape[0] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 255), 1)

    start_point = (10, border_inferior_y)
    end_point = (frame.shape[1] - 10, frame.shape[0] - border_inferior_y)
    color = (255, 150, 150)
    thickness = 1
    cv2.rectangle(overlay, start_point, end_point, color, thickness)

    if pts[-1] is not None:
        altura = (frame.shape[0] - int(pts[-1][1]) - border_inferior_y) * (56 / (frame.shape[0] - 2 * border_inferior_y))-1
        print("Altura:", round(altura,2)) 
        # time.sleep(1)
        client.send(str(round(altura,2)).encode('utf-8'))


    display_mask1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    gui = np.hstack((overlay, display_mask1))

    cv2.imshow(windowTitle, gui)




def main():
    args = get_arguments()
    pts = deque(maxlen=args["buffer"])
    counter = 0
    (dX, dY) = (0, 0)

    if not args.get("video", False):
        vs = VideoStream(src=2).start()
    else:
        vs = cv2.VideoCapture(args["video"])

    time.sleep(2.0)

    if not args['green'] and not args['blue']:
        # setup_trackbars(defaultGreenLower, defaultGreenUpper)
        # cv2.createTrackbar("Y desejado", windowTitle, 0, 40, callback)
        setup_trackbars(defaultRedLower, defaultRedUpper)
        cv2.createTrackbar("Y desejado", windowTitle, 0, 53, callback)
    if args['green']:
        setup_trackbars(defaultGreenLower, defaultGreenUpper)
        cv2.createTrackbar("Y desejado", windowTitle, 0, 40, callback)
    if args['blue']:
        setup_trackbars(defaultBlueLower, defaultBlueUpper)
        cv2.createTrackbar("Y desejado", windowTitle, 0, 40, callback)
    if args['threshold']:
        direction_detection = args['threshold']



    client = start_client()


    while True:
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame

        if frame is None:
            break

        overlay, center, mask = process_frame(frame, args)  # Adicione a máscara como um retorno de process_frame
        pts.appendleft(center)
        dX, dY, direction = updatePosition(pts, counter, direction_detection, dX, dY)

        # Atualize a chamada para display_info_local
        display_info_local(overlay, dX, dY, direction, frame, mask, args, pts, client)

        key = cv2.waitKey(1) & 0xFF
        counter += 1

        if key == ord("q"):
            print(get_trackbar_values())
            break

    if not args.get("video", False):
        vs.stop()
    else:
        vs.release()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
