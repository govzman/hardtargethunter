from ast import excepthandler
import functools
from functools import partial
import PyQt5
from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5 import QtCore
import cv2 as cv
import qimage2ndarray
import serial
import imutils
import skimage
from skimage.metrics import structural_similarity as compare_ssim
import struct



def findObject(img):
    global mode, x, y, w, h, dx, dy, x_camera, y_camera, background, ser, mode
    img_copy = img.copy()
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    (_, diff) = compare_ssim(background, img, full=True)
    diff = (diff * 255).astype("uint8")

    thresh = cv.threshold(diff, 0, 255,
        cv.THRESH_BINARY_INV | cv.THRESH_OTSU)[1]
    cnts = cv.findContours(thresh.copy(), cv.RETR_TREE,
        cv.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    try:
        c = sorted(cnts, key=lambda x: cv.contourArea(x))[-1]
    except:
        pass
    cv.drawContours(img_copy, c, -1, (0, 255, 0), 3, cv.LINE_AA)
    (x, y, w, h) = cv.boundingRect(c)
        
    if mode not in ['work', 'stop']:
        cv.rectangle(img_copy, (640 - x_camera[0], 480 - y_camera[0]), (640 - x_camera[1], 480 - y_camera[1]), (255, 0, 0), 1)
        cv.rectangle(img_copy, (x, y), (x + w, y + h), (0, 0, 255), 5)
        cv.circle(img_copy, (x + w//2, y + h//2), 3, (255, 0, 0), 1)

    if ser != '':
        try:
            if mode == 'work':
                ser.write(struct.pack('>B', int((640 - (x + w // 2) - x_camera[0]) / (x_camera[1] - x_camera[0]) * (x_laser[1] - x_laser[0]) + x_laser[0]))) # 2 серва
                ser.write(struct.pack('>B', int((480 - (y + h // 2) - y_camera[0]) / (y_camera[1] - y_camera[0]) * (y_laser[1] - y_laser[0]) + y_laser[0]))) # 2 серва
            elif mode == 'callibrate_x':
                ser.write(struct.pack('>B', 90 + dx)) # 1 серва
                ser.write(struct.pack('>B', 75)) # 2 серва    
            elif mode == 'callibrate_y':
                ser.write(struct.pack('>B', 90)) # 1 серва                                                                 
                ser.write(struct.pack('>B', 90 + dy)) # 2 серва
            else:
                ser.write(struct.pack('>B', 90)) # 1 серва
                ser.write(struct.pack('>B', 90)) # 2 серва
        except:
            ser = ''
    return img_copy, thresh

def setupCamera(ui, frame_timer, fps=30):
    global background
    camera_capture = cv.VideoCapture(0)
    camera_capture.set(3, 640)
    camera_capture.set(4, 480)
    background = cv.cvtColor(camera_capture.read()[1], cv.COLOR_BGR2GRAY)
    frame_timer.timeout.connect(partial(display_video_stream, ui, camera_capture))
    frame_timer.start(int(1000//fps))
    return camera_capture

def setup_serial(ui):       
        serial = QSerialPort()
        serial.setBaudRate(9600)
        portList = []
        ports = QSerialPortInfo().availablePorts()
        for port in ports:
            portList.append(port.portName())
        ui.ports.clear()
        ui.ports.addItems(portList)

def display_video_stream(ui, camera_capture, fps=30):
    global background
    ret, frame = camera_capture.read()
    if not ret:
        return False

    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
    frame, mask = findObject(frame)
    mask = cv.resize(mask, (240, 180), interpolation = cv.INTER_AREA)
    frame = cv.flip(frame, 1)
    mask = cv.flip(mask, 1)
    frame = cv.putText(frame, 'x: ' + str(640 - (x + w//2)) + '; y: ' + str(480 - (y + h//2)), (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv.LINE_AA)
    image = qimage2ndarray.array2qimage(frame)
    mask = qimage2ndarray.array2qimage(mask)

    ui.frame_label.setPixmap(QtGui.QPixmap.fromImage(image))
    ui.mask_label.setPixmap(QtGui.QPixmap.fromImage(mask))

def update_background():
    global background, camera_capture
    background = cv.cvtColor(camera_capture.read()[1], cv.COLOR_BGR2GRAY)

def change_mode(new_mode):
    global ui, mode
    mode = new_mode
    en = True
    if new_mode in ['stop', 'work']:
        en = False
    ui.calib_label.setEnabled(en)
    ui.laser_label.setEnabled(en)
    ui.minus_button.setEnabled(en)
    ui.plus_button.setEnabled(en)
    ui.calib_label.setEnabled(en)
    ui.set_max_button.setEnabled(en)
    ui.set_min_button.setEnabled(en)
    print(new_mode)

def serial_connect():
    global ui, ser
    try:
        ser = serial.Serial(
        port=ui.ports.currentText(),\
        baudrate=9600)
        ser.reset_input_buffer()
    except:
        ser = ''

def d_calib(d):
    global mode, dx, dy
    if mode == 'callibrate_x':
        dx += d
        print(90 + dx)
    elif mode == 'callibrate_y':
        dy += d
        print(90 + dy)

def set_extremum(type_of_extr):
    global mode, x, y, w, h, dx, dy, x_camera, x_laser, y_laser, y_camera
    if mode == 'callibrate_x':
        if type_of_extr == 'max':
            x_camera[1] = 640 - (x + w // 2)
            x_laser[1] = 90 + dx
            print('new x max', x_camera[1], x_laser[1])
        elif type_of_extr == 'min':
            x_camera[0] = 640 - (x + w // 2)
            x_laser[0] = 90 + dx
            print('new x min', x_camera[0], x_laser[0])
    elif mode == 'callibrate_y':
        if type_of_extr == 'max':
            y_camera[1] = 480 - (y + h // 2)
            y_laser[1] = 90 + dy
            print('new y max', y_camera[1], y_laser[1])
        elif type_of_extr == 'min':
            y_camera[0] = 480 - (y + h // 2)
            y_laser[0] = 90 + dy
            print('new y min', y_camera[0], y_laser[0])

dy, dx = 0, 0 
x, y, w, h = 0, 0, 0, 0
x_camera = [63, 598] # minimum and maximum on camera
x_laser = [66, 109]
y_camera = [249, 446] # minimum and maximum on camera
y_laser = [67, 81]
mode = 'work'
ser = ''

app = QtWidgets.QApplication([])
ui = uic.loadUi('hard_target_hunter.ui')
ui.setWindowTitle('Наводчик')
ui.update_ports_pushButton.clicked.connect(partial(setup_serial, ui))
ui.ports.currentTextChanged.connect(serial_connect)

ui.radioButton_1.clicked.connect(partial(change_mode, 'work'))
ui.radioButton_3.clicked.connect(partial(change_mode, 'stop'))
ui.radioButton_4.clicked.connect(partial(change_mode, 'callibrate_x'))
ui.radioButton_5.clicked.connect(partial(change_mode, 'callibrate_y'))

ui.plus_button.clicked.connect(partial(d_calib, 1))
ui.minus_button.clicked.connect(partial(d_calib, -1))

ui.set_max_button.clicked.connect(partial(set_extremum, 'max'))
ui.set_min_button.clicked.connect(partial(set_extremum, 'min'))

frame_timer = QtCore.QTimer()
camera_capture = setupCamera(ui, frame_timer)
ui.reset_background_button.clicked.connect(update_background)
display_video_stream(ui, camera_capture)


ui.show()
app.exec()