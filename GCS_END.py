import math
import cv2
import numpy as np
from pyzbar import pyzbar
import collections
import serial
import serial.tools.list_ports
import time
from picamera2 import Picamera2
from time import sleep
import re
import json
import os

# 使用树莓派picamera2库打开摄像头0，听说可以硬件加速，减少延迟，但我用的是usb摄像头，感觉和opencv直接读取差别不大
# 摄像头0，位于机械臂上，主要用来颜色识别与圆的识别
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)})
picam2.configure(config)
picam2.start()

# 打开摄像头1
# 摄像头1，位于小车上，主要用来识别二维码
cap1 = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)  # 摄像头2，car上，用来识别ewm

# 设置MJPG格式
cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap1.set(cv2.CAP_PROP_AUTO_WB, 1)

# 设置分辨率和帧率
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 或 640
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 或 480
cap1.set(cv2.CAP_PROP_FPS, 60)

# 检查实际帧率
fps = cap1.get(cv2.CAP_PROP_FPS)
print(f"实际帧率: {fps} FPS")

# 定义感兴趣区域 (ROI) 的坐标和大小
# 原来是为了裁切画面，减少干扰，后因为机械组可以调节高度增大目标，故本代码没用上
roi_line_x, roi_line_y, roi_line_width, roi_line_height = 160, 120, 320, 240  # 直线roi
roi_block_x, roi_block_y, roi_block_width, roi_block_height = 0, 0, 640, 480  # 物块roi
roi_target_x, roi_target_y, roi_target_width, roi_target_height = 0, 0, 640, 480  # a圆环roi

threshold_line_1 = 50
threshold_line_2 = 150


def empty(a):
    pass

# 串口初始化
def init_uart1():
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("NO UART")
    else:
        print("HAVE UART")
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])

    ser = serial.Serial("/dev/ttyAMA10", 115200)
    if ser.isOpen():
        print("UART YES")
        print(ser.name)
        return ser

    else:
        print("UART NO")


# 处理数据
def uart_send(x, y, z):
    x_str = f"{x:.1f}"  # 发送信息
    y_str = f"{y:.1f}"
    color_str = str(z)
    data_str = "sm" + x_str + "," + y_str + "," + color_str + "ms"
    return data_str

# 读取物块颜色阈值文件
def load_object_thresholds(file_path='/home/pi/color_thresholds.json'):
    """加载物体检测阈值"""
    default_thresholds = {
        'red1': {'lower': [0, 106, 185], 'upper': [35, 255, 255]},
        'red2': {'lower': [153, 32, 148], 'upper': [179, 212, 220]},
        'blue': {'lower': [91, 81, 198], 'upper': [128, 230, 255]},
        'green': {'lower': [46, 73, 176], 'upper': [81, 255, 237]}
    }

    if not os.path.exists(file_path):
        with open(file_path, 'w') as f:
            json.dump(default_thresholds, f, indent=4)
        return default_thresholds

    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except (IOError, json.JSONDecodeError):
        return default_thresholds

# 读取地面颜色阈值
def load_color_thresholds(file_path='/home/pi/yuzhi.json'):
    """加载颜色阈值"""
    default_thresholds = {
        'L_R_H': 50,
        'U_R_H': 100,
        'L_R_S': 155,
        'U_R_S': 255,
        'L_R_V': 155,
        'U_R_V': 255
    }

    if not os.path.exists(file_path):
        with open(file_path, 'w') as f:
            json.dump(default_thresholds, f, indent=4)
        return default_thresholds

    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except (IOError, json.JSONDecodeError):
        return default_thresholds

# 二维码识别函数
def qr_detector():
    ret0, frame0 = cap1.read()
    if not ret0:
        print("Failed to grab frame")
        return frame0, 0  # 返回空数据

    barcodes = pyzbar.decode(frame0)

    if barcodes:
        for barcode in barcodes:
            try:
                barcode_data = barcode.data.decode("utf-8")  # 严格 UTF-8 解码
                # 可选：检查数据格式（如必须是 "数字+数字"）
                if not re.match(r'^\d+\+\d+$', barcode_data):
                    print(f"非法格式: {barcode_data}")
                    continue  # 跳过

                print(f"识别到的二维码数据: {barcode_data}")

                # 绘制二维码框和文本
                (x, y, w, h) = barcode.rect
                cv2.rectangle(frame0, (x, y), (x + w, y + h), (0, 255, 0), 2)
                text = f"{barcode_data} ({barcode.type})"
                cv2.putText(frame0, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                return frame0, barcode_data

            except UnicodeDecodeError:
                print("非法二维码（非 UTF-8 编码），已跳过")
                continue  # 跳过当前二维码，继续检测下一个

    else:
        print("未检测到二维码")

    return frame0, 0  # 返回空数据

# 一开始没学到文件管理阈值，故遗留
# 物块颜色阈值
lower_red1_objects = np.array([0, 106, 185])
upper_red1_objects = np.array([35, 255, 255])
lower_red2_objects = np.array([153, 32, 148])
upper_red2_objects = np.array([179, 212, 220])
lower_blue_objects = np.array([91, 81, 198])
upper_blue_objects = np.array([128, 230, 255])
lower_green_objects = np.array([46, 73, 176])
upper_green_objects = np.array([81, 255, 237])
# 圆环颜色阈值
lower_red1_target = np.array([0, 33, 133])
upper_red1_target = np.array([25, 255, 255])
lower_red2_target = np.array([143, 33, 170])
upper_red2_target = np.array([179, 255, 155])
lower_blue_target = np.array([94, 46, 25])
upper_blue_target = np.array([131, 255, 213])
lower_green_target = np.array([24, 43, 115])
upper_green_target = np.array([69, 146, 200])


# 存储多帧的圆心和半径
circle_buffer = collections.deque(maxlen=2)

# 物块颜色识别函数
def identify_objects(frame, lower_red1, upper_red1, lower_red2, upper_red2, lower_blue, upper_blue, lower_green,
                     upper_green):  # 物块阈值

    data_str = None
    color = ""

    # 处理图像
    # 将帧转换为HSV颜色空间
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 根据颜色范围创建掩膜
    red_mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    # 识别颜色
    # 对掩膜进行形态学操作，以去除噪声
    kernel = np.ones((7, 7), np.uint8)

    red_mask_color = cv2.morphologyEx(red_mask, cv2.MORPH_DILATE, kernel)
    blue_mask_color = cv2.morphologyEx(blue_mask, cv2.MORPH_DILATE, kernel)
    green_mask_color = cv2.morphologyEx(green_mask, cv2.MORPH_DILATE, kernel)

    red_mask_color = cv2.dilate(red_mask_color, kernel, 1)
    blue_mask_color = cv2.dilate(blue_mask_color, kernel, 1)
    green_mask_color = cv2.dilate(green_mask_color, kernel, 1)

    cv2.imshow('red_mask_color', red_mask_color)
    # cv2.imshow('blue_mask_color', blue_mask_color)
    # cv2.imshow('green_mask_color', green_mask_color)

    # 在原始帧中找到颜色区域并绘制方框
    red_contours, _ = cv2.findContours(red_mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv2.findContours(blue_mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in red_contours:
        x, y, w, h = cv2.boundingRect(contour)
        if cv2.contourArea(contour) > 20000:  # 设置最小区域面积以排除噪声
            color = "1"         # 标记1为红色
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            # 处理信息
            x = (2 * x + w) / 2
            y = (2 * y + h) / 2
            return frame, x, y, color
    for contour in blue_contours:
        x, y, w, h = cv2.boundingRect(contour)
        if cv2.contourArea(contour) > 20000:  # 设置最小区域面积以排除噪声
            color = "3"         # 标记3为蓝色
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            # 处理信息
            x = (2 * x + w) / 2
            y = (2 * y + h) / 2
            return frame, x, y, color
    for contour in green_contours:
        x, y, w, h = cv2.boundingRect(contour)
        if cv2.contourArea(contour) > 20000:  # 设置最小区域面积以排除噪声
            color = "2"         # 标记2为绿色
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            # 处理信息
            x = (2 * x + w) / 2
            y = (2 * y + h) / 2
            return frame, x, y, color
    return frame, 0, 0, 0

# 圆环识别函数
def identify_target(frame, lower_color, upper_color):
    # 将图像从 BGR 转换为 HSV 颜色空间
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_transit = frame
    # 创建掩码，提取目标颜色区域
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)
    # 这一步的操作是为了消除画面影响，中间绿色圆环与地面颜色相近，边缘提取不出来，所以想了这个方法，把底面颜色提取滤掉
    # 将目标颜色区域变为白色 (255, 255, 255)
    frame[mask > 0] = [255, 255, 255]

    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    gray = cv2.GaussianBlur(gray, (9, 9), 2)
    # Canny 边缘检测
    edges = cv2.Canny(gray, 50, 150)
    cv2.imshow("edges", edges)
    # 查找轮廓
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # identify_objects
    frame_transit, x_transit, y_transit, color_transit = identify_objects(frame_transit, lower_red1_target,
                                                                          upper_red1_target, lower_red2_target,
                                                                          upper_red2_target, lower_blue_target,
                                                                          upper_blue_target, lower_green_target,
                                                                          upper_green_target)
    min_area = 10000
    for contour in contours:
        if cv2.contourArea(contour) > min_area and len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)
            (center, axes, angle) = ellipse
            major_axis = max(axes)
            minor_axis = min(axes)
            aspect_ratio = minor_axis / major_axis
            if aspect_ratio > 0.8:  # 只保留长宽比大于 0.8 的椭圆
                cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
                center666 = (int(center[0]), int(center[1]))
                cv2.circle(frame, center666, 5, (0, 0, 255), -1)
                print(f"Ellipse center: ({center[0]}, {center[1]})")
                return frame, center[0], center[1], color_transit
    return frame, 0, 0, 0


sharpening_kernel = np.array([
    [0, -1, 0],
    [-1, 5, -1],
    [0, -1, 0]
], dtype=np.float32)

# 直线角度识别，使用霍夫变换检测直线，（因陀螺仪漂移，途中需用以矫正。之后换了好一点的陀螺仪，这段代码废弃
def identify_line(frame):
    frame = cv2.filter2D(frame, -1, sharpening_kernel)
    # 提取 ROI 区域
    roi = frame[roi_line_y:roi_line_y + roi_line_height, roi_line_x:roi_line_x + roi_line_width]
    # 转换为灰度图
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    # 使用Canny边缘检测
    edges = cv2.Canny(gray, threshold_line_1, threshold_line_2, apertureSize=3)
    cv2.imshow("edges", edges)
    # 使用霍夫变换检测直线
    lines = cv2.HoughLines(edges, 1, np.pi / 720, 150)

    # 遍历检测到的直线
    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b)) + roi_line_x
            y1 = int(y0 + 1000 * (a)) + roi_line_y
            x2 = int(x0 - 1000 * (-b)) + roi_line_x
            y2 = int(y0 - 1000 * (a)) + roi_line_y

            # 在图像上绘制直线
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

            # 计算角度（弧度转角度）
            angle = math.degrees(theta)
            print(666)
            print(f"直线角度: {angle} 度")
            return frame, angle, 4, 4
    return frame, 0, 0, 0


def main():
    receive = '0'
    # 初始化串口
    ser = init_uart1()
    # 加载物体检测阈值
    object_thresholds = load_object_thresholds()

    # 转换为numpy数组
    lower_red1_objects = np.array(object_thresholds['red1']['lower'])
    upper_red1_objects = np.array(object_thresholds['red1']['upper'])
    lower_red2_objects = np.array(object_thresholds['red2']['lower'])
    upper_red2_objects = np.array(object_thresholds['red2']['upper'])
    lower_blue_objects = np.array(object_thresholds['blue']['lower'])
    upper_blue_objects = np.array(object_thresholds['blue']['upper'])
    lower_green_objects = np.array(object_thresholds['green']['lower'])
    upper_green_objects = np.array(object_thresholds['green']['upper'])
    # 加载di_mian颜色阈值
    color_thresholds = load_color_thresholds()
    L_R_H = color_thresholds['L_R_H']
    U_R_H = color_thresholds['U_R_H']
    L_R_S = color_thresholds['L_R_S']
    U_R_S = color_thresholds['U_R_S']
    L_R_V = color_thresholds['L_R_V']
    U_R_V = color_thresholds['U_R_V']
    lower_color = np.array([L_R_H, L_R_S, L_R_V])
    upper_color = np.array([U_R_H, U_R_S, U_R_V])

    while True:
        # 逐帧读取摄像头
        frame00 = picam2.capture_array()
        frame0 = cv2.cvtColor(frame00, cv2.COLOR_RGB2BGR)

        # 接收串口数据
        if ser.in_waiting > 0:
            receive = ser.read(1).decode('utf-8', errors='ignore').strip()
            print(receive)
        if receive == '0':          # 收到0信号，进行二维码识别
            result, qr = qr_detector()
            qr_str = str(qr)
            data_str = "sm" + qr_str + "ms"
            write_len = ser.write(data_str.encode('utf-8'))
            print(f'data:{data_str}, bit:{write_len}, order:0')
            cv2.imshow("identify_line", result)
        elif receive == '1':        # 收到1信号，进行物块颜色识别
            result, x, y, color = identify_objects(frame0, lower_red1_objects, upper_red1_objects, lower_red2_objects,
                                                   upper_red2_objects, lower_blue_objects, upper_blue_objects,
                                                   lower_green_objects, upper_green_objects)
            data_str = uart_send(x, y, color)
            write_len = ser.write(data_str.encode('utf-8'))
            print(f'data:{data_str}, bit:{write_len},order:1')
            cv2.imshow("identify_objects", result)
        elif receive == '2':        # 收到2信号，进行圆环识别
            result, x, y, color = identify_target(frame0, lower_color, upper_color)
            data_str = uart_send(x, y, color)
            write_len = ser.write(data_str.encode('utf-8'))
            print(f'data:{data_str}, bit:{write_len},order:2')
            cv2.imshow("identify_target", result)
        elif receive == '4':        # 收到4信号，进行直线角度识别
            result, x, y, angle = identify_line(frame0)
            data_str = uart_send(x, y, angle)
            write_len = ser.write(data_str.encode('utf-8'))
            print(f'data:{data_str}, bit:{write_len},order:4')
            cv2.imshow("identify_line", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    main()
    picam2.stop()
    cv2.destroyAllWindows()