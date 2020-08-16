import sensor, image, time, ustruct, pyb
import json
from pyb import UART

ledBlue = pyb.LED(3) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.
ledGreen = pyb.LED(2)
ledRed = pyb.LED(1)
# 颜色跟踪阈值(L Min, L Max, A Min, A Max, B Min, B Max)
thresholds2 = [(0, 100, -66, -25, -8, 58)] # generic_green_thresholds

# 但是，在颜色阈值开始重叠之前，实际上不可能使用16个阈值对任何场景进行分段。
sensor.reset()
#初始化摄像头，reset()是sensor模块里面的函数

sensor.set_pixformat(sensor.RGB565)
#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种

sensor.set_framesize(sensor.QVGA)#320 240
#设置图像像素大小

sensor.skip_frames(3) # Let new settings take affect.
sensor.set_auto_gain(False) # 颜色跟踪必须关闭自动增益
sensor.set_auto_whitebal(False) # 颜色跟踪必须关闭白平衡
clock = time.clock()

#struct用来编码，接收到send指令后，发送数据
uart = UART(3, 115200)
# postion
offsetX = int(320/2)
offsetY = int(240/2)
position = [0,0,0]
while(True):
    ledBlue.on()
    clock.tick()
    img = sensor.snapshot()
    cmd = b'wait'
    if (uart.any()):
        cmd = uart.read(4)
    if(cmd == b'send'):
        uart.write(ustruct.pack("<lll", position[0],position[1],position[2]))
    for blob in img.find_blobs(thresholds2, pixels_threshold=200, area_threshold=200):
        img.draw_rectangle(blob.rect())
        x = blob.cx()- offsetX
        y = blob.cy()- offsetY
        position[0] = y
        position[1] = -x
        print(position)
