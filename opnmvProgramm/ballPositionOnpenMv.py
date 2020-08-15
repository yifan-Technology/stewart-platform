import sensor, image, time, ustruct
import json
from pyb import UART
# 颜色跟踪阈值(L Min, L Max, A Min, A Max, B Min, B Max)
thresholds2 = [(0, 100, -66, -25, -8, 58)] # generic_green_thresholds

# 但是，在颜色阈值开始重叠之前，实际上不可能使用16个阈值对任何场景进行分段。
sensor.reset()
#初始化摄像头，reset()是sensor模块里面的函数

sensor.set_pixformat(sensor.RGB565)
#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种

sensor.set_framesize(sensor.QVGA)
#设置图像像素大小

sensor.skip_frames(3) # Let new settings take affect.
sensor.set_auto_gain(False) # 颜色跟踪必须关闭自动增益
sensor.set_auto_whitebal(False) # 颜色跟踪必须关闭白平衡
clock = time.clock()

#struct用来编码，接收到send指令后，发送数据
uart = UART(3, 115200)
# postion
position = [0,0,0]
while(True):
    clock.tick()
    img = sensor.snapshot()
    cmd = uart.read(4)
    if(cmd == b'send'):
        uart.write(ustruct.pack("<lll", position[0],position[1],position[2]))
    for blob in img.find_blobs(thresholds2, pixels_threshold=200, area_threshold=200):
        img.draw_rectangle(blob.rect())
        position[0] = blob.cx()
        position[1] = blob.cy()
        print(position)
