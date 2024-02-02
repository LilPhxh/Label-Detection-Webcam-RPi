from PyQt5 import uic, QtTest                                                   
from PyQt5.QtWidgets import QApplication, QMainWindow, QDialog ,QWidget         
from PyQt5.QtCore import pyqtSignal, QThread, QTimer,pyqtSlot, QObject          
from PyQt5.QtGui import QImage, QPixmap, QIcon                                  
from PyQt5.QtMultimedia import QCameraInfo                                      
import cv2,time,utility.systeminfo as systeminfo                                
import numpy as np                                                              
import pyzbar.pyzbar as pyzbar                                                  
from pyzbar.pyzbar import decode, ZBarSymbol                                    
import RPi.GPIO as IO                                                           
import sys,random                                                               

class CameraThread(QThread): 

    ImageUpdate = pyqtSignal(np.ndarray)
    CameraStatus = pyqtSignal(bool)

# กำหนด constructor ของคลาส โดยรับ parameter cameraIndex
def __init__(self, cameraIndex):
    # เรียก constructor ของคลาสแม่ (superclass)
    super().__init__()

    # กำหนดค่าตัวแปร camIndex เท่ากับค่าที่รับมาจาก parameter
    self.camIndex = cameraIndex
    # กำหนดตัวแปร cap ให้เป็น None (ยังไม่ได้เปิดกล้อง)
    self.cap = None
    # กำหนดตัวแปร ThreadActive เป็น False เพื่อบอกว่า thread ยังไม่ทำงาน
    self.ThreadActive = False

# เมธอดที่ใช้กำหนดสีของข้อความ (text) ตามค่า fps
def Textcolor(self, fps):
    # กำหนดสีเริ่มต้นเป็นสีดำ (0, 0, 0)
    textcolor_fps = (0, 0, 0)

    # ตรวจสอบค่า fps และกำหนดสีของ textcolor_fps ตามเงื่อนไขที่ระบุ
    if fps < 5: 
        textcolor_fps = (255, 0, 0)  # สีแดง
    if fps > 15: 
        textcolor_fps = (250, 80, 10)  # สีส้ม
    if fps < 25: 
        textcolor_fps = (160, 150, 0)  # สีเหลือง
    if fps > 30: 
        textcolor_fps = (0, 250, 0)  # สีเขียว

    # ส่งค่าสีที่กำหนดไว้กลับ
    return textcolor_fps

# เมธอด run ที่ถูกเรียกเมื่อ thread ทำงาน
def run(self):
    # เปิดใช้งานกล้องด้วย index ที่ระบุใน self.camIndex
    self.Cap = cv2.VideoCapture(self.camIndex)
    # กำหนดความสูงของเฟรมที่ถ่ายรูป
    self.Cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # กำหนดความกว้างของเฟรมที่ถ่ายรูป
    self.Cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # กำหนดให้ thread ทำงานอยู่
    self.ThreadActive = True
    # กำหนดตัวแปร start และ end ในการวัดเวลา
    start = 0
    end = 0

    # ทำงานในลูปแบบไม่จบ
    while self.ThreadActive:
        # อ่านเฟรมจากกล้อง
        ret, frame = self.Cap.read()
        # วัดเวลาที่ใช้ในการอัปเดตเฟรม
        end = time.time()
        total = start - end
        # คำนวณ frames per second (fps)
        fps = 1 / -(total)
        # กำหนดสีของข้อความตาม fps ที่คำนวณได้
        color = self.Textcolor(fps)
        # แปลง fps เป็น integer
        fps = int(fps)
        # นำ fps มาแสดงผลบนเฟรม
        cv2.putText(img=frame, text=f"FPS:{fps:.2f}", org=(540, 25), fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=0.5, color=color, thickness=1)
        # ถ้าการอ่านเฟรมจากกล้องเป็นไปตามปกติ
        if ret:
            # ส่งเฟรมที่ได้รับไปอัปเดต GUI
            self.ImageUpdate.emit(frame)
            # ส่งสถานะของกล้อง (ถูกเปิดหรือไม่) ไปยัง GUI
            self.CameraStatus.emit(ret)

# เมธอดที่ใช้หยุดการทำงานของ thread และออกจากโปรแกรม
def stop(self):
    # กำหนดให้ thread หยุดทำงาน
    self.ThreadActive = False
    # เรียกเมธอด quit() เพื่อออกจากโปรแกรม
    self.quit()

class boardinfoclass(QThread):
    cpu = pyqtSignal(float)
    ram = pyqtSignal(tuple)
    temp = pyqtSignal(float)

# เมธอดที่ทำงานใน thread เพื่อเก็บข้อมูล CPU, RAM, และอุณหภูมิ
def run(self):
    # กำหนดให้ isThread เป็น True เพื่อให้ thread ทำงาน
    self.isThread = True
    # ทำงานในลูปแบบไม่จบ
    while self.isThread:
        # เรียกใช้เมธอดที่เขียนไว้ใน systeminfo เพื่อดึงข้อมูล CPU, RAM, และอุณหภูมิ
        cpu = systeminfo.getCPU()
        ram = systeminfo.getRAM()
        temp = systeminfo.getTemp()
        # ส่งสัญญาณไปยัง GUI โดยใช้ pyqtSignal
        self.cpu.emit(cpu)
        self.ram.emit(ram)
        self.temp.emit(temp)


# เมธอดที่ใช้หยุดการทำงานของ thread
def stop(self):
    # เรียกเมธอด quit() เพื่อหยุด thread
    self.quit()


class Window_ERROR(QDialog):
    # คอนสตรัคเตอร์ของคลาส
    def __init__(self):
        # เรียก constructor ของคลาสแม่ (superclass)
        super().__init__()

        # โหลดไฟล์ UI และเชื่อมต่อกับคลาส
        self.ui = uic.loadUi("/home/pi/RMUTI-QRCodeDetector/UI/Error_Dialog.ui", self)



class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        
        self.ui = uic.loadUi("/home/pi/RMUTI-QRCodeDetector/UI/mainWindow.ui", self)        # โค้ดนี้ใช้สำหรับโหลดไฟล์ UI และกำหนดค่าเริ่มต้นของหน้าต่าง UI
        self.setWindowIcon=(QIcon("/home/pi/RMUTI-QRCodeDetector/resource/vision.png"))     # กำหนดไอคอนของหน้าต่างด้วยไฟล์รูปภาพที่กำหนด
        self.online_cam = QCameraInfo.availableCameras()                                    # ดึงข้อมูลของกล้องที่พร้อมใช้งาน
        self.cboWebcam.addItems([c.description() for c in self.online_cam])                 # เพิ่มรายการที่สามารถเลือกได้ใน combobox (cboWebcam) จากรายชื่อกล้องที่พร้อมใช้งาน
    
        IO.output(OUTPUT_PINS[0],IO.LOW)
        #ERROR widget

        #Close_app
        self.btnClose.clicked.connect(self.close_application)

        #Syteminfo
        self.resource_info = boardinfoclass()
        self.resource_info.start()
        self.resource_info.cpu.connect(self.getCPU_usage)
        self.resource_info.ram.connect(self.getRAM_usage)
        self.resource_info.temp.connect(self.getTEMP_usage)

        #Stetus
        self.Stetus_lamp = [False,False,False]
        self.action = False
        self.action_con = False
        self.action_st = False
        self.action_start = False
        self.action_Ready  = False

        #controler
        #self.btnAutostart.setEnabled(True)
        self.btnAutostart.setCheckable(True)
        self.btnAutostart.clicked.connect(self.Button_start)

        # Trigger for 1 capture
        self.btn_Trigger.clicked.connect(self.singleDetect_QRCode)

        # Continue Trigger
        self.btn_Trigger_cont.setCheckable(True)
        self.btn_Trigger_cont.clicked.connect(self.continue_QRcode)

        #ButtonReset
        self.btnRST_Total.clicked.connect(self.Button_RSTTotal)
        self.btnRST_OK.clicked.connect(self.Button_RSTOK)
        self.btnRST_NG.clicked.connect(self.Button_RSTNG)
        
        #Stetus
        self.LampReady.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))
        self.LampAlarm.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))
        self.LampStop.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))
        self.Lamp_conveyor.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))
        self.Lamp_flash.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))
        self.Lamp_pusher.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))        
        self.gbControl.setEnabled(False)

        #Camera_Start_Stop
        self.btnOnOff.setCheckable(True)
        self.btnOnOff.clicked.connect(self.camera_action)

        # Counter Data
        self.QRCode_OK = 0
        self.QRCode_NG = 0
        self.QRCode_TOTAL = 0

        # นี่คือการตั้งค่าการตรวจจับเหตุการณ์บนขาของ Raspberry Pi GPIO
        # ตั้งค่าการตรวจจับเหตุการณ์เมื่อมีการเปลี่ยนแปลงของสถานะ (BOTH) ที่ INPUT_PINS[0]
        # และเรียกใช้งาน callback function ที่ชื่อว่า Button_start พร้อมกับกำหนดเวลา debounce ที่ 100 milliseconds
        IO.add_event_detect(INPUT_PINS[0], IO.BOTH, callback=self.Button_start, bouncetime=100)
        
        # ตั้งค่าการตรวจจับเหตุการณ์เมื่อมีการขึ้น (RISING) ที่ INPUT_PINS[2]
        # และเรียกใช้งาน callback function ที่ชื่อว่า singleDetect_QRCode พร้อมกับกำหนดเวลา debounce ที่ 1000 milliseconds
        IO.add_event_detect(INPUT_PINS[2], IO.RISING, callback=self.singleDetect_QRCode, bouncetime=1000)
        
        # ตั้งค่าการตรวจจับเหตุการณ์เมื่อมีการเปลี่ยนแปลงของสถานะ (BOTH) ที่ INPUT_PINS[1]
        # และเรียกใช้งาน callback function ที่ชื่อว่า PLC_Ready พร้อมกับกำหนดเวลา debounce ที่ 100 milliseconds
        IO.add_event_detect(INPUT_PINS[1], IO.BOTH, callback=self.PLC_Ready, bouncetime=100)
        
        # ตั้งค่าการตรวจจับเหตุการณ์เมื่อมีการเปลี่ยนแปลงของสถานะ (BOTH) ที่ INPUT_PINS[5]
        # และเรียกใช้งาน callback function ที่ชื่อว่า Lamp_Conveyor พร้อมกับกำหนดเวลา debounce ที่ 100 milliseconds
        IO.add_event_detect(INPUT_PINS[5], IO.BOTH, callback=self.Lamp_Conveyor, bouncetime=100)
        
        # ตั้งค่าการตรวจจับเหตุการณ์เมื่อมีการเปลี่ยนแปลงของสถานะ (BOTH) ที่ INPUT_PINS[6]
        # และเรียกใช้งาน callback function ที่ชื่อว่า Lamp_Pusher พร้อมกับกำหนดเวลา debounce ที่ 100 milliseconds
        IO.add_event_detect(INPUT_PINS[6], IO.BOTH, callback=self.Lamp_Pusher, bouncetime=100)
        
        # ตั้งค่าการตรวจจับเหตุการณ์เมื่อมีการเปลี่ยนแปลงของสถานะ (BOTH) ที่ INPUT_PINS[7]
        # และเรียกใช้งาน callback function ที่ชื่อว่า Lamp_Flash พร้อมกับกำหนดเวลา debounce ที่ 100 milliseconds
        IO.add_event_detect(INPUT_PINS[7], IO.BOTH, callback=self.Lamp_Flash, bouncetime=100)

        
    def camera_action(self): 
        self.action = not self.action  
        if self.action:
            time.sleep(0.01)
            self.start_camera()
            #print("CAMERA ON")

        else:
            time.sleep(0.01)
            self.stop_camera()
            #print("CAMERA SHUTDOWN")

        
    def start_camera(self):
        IO.output(OUTPUT_PINS[1], IO.LOW) 
        camIndex = self.cboWebcam.currentIndex()
        #Camera thread
        self.camera_thread = CameraThread(camIndex)
        self.camera_thread.ImageUpdate.connect(self.opencv_emit)
        self.camera_thread.start()
        self.btnOnOff.setText("Stop")
        self.gbControl.setEnabled(True)
        
    def stop_camera(self):
        IO.output(OUTPUT_PINS[1], IO.HIGH) 
        self.camera_thread.stop()
        self.picbox1.setStyleSheet("background-color:  rgb(0,0,0);")
        self.btnOnOff.setText("Start")
        self.gbControl.setEnabled(False)
        
    # นี่คือฟังก์ชัน opencv_emit ที่ถูกติดป้าย (@pyqtSlot) เพื่อรับข้อมูลรูปภาพ (Image) จากส่วนอื่นของโปรแกรม
    @pyqtSlot(np.ndarray)
    def opencv_emit(self, Image):
        if self.action_st == False:
            self._mainImage = Image
            original = self.cvt_cv_qt(self._mainImage)
            self.picbox1.setPixmap(original)
            self.picbox1.setScaledContents(True)
        else:
            Image, dec_data = self.Qrcode(Image)
            img_show = self.cvt_cv_qt(Image)
            self.picbox1.setPixmap(img_show)
            self.picbox1.setScaledContents(True)
            self.Data_value.setText(f"Data : {dec_data}")
            
        # แสดงค่าจำนวน QR Code ที่ถูกตรวจสอบผ่าน, ไม่ผ่าน และทั้งหมด ใน Widget ประเภท QLCDNumber
        self.num_ok.display(self.QRCode_OK)
        self.num_ng.display(self.QRCode_NG)
        self.num_total.display(self.QRCode_TOTAL)

        self.ERROR_signal
        self.Ready_Lamp()
        self.Stop_Lamp()
        self.Alarm_Lamp()
    # นี่คือฟังก์ชัน singleDetect_QRCode ที่ถูกเรียกเมื่อมีการตรวจจับเหตุการณ์ที่ขา GPIO ที่กำหนด
    def singleDetect_QRCode(self,channel):

        IO.output(OUTPUT_PINS[2],IO.LOW)                       # ON: BUSY BIT
        IO.output(OUTPUT_PINS[3:4],IO.HIGH) 
        Image, dec_data = self.Qrcode(self._mainImage.copy())  # ประมวลผล QR Code ด้วยฟังก์ชัน Qrcode บนรูปภาพหลัก (self._mainImage)
        img_show = self.cvt_cv_qt(Image)                       # แปลงรูปภาพให้เข้ากับ Qt ด้วยฟังก์ชัน cvt_cv_qt

        self.picbox2.setPixmap(img_show)
        self.picbox2.setScaledContents(True)

        self.Data_value.setText(f"Data : {dec_data}")
        # ตรวจสอบผลลัพธ์จาก QR Code
        if dec_data == '':                        # ถ้าไม่มีผลลัพธ์ (dec_data เป็นสตริงว่าง)
            IO.output(OUTPUT_PINS[4],IO.HIGH)     
            IO.output(OUTPUT_PINS[3],IO.LOW)      # sent Signal.NG
            self.QRCode_NG = self.QRCode_NG + 1   # เพิ่มจำนวน QR Code ที่ไม่ผ่านการตรวจสอบ (NG)

        elif dec_data > "":
            IO.output(OUTPUT_PINS[3],IO.HIGH)
            IO.output(OUTPUT_PINS[4],IO.LOW)      # sent Signal.OK
            self.QRCode_OK = self.QRCode_OK + 1   # เพิ่มจำนวน QR Code ที่ผ่านการตรวจสอบ (OK)

        self.QRCode_TOTAL = self.QRCode_OK + self.QRCode_NG  # คำนวณและแสดงผลรวมของ QR Code ที่ผ่านและไม่ผ่าน
        IO.output(OUTPUT_PINS[2],IO.HIGH)                    # OFF: BUSY BIT after get result
        #print('Camera judment ! ------')
        
    def continue_QRcode(self):
        try:
            self.action_con = not self.action_con
            if self.action_con:
                self.action_st = True
                #self.btnAutostart.setEnabled(True)
                self.btn_Trigger_cont.setStyleSheet("background-color:  rgb(100,250,250);")
            else: 
                self.action_st = False
                #self.btnAutostart.setEnabled(False)
                self.btn_Trigger_cont.setStyleSheet("background-color:  rgb(250,0,0);")
        except Exception as e:
         # Handle exceptions if needed
            print(f"An error Trigger_cont: {e}")  
            
    def Qrcode (self, Image):                                      # นี่คือฟังก์ชัน Qrcode ที่ใช้สำหรับการประมวลผล QR Code บนรูปภาพ
        dec_data = ""
        gray_frame = cv2.cvtColor(Image, code= cv2.COLOR_BGR2GRAY) # แปลงรูปภาพเป็นภาพขาวดำ (grayscale)
        decode_object = decode(gray_frame)                         # ใช้ฟังก์ชัน decode จากไลบรารี pyzbar เพื่อตรวจจับ QR Code

        bx,by,bw,bh = 0,0,150,80
        tx,ty = 30,60
        font = cv2.FONT_HERSHEY_DUPLEX
        font_Scale = 2
        thickness = 2
       
        if decode_object:                             # ตรวจสอบว่ามีการตรวจจับ QR Code หรือไม่
            for obj in decode_object:
                dec_data = obj.data.decode('utf-8')
                
                # Background : Green 
                cv2.rectangle(Image,pt1=(bx,bx),pt2=(bx+bw, by+bh),color=(0, 255, 0),thickness=-1)
                # Text : OK
                cv2.putText(img=Image,text="OK",org=(tx,ty),fontFace=font,fontScale=font_Scale,color=(255,0,0),thickness=thickness)
                # กำหนดกรอบสีน้ำเงินรอบ QR Code
                cv2.rectangle(Image,pt1=(obj.rect.left, obj.rect.top),pt2=(obj.rect.left + obj.rect.width, obj.rect.top + obj.rect.height),color=(255, 0, 0),thickness=2)
        else:   # ถ้าไม่มีการตรวจจับ QR Code            
            # Background : Red 
            cv2.rectangle(Image,pt1=(bx,bx),pt2=(bx+bw, by+bh),color=(0, 0, 255),thickness=-1)
            # Text : NG
            cv2.putText(img=Image,text="NG",org=(tx,ty),fontFace=font,fontScale=font_Scale,color=(255,0,0),thickness=thickness)
        # ส่งค่ารูปภาพที่ปรับแล้วและข้อมูลที่ถูกถอดออกจากรหัส QR Code กลับ        
        dec_data = dec_data
        Image = Image

        return Image,dec_data

    def cvt_cv_qt(self, Image):                                    # นี่คือฟังก์ชัน cvt_cv_qt ที่ใช้สำหรับแปลงรูปภาพจาก OpenCV เป็นภาพที่ Qt สามารถแสดงได้
        rgb_img = cv2.cvtColor(src=Image, code=cv2.COLOR_BGR2RGB)  # แปลงรูปภาพจากรูปแบบ BGR ไปยังรูปแบบ RGB
        h, w, ch = rgb_img.shape       # ดึงขนาดของรูปภาพ
        bytes_per_line = ch * w        # คำนวณจำนวนไบต์ที่ใช้ในแต่ละบรรทัด
        cvt2QtFormat = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888) # สร้าง QImage จากข้อมูลรูปภาพที่แปลงแล้ว
        pixmap = QPixmap.fromImage(cvt2QtFormat)  # สร้าง QPixmap จาก QImage
        return pixmap  # ส่งค่า QPixmap กลับ
    
    def PLC_Ready(self,channel):  # นี่คือฟังก์ชัน PLC_Ready ตรวจจับเหตุการณ์ที่ขา GPIO ที่กำหนด
        if IO.input(INPUT_PINS[1]) == IO.HIGH :
            time.sleep(0.01)
            self.Stetus_lamp[0]= False
            self.Stetus_lamp[1]= True
        else:
            time.sleep(0.01)
            self.Stetus_lamp[0]= True
            self.Stetus_lamp[1]= False
            
    def Button_RSTTotal(self):

        self.QRCode_TOTAL = 0
        self.QRCode_NG = 0
        self.QRCode_OK = 0
 
    def Button_RSTOK(self):

        self.QRCode_OK = 0
        self.QRCode_TOTAL =  self.QRCode_TOTAL

    def Button_RSTNG(self):

        self.QRCode_NG = 0
        self.QRCode_TOTAL =  self.QRCode_TOTAL
        
    def Button_start(self,channel):
        try:
            self.action_start = not self.action_start
            if  self.action_start:
                time.sleep(0.01)
                IO.output(OUTPUT_PINS[5],IO.LOW)
                self.btnAutostart.setStyleSheet("background-color: rgb(200, 0, 0);")
                self.btnAutostart.setText("Stop")
                self.Stetus_lamp[2]= False
                self.btn_Trigger.setEnabled(False)
                self.btn_Trigger_cont.setEnabled(False)
                #print("start")
                time.sleep(0.1)
                IO.output(OUTPUT_PINS[5],IO.HIGH)

            else:
                time.sleep(0.01)
                IO.output(OUTPUT_PINS[6],IO.LOW)
                self.btnAutostart.setStyleSheet("background-color:  rgb(100,250,55);")
                self.btnAutostart.setText("Start")
                self.Stetus_lamp[2]= True
                self.btn_Trigger.setEnabled(True)
                self.btn_Trigger_cont.setEnabled(True)
                time.sleep(0.1)
                IO.output(OUTPUT_PINS[6],IO.HIGH)
                #print("stop")

        except Exception as e:
            # Handle exceptions if needed
                print(f"An error occurred_BTN_Start: {e}")   
                
    def Ready_Lamp (self):
        if self.Stetus_lamp[0]:
            self.LampReady.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_ON.png"))

        else :
            self.LampReady.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))

    def Alarm_Lamp (self):
        if self.Stetus_lamp[1]:
            self.LampAlarm.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_Alarm.png"))

        else :
            self.LampAlarm.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))

    def Stop_Lamp (self):
        if self.Stetus_lamp[2]:
            self.LampStop.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_Stop.png"))

        else :
            self.LampStop.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))

    def Lamp_Conveyor (self,channel):
        try:
            if IO.input(INPUT_PINS[5]) == IO.HIGH:
                time.sleep(0.01)
                #print("BTN_conv: released")
                self.Lamp_conveyor.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))

            else :
                time.sleep(0.01)
                #print("BTN_conv: pressed")
                self.Lamp_conveyor.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_ON.png"))

        except Exception as e:
            # Handle exceptions if needed
                print(f"An error occurred_BTN_cv: {e}")  

    def Lamp_Pusher (self,channel):
        try:
            if IO.input(INPUT_PINS[6]) == IO.HIGH:
                time.sleep(0.01)
                #print("BTN_push: released")
                self.Lamp_pusher.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))

            else :
                time.sleep(0.01)
                #print("BTN_push: pressed")
                self.Lamp_pusher.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_ON.png"))
                
        except Exception as e:
            # Handle exceptions if needed
                print(f"An error occurred_BTN_push: {e}")  

    def Lamp_Flash (self,channel):
        try:
            if IO.input(INPUT_PINS[7]) == IO.HIGH:
                time.sleep(0.01)
                #print("BTN_flash: released")
                self.Lamp_flash.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_OFF.png"))

            else :
                time.sleep(0.01)
                #print("BTN_flash: pressed")
                self.Lamp_flash.setPixmap(QPixmap("/home/pi/RMUTI-QRCodeDetector/resource/LED_ON.png"))
                
        except Exception as e:
            # Handle exceptions if needed
                print(f"An error occurred_BTN_fsh: {e}")  

    def getCPU_usage(self,cpu):
        self.Data_value_2.setText(f"CPU: {str(cpu)}" + "%")
        # if cpu > 15: self.Data_value_2.setStyleSheet("font color: rgb(23, 63, 95);")
        # elif cpu > 25: self.Data_value_2.setStyleSheet("font color: rgb(32, 99, 155);")
        # elif cpu > 45: self.Data_value_2.setStyleSheet("font color: rgb(60, 174, 163);")
        # elif cpu > 65: self.Data_value_2.setStyleSheet("font color: rgb(246, 160, 70);")
        # elif cpu > 85: self.Data_value_2.setStyleSheet("font color: rgb(255, 63, 95);")

    def getRAM_usage(self,ram):
        self.Data_value_3.setText(f"RAM: {str(ram[2])}" + "%")
        # if ram[2] > 15: self.Data_value_3.setStyleSheet("color: rgb(23, 63, 95);")
        # elif ram[2] > 25: self.Data_value_3.setStyleSheet("color: rgb(32, 99, 155);")
        # elif ram[2] > 45: self.Data_value_3.setStyleSheet("color: rgb(60, 174, 163);")
        # elif ram[2] > 65: self.Data_value_3.setStyleSheet("color: rgb(246, 160, 70);")
        # elif ram[2] > 85: self.Data_value_3.setStyleSheet("color: rgb(255, 63, 95);")

    def getTEMP_usage(self,temp):
        self.Data_value_4.setText(f"Temp CPU: {str(temp)}" + "*C")
        # if temp > 15: self.Data_value_4.setStyleSheet("color: rgb(23, 63, 95);")
        # elif temp > 25: self.Data_value_4.setStyleSheet("color: rgb(32, 99, 155);")
        # elif temp > 45: self.Data_value_4.setStyleSheet("color: rgb(60, 174, 163);")
        # elif temp > 65: self.Data_value_4.setStyleSheet("color: rgb(246, 160, 70);")
        # elif temp > 85: self.Data_value_4.setStyleSheet("color: rgb(255, 63, 95);")

    def ERROR_signal(self):   # นี่คือฟังก์ชัน ERROR_signal ที่ใช้สำหรับการตรวจสอบสถานะของขา GPIO และแสดงหน้าต่างข้อผิดพลาด (Window_ERROR)
        if IO.input(INPUT_PINS[5])==IO.LOW:
            time.sleep(0.01)
            self.Error_Show = Window_ERROR()
            self.Stetus_lamp[1] = True
            self.Error_Show.show()
            self.Error_Show.label_Error.setText("Please press the STOP Button 3 times to reset.")
        else:
            time.sleep(0.01)
            self.Stetus_lamp[1] = False
            self.Error_Show.close()

    def close_application(self):
        time.sleep(0.01)
        IO.cleanup()
        app.quit()  # Use app.quit() to exit the application

if __name__ == "__main__":
    # IO LIST
    #INPUT
    #0 GPIO NO.7     = Signal.start
    #1 GPIO NO.11    = PLC.Ready
    #2 GPIO NO.13    = Signal.Trigger
    #3 GPIO NO.15    = Signal.ERROR
    #4 GPIO NO.29    = -
    #5 GPIO NO.31    = ST.conveyor
    #6 GPIO NO.33    = ST.pusher
    #7 GPIO NO.35    = ST.Flash

    #OUTPUT
    #0 GPIO NO.12    = Software.Ready
    #1 GPIO NO.16    = Camera ON
    #2 GPIO NO.18    = Camera Busy
    #3 GPIO NO.22    = Signal.NG
    #4 GPIO NO.32    = Signal.OK
    #5 GPIO NO.36    = btn.Start
    #6 GPIO NO.38    = btn.Stop
    #7 GPIO NO.40    =  
    # กำหนดขาของ Input และ Output ที่จะใช้
    INPUT_PINS = [7,11,13,15,29,31,33,35]
    OUTPUT_PINS = [12,16,18,22,32,36,38,40]

    IO.setwarnings(False)  # ปิดการแจ้งเตือนข้อผิดพลาดจากไลบรารี GPIO
    IO.setmode(IO.BOARD)   # กำหนดโหมดการทำงานของ GPIO เป็นโหมด Board
    
    # กำหนดขา Input ทั้งหมดในลิสต์ INPUT_PINS เป็น Input และใช้ Pull-up resistor
    for input_pin in INPUT_PINS:
        IO.setup(input_pin, IO.IN, pull_up_down=IO.PUD_UP) 
        
    # กำหนดขา Output ทั้งหมดในลิสต์ OUTPUT_PINS เป็น Output และตั้งค่าค่าเริ่มต้นเป็น HIGH
    for output_pin in OUTPUT_PINS:
        IO.setup(output_pin, IO.OUT, initial=IO.HIGH)
    
    app = QApplication([])      
    window = MainWindow()
    window.showFullScreen()
    #window.show()
    app.exec()

    cv2.destroyAllWindows()
    IO.cleanup()
