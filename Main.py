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

    def __init__(self, cameraIndex):
        super().__init__()

        self.camIndex = cameraIndex
        self.cap = None
        self.ThreadActive = False

    def Textcolor(self,fps):
        textcolor_fps = (0, 0, 0)
        if fps < 5 : textcolor_fps = (255, 0, 0)
        if fps > 15 : textcolor_fps = (250,80,10)
        if fps < 25 : textcolor_fps = (160,150,0)
        if fps > 30 : textcolor_fps = (0, 250, 0) 
        
        return textcolor_fps

    def run(self):
        self.Cap = cv2.VideoCapture(self.camIndex)
        self.Cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        self.Cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)

        self.ThreadActive = True
        start = 0
        end = 0
        while self.ThreadActive:
            #time.sleep(0.01)
            ret, frame = self.Cap.read()
            #frame = cv2.flip(frame, 1)
            end = time.time()
            total = start - end
            fps = 1 / -(total)
            start = end
            color = self.Textcolor(fps)
            fps = int(fps)
            #fps = str(fps)
            cv2.putText(img=frame,text=f"FPS:{fps:.2f}",org=(540,25),fontFace=cv2.FONT_HERSHEY_DUPLEX,fontScale=0.5,color=color,thickness=1)

            if ret:
                self.ImageUpdate.emit(frame)
                self.CameraStatus.emit(ret)

    def stop(self):
        self.ThreadActive = False
        self.quit()

class boardinfoclass(QThread):
    cpu = pyqtSignal(float)
    ram = pyqtSignal(tuple)
    temp = pyqtSignal(float)

    def run(self):
        self.isThread= True
        while self.isThread:
            cpu = systeminfo.getCPU()
            ram = systeminfo.getRAM()
            temp = systeminfo.getTemp()
            self.cpu.emit(cpu)
            self.ram.emit(ram)
            self.temp.emit(temp)

    def stop(self):
        self.quit()
class Window_ERROR(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("/home/pi/RMUTI-QRCodeDetector/UI/Error_Dialog.ui",self)


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        
        self.ui = uic.loadUi("/home/pi/RMUTI-QRCodeDetector/UI/mainWindow.ui", self)
        self.setWindowIcon=(QIcon("/home/pi/RMUTI-QRCodeDetector/resource/vision.png"))
        self.online_cam = QCameraInfo.availableCameras()
        self.cboWebcam.addItems([c.description() for c in self.online_cam])
    
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
        self.Stetus_lamp = [True,True,True]
        self.Stetus_machine = [True,True,True]
        self.action = False
        self.action_con = False
        self.action_st = False
        self.action_Ready  = False

        #controler
        self.btnAutostart.setEnabled(False)
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
        self.LampReady.setPixmap(QPixmap("resource/LED_OFF.png"))
        self.LampAlarm.setPixmap(QPixmap("resource/LED_OFF.png"))
        self.LampStop.setPixmap(QPixmap("resource/LED_OFF.png"))
        self.Lamp_conveyor.setPixmap(QPixmap("resource/LED_OFF.png"))
        self.Lamp_flash.setPixmap(QPixmap("resource/LED_OFF.png"))
        self.Lamp_pusher.setPixmap(QPixmap("resource/LED_OFF.png"))        
        self.gbControl.setEnabled(False)

        #Camera_Start_Stop
        self.btnOnOff.setCheckable(True)
        self.btnOnOff.clicked.connect(self.camera_action)

        # Counter Data
        self.QRCode_OK = 0
        self.QRCode_NG = 0
        self.QRCode_TOTAL = 0

        IO.add_event_detect(INPUT_PINS[0], IO.BOTH,callback=self.Button_start,bouncetime=100)
        IO.add_event_detect(INPUT_PINS[2], IO.RISING,callback=self.singleDetect_QRCode,bouncetime=1000)
        IO.add_event_detect(INPUT_PINS[1], IO.BOTH,callback=self.PLC_Ready,bouncetime=100)

        IO.add_event_detect(INPUT_PINS[5], IO.BOTH,callback=self.Lamp_Conveyor,bouncetime=100)
        IO.add_event_detect(INPUT_PINS[6], IO.BOTH,callback=self.Lamp_Pusher,bouncetime=100)
        IO.add_event_detect(INPUT_PINS[7], IO.BOTH,callback=self.Lamp_Flash,bouncetime=100)
        
        
    def PLC_Ready(self,channel):
        if IO.input(INPUT_PINS[1]) == IO.HIGH :
            time.sleep(0.01)
            self.Stetus_lamp[0]= False
            self.Stetus_lamp[1]= True
        else:
            time.sleep(0.01)
            self.Stetus_lamp[0]= True

    def camera_action(self): 
        self.action = not self.action  
        if self.action:
            self.start_camera()
            print("CAMERA ON")

        else:
            self.stop_camera()
            print("CAMERA SHUTDOWN")

        
    def start_camera(self):
        camIndex = self.cboWebcam.currentIndex()
        #Camera thread
        self.camera_thread = CameraThread(camIndex)
        self.camera_thread.ImageUpdate.connect(self.opencv_emit)
        self.camera_thread.start()
        self.btnOnOff.setText("Stop")
        self.gbControl.setEnabled(True)
        
    def stop_camera(self):
        self.camera_thread.stop()
        self.picbox1.setStyleSheet("background-color:  rgb(0,0,0);")
        self.btnOnOff.setText("Start")
        self.gbControl.setEnabled(False)

    @pyqtSlot(np.ndarray)
    def opencv_emit(self, Image):
        self._mainImage = Image
        original = self.cvt_cv_qt(self._mainImage)
        self.picbox1.setPixmap(original)
        self.picbox1.setScaledContents(True)

        self.num_ok.display(self.QRCode_OK)
        self.num_ng.display(self.QRCode_NG)
        self.num_total.display(self.QRCode_TOTAL)

        self.ERROR_signal
        self.Ready_Lamp()
        self.Stop_Lamp()
        self.Alarm_Lamp()

    def singleDetect_QRCode(self,channel):

        IO.output(OUTPUT_PINS[2],IO.LOW)          # ON: BUSY BIT
        IO.output(OUTPUT_PINS[3:4],IO.HIGH) 
        Image, dec_data = self.Qrcode(self._mainImage.copy())
        img_show = self.cvt_cv_qt(Image)
        self.picbox2.setPixmap(img_show)
        self.picbox2.setScaledContents(True)

        self.Data_value.setText(f"Data : {dec_data}")

        if dec_data == '':
            # Turn NG bit no result
            IO.output(OUTPUT_PINS[4],IO.HIGH)     # OK
            IO.output(OUTPUT_PINS[3],IO.LOW)      # NG

            self.QRCode_NG = self.QRCode_NG + 1

        elif dec_data > "":
            # Turn OK bit has result
            IO.output(OUTPUT_PINS[3],IO.HIGH)
            IO.output(OUTPUT_PINS[4],IO.LOW)

            self.QRCode_OK = self.QRCode_OK + 1

        self.QRCode_TOTAL = self.QRCode_OK + self.QRCode_NG
        IO.output(OUTPUT_PINS[2],IO.HIGH)         # OFF: BUSY BIT after get result
        print('Camera judment ! ------')
        


    def continue_QRcode(self):
        try:
            self.action_con = not self.action_con
            if self.action_con:
                IO.output(OUTPUT_PINS[1], IO.LOW) 
                self.btnAutostart.setEnabled(True)
                self.btn_Trigger_cont.setStyleSheet("background-color:  rgb(100,250,250);")
            else:
                IO.output(OUTPUT_PINS[1], IO.HIGH) 
                self.btnAutostart.setEnabled(False)
                self.btn_Trigger_cont.setStyleSheet("background-color:  rgb(250,0,0);")
        except Exception as e:
         # Handle exceptions if needed
            print(f"An error Trigger_cont: {e}")  
            
    def Qrcode (self, Image):
        dec_data = ""
        gray_frame = cv2.cvtColor(Image, code= cv2.COLOR_BGR2GRAY)
        decode_object = decode(gray_frame)

        bx,by,bw,bh = 0,0,150,80
        tx,ty = 30,60
        font = cv2.FONT_HERSHEY_DUPLEX
        font_Scale = 2
        thickness = 2

        if decode_object:
            for obj in decode_object:
                dec_data = obj.data.decode('utf-8')
                
                # Background : Green 
                cv2.rectangle(Image,pt1=(bx,bx),pt2=(bx+bw, by+bh),color=(0, 255, 0),thickness=-1)
                # Text : OK
                cv2.putText(img=Image,text="OK",org=(tx,ty),fontFace=font,fontScale=font_Scale,color=(255,0,0),thickness=thickness)
                
                cv2.rectangle(Image,pt1=(obj.rect.left, obj.rect.top),pt2=(obj.rect.left + obj.rect.width, obj.rect.top + obj.rect.height),color=(255, 0, 0),thickness=2)
        else:
            # Background : Red 
            cv2.rectangle(Image,pt1=(bx,bx),pt2=(bx+bw, by+bh),color=(0, 0, 255),thickness=-1)
            # Text : NG
            cv2.putText(img=Image,text="NG",org=(tx,ty),fontFace=font,fontScale=font_Scale,color=(255,0,0),thickness=thickness)
                
        dec_data = dec_data
        Image = Image

        return Image,dec_data

    def cvt_cv_qt(self, Image):
        rgb_img = cv2.cvtColor(src=Image, code=cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        cvt2QtFormat = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(cvt2QtFormat)
        return pixmap

    
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
            if  IO.input(INPUT_PINS[0]) == IO.HIGH:
                time.sleep(0.01)
                #IO.output(OUTPUT_PINS[5],IO.LOW)
                self.btnAutostart.setStyleSheet("background-color: rgb(200, 0, 0);")
                self.btnAutostart.setText("Stop")
                self.Stetus_lamp[2]= False
                self.btn_Trigger.setEnabled(False)
                self.btn_Trigger_cont.setEnabled(False)
                print("start")
                #IO.output(OUTPUT_PINS[5],IO.HIGH)

            else:
                time.sleep(0.01)
                #IO.output(OUTPUT_PINS[5],IO.HIGH)
                self.btnAutostart.setStyleSheet("background-color:  rgb(100,250,55);")
                self.btnAutostart.setText("Start")
                self.Stetus_lamp[2]= True
                self.btn_Trigger.setEnabled(True)
                self.btn_Trigger_cont.setEnabled(True)
                print("stop")

        except Exception as e:
            # Handle exceptions if needed
                print(f"An error occurred_BTN_Start: {e}")   
                
    def Ready_Lamp (self):
        if self.Stetus_lamp[0]:
            self.LampReady.setPixmap(QPixmap("resource/LED_ON.png"))
            self.Stetus_lamp[1]= False
            self.Stetus_lamp[2]= False
        else :
            self.LampReady.setPixmap(QPixmap("resource/LED_OFF.png"))

    def Alarm_Lamp (self):
        if self.Stetus_lamp[1]:
            self.LampAlarm.setPixmap(QPixmap("resource/LED_Alarm.png"))

        else :
            self.LampAlarm.setPixmap(QPixmap("resource/LED_OFF.png"))

    def Stop_Lamp (self):
        if self.Stetus_lamp[2]:
            self.LampStop.setPixmap(QPixmap("resource/LED_Stop.png"))
            self.Stetus_lamp[0] = False

        else :
            self.LampStop.setPixmap(QPixmap("resource/LED_OFF.png"))

    def Lamp_Conveyor (self,channel):
        try:
            if IO.input(INPUT_PINS[5]) == IO.HIGH:
                time.sleep(0.01)
                print("BTN_conv: released")
                self.Lamp_conveyor.setPixmap(QPixmap("resource/LED_OFF.png"))

            else :
                time.sleep(0.01)
                print("BTN_conv: pressed")
                self.Lamp_conveyor.setPixmap(QPixmap("resource/LED_ON.png"))

        except Exception as e:
            # Handle exceptions if needed
                print(f"An error occurred_BTN_cv: {e}")  

    def Lamp_Pusher (self,channel):
        try:
            if IO.input(INPUT_PINS[6]) == IO.HIGH:
                time.sleep(0.01)
                print("BTN_push: released")
                self.Lamp_pusher.setPixmap(QPixmap("resource/LED_OFF.png"))

            else :
                time.sleep(0.01)
                print("BTN_push: pressed")
                self.Lamp_pusher.setPixmap(QPixmap("resource/LED_ON.png"))
                
        except Exception as e:
            # Handle exceptions if needed
                print(f"An error occurred_BTN_push: {e}")  

    def Lamp_Flash (self,channel):
        try:
            if IO.input(INPUT_PINS[7]) == IO.HIGH:
                time.sleep(0.01)
                print("BTN_flash: released")
                self.Lamp_flash.setPixmap(QPixmap("resource/LED_OFF.png"))

            else :
                time.sleep(0.01)
                print("BTN_flash: pressed")
                self.Lamp_flash.setPixmap(QPixmap("resource/LED_ON.png"))
                
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

    def ERROR_signal(self):
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
    # GPIO NO.7     = btn Start
    # GPIO NO.11    = PLC Ready
    # GPIO NO.13    = Continous Trigger
    # GPIO NO.15    = 
    # GPIO NO.29    =
    # GPIO NO.31    =
    # GPIO NO.33    =
    # GPIO NO.35    =  

    #OUTPUT
    # GPIO NO.12    = Software Ready
    # GPIO NO.16    = Camera ON
    # GPIO NO.18    = Camera Busy
    # GPIO NO.22    = Camera NG
    # GPIO NO.32    = Camera OK
    # GPIO NO.36    =
    # GPIO NO.38    =
    # GPIO NO.40    =  
    
    INPUT_PINS = [7,11,13,15,29,31,33,35]
    OUTPUT_PINS = [12,16,18,22,32,36,38,40]

    IO.setwarnings(False)
    IO.setmode(IO.BOARD)

    for input_pin in INPUT_PINS:
        IO.setup(input_pin, IO.IN, pull_up_down=IO.PUD_UP) 

    for output_pin in OUTPUT_PINS:
        IO.setup(output_pin, IO.OUT, initial=IO.HIGH)
    
    

    app = QApplication([])      
    window = MainWindow()
    window.showFullScreen()
    #window.show()
    app.exec()

    cv2.destroyAllWindows()
    IO.cleanup()