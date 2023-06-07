# ----- Imports -------------------------------------------------------

# ROS2 libs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# PyQt5 Imports
from PyQt5.QtCore import QDateTime, Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget, QFileDialog, QButtonGroup, QFormLayout)
from PyQt5.QtGui import QPixmap
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from sklearn.cluster import DBSCAN
from pyqtgraph.pgcollections import OrderedDict
import datetime

# Local File Imports
from .gui_common import *
from .parseFrame import *
import serial


class uartParser():
    def __init__(self):
        # Set this option to 1 to save UART output from the radar device
        self.saveBinary = 0
        self.replay = 0
        if(self.saveBinary == 1):
            self.binData = bytearray(0)
            self.uartCounter = 0
            self.framesPerFile = 100
            self.first_file = True
            self.filepath = datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
            
        self.parserType = "Standard"

        # Data storage
        self.now_time = datetime.datetime.now().strftime('%Y%m%d-%H%M')

        cliCom="/dev/ttyUSB0"
        dataCom="/dev/ttyUSB1"
        self.cliCom = serial.Serial(cliCom, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)
        self.dataCom = serial.Serial(dataCom, 921600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)

    # This function is always called - first read the UART, then call a function to parse the specific demo output
    # This will return 1 frame of data. This must be called for each frame of data that is expected. It will return a dict containing all output info
    # Point Cloud and Target structure are liable to change based on the lab. Output is always cartesian.
    def readAndParseUart(self):
        
        self.fail = 0
        if (self.replay):
            return self.replayHist()
    
        # Find magic word, and therefore the start of the frame
        index = 0
        magicByte = self.dataCom.read(1)
        frameData = bytearray(b'')
        while (1):
            # If the device doesnt transmit any data, the COMPort read function will eventually timeout
            # Which means magicByte will hold no data, and the call to magicByte[0] will produce an error
            # This check ensures we can give a meaningful error
            if (len(magicByte) < 1):
                print ("ERROR: No data detected on COM Port, read timed out")
                print("\tBe sure that the device is in the proper mode, and that the cfg you are sending is valid")
                magicByte = self.dataCom.read(1)
                
            # Found matching byte
            elif (magicByte[0] == UART_MAGIC_WORD[index]):
                index += 1
                frameData.append(magicByte[0])
                if (index == 8): # Found the full magic word
                    break
                magicByte = self.dataCom.read(1)
                
            else:
                # When you fail, you need to compare your byte against that byte (ie the 4th) AS WELL AS compare it to the first byte of sequence
                # Therefore, we should only read a new byte if we are sure the current byte does not match the 1st byte of the magic word sequence
                if (index == 0): 
                    magicByte = self.dataCom.read(1)
                index = 0 # Reset index
                frameData = bytearray(b'') # Reset current frame data
        
        # Read in version from the header
        versionBytes = self.dataCom.read(4)
        
        frameData += bytearray(versionBytes)

        # Read in length from header
        lengthBytes = self.dataCom.read(4)
        frameData += bytearray(lengthBytes)
        frameLength = int.from_bytes(lengthBytes, byteorder='little')
        
        # Subtract bytes that have already been read, IE magic word, version, and length
        # This ensures that we only read the part of the frame in that we are lacking
        frameLength -= 16 

        # Read in rest of the frame
        frameData += bytearray(self.dataCom.read(frameLength))

        # If save binary is enabled
        if(self.saveBinary == 1):
            self.binData += frameData
            # Save data every framesPerFile frames
            self.uartCounter += 1
            if (self.uartCounter % self.framesPerFile == 0):
                # First file requires the path to be set up
                if(self.first_file is True): 
                    if(os.path.exists('binData/') == False):
                        # Note that this will create the folder in the caller's path, not necessarily in the Industrial Viz Folder                        
                        os.mkdir('binData/')
                    os.mkdir('binData/'+self.filepath)
                    self.first_file = False
                toSave = bytes(self.binData)
                fileName = 'binData/' + self.filepath + '/pHistBytes_' + str(math.floor(self.uartCounter/self.framesPerFile)) + '.bin'
                bfile = open(fileName, 'wb')
                bfile.write(toSave)
                bfile.close()
                # Reset binData and missed frames
                self.binData = []
 
        # frameData now contains an entire frame, send it to parser
        if (self.parserType == "Standard"):
            outputDict = parseStandardFrame(frameData)
        else:
            print ('FAILURE: Bad parserType')
        
        return outputDict


class parseUartThread(QThread):
    fin = pyqtSignal('PyQt_PyObject')

    def __init__(self, uParser):
        QThread.__init__(self)
        self.parser = uParser
        self.outputDict = None

    def run(self):            
        self.outputDict = self.parser.readAndParseUart()
        #print("output ->", self.outputDict)
        self.fin.emit(self.outputDict)


class mmwave_ros2(Node):

    def __init__(self):
        super().__init__('mmwave_ros2')
        print("Start mmwave_ros2 node...")
        
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        

        self.parser = uartParser()
        self.parser.frameTime = 50

        # init threads and timers
        self.uart_thread = parseUartThread(self.parser)
        self.uart_thread.start(priority=QThread.HighestPriority)
        #self.uart_thread.fin.connect(self.fabio_func) # prova

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self): 
       
        #self.uart_thread = parseUartThread(self.parser)
        #self.uart_thread.start(priority=QThread.HighestPriority)
        #self.uart_thread.fin.connect(self.fabio_func) # prova
        
        print("Reading mmwave data...")
        print("outttt", self.uart_thread.outputDict)

     
    


        
        
        

def main():
    
    rclpy.init()
    node = mmwave_ros2()

    #run_gui(show=True) # run the "gui_main.py" script to show the GUI and make the mmWave sensor work
    rclpy.spin(node)
    


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
