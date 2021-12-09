from myimports import *

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pixy.init ()
pixy.change_prog ("color_connected_components");

class Blocks (Structure):
  _fields_ = [ ("m_signature", c_uint),
    ("m_x", c_uint),
    ("m_y", c_uint),
    ("m_width", c_uint),
    ("m_height", c_uint),
    ("m_angle", c_uint),
    ("m_index", c_uint),
    ("m_age", c_uint) ]
  
class RadioThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        global coord_list
        global fieldRight
        temp_list = []
        i = 0
        ser = serial.Serial('/dev/ttyACM1',115200,timeout=.1)
        time.sleep(1)
        start_time = time.time()
        while True:
            serdata = ser.readline().decode('utf-8').rstrip()
            if (time.time()-start_time > 0.5):
                if serdata == 'a':
                    i = 0
                    #print(temp_list)
                    if len(temp_list) == 6:
                        if not fieldRight: #flips x axis if home is on left side of field
                            temp_list[0] = -temp_list[0] + 521
                            temp_list[2] = -temp_list[2] + 521
                            temp_list[4] = -temp_list[4] + 521
                            
                        coord_list = temp_list.copy()
                        
                        #print(coord_list)
                        #print('.')
                    #time.sleep(0.05)
                    temp_list.clear()
                    time.sleep(0.002)
                else:
                    if serdata is not '' and serdata.isdigit():
                        time.sleep(0.002)
                        temp_list.append(int(serdata))
                        i += 1
                    else:
                        temp_list.append(0)
                        i += 1
            #ser.flush()
            #return serdata
        
            
class SerialThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):
        #global byte_to_send
        global anglecurrent
        ser = serial.Serial('/dev/ttyACM0',115200,timeout=.1)
        #ser.flush()
        time.sleep(1)
        while True:
            #ser = serial.Serial('/dev/ttyACM0',115200,timeout=.1)
            try:
                serdata = ser.readline().decode('utf-8').rstrip()
            except:
                serdata = ''
            #print(serdata)
            #print("thread: " + byte_to_send)
            ser.write(bytes((byte_to_send+'\n').encode('utf-8')))
            #time.sleep(0.05)
            ser.flush()
            #return serdata
            
            if serdata != '':
                zgyrorad = float(serdata)/16384
                zgyrodeg = 57.2963 * (zgyrorad + .01125)
                anglepast = zgyrodeg
                anglecurrent = anglecurrent + anglepast
                ##print(anglecurrent)
                
class Camera:
    def isDetected(self):
        count = pixy.ccc_get_blocks (1, blocks)
        return (count>0)
    
    def get(self):
        count = pixy.ccc_get_blocks (1, blocks)
        xPixy = blocks[0].m_x
        #print(xPixy)
        return xPixy     
    
    def isCentered(self,xPixy):
        return (153<xPixy<163)

class Gyroscope:
    def calibrate(self, old, new, direction):
        if (direction == '1'):
            calibrate = 0
        if (direction == '2'):
            calibrate = 180
        if (direction == '3'):
            calibrate = -90
        if (direction == '4'):
            calibrate = 90
        if (direction == '5'):
            calibrate = -45
        if (direction == '6'):
            calibrate = 45
        if (direction == '7'):
            calibrate = -135
        if (direction == '8'):
            calibrate = 135
        #get old
        #get new
        if (old[0] == new[0]):
            return 0
        return (calibrate+math.atan((old[1]-new[1])/(new[0]-old[0]))) #returns angle
        #return 0
    '''
    def read(self, data, anglecurrent):
        if data != '':
            zgyrorad = float(data)/16384
            zgyrodeg = 57.2963 * (zgyrorad - -.0105)
            anglepast = zgyrodeg
            anglecurrent += anglepast
            time.sleep(0.05)
            return anglecurrent
        else:
            return
    '''
    
class Coordinates:
    global coord_list
    global playerBlue
    global fieldRight
    
    def Player(self, coord_list):
        use_list = coord_list.copy()
        if playerBlue:
            return [use_list[2], use_list[3]]
        else:
            return [use_list[0], use_list[1]]
        
    def Ball(self, coord_list):
        use_list = coord_list.copy()
        return [use_list[4], use_list[5]]
        
    '''
    def getBallDistance(self):
        player = self.Player()
        return (sqrt((self.player[0]-coord_list[4])^2+(self.player[1]-coord_list[5])^2))
    
    def getKickPosition(self):
        #y=mx+b
        m = (self.Ball(1)-self.Goal(1))/(self.Ball(0)-self.Goal(0))
        b = self.Goal(0)
        x = self.Ball(0)+sqrt((ballDistanceTerminate^2)/2)
        y = m*x+b
        return [x, y]
    
    def getKickAngle(self, whichside):
        theta = math.atan(abs((self.Ball(1)-self.Goal(1))/(self.Ball(0)-self.Goal(0))))
        if (whichside):
            theta = theta * -1 #this corresponds to players angle
        return theta
        
    def whichSide(self):
        return (self.Ball[0]>48)
    '''
    
class Drive:
    #int sent to arduino
    stop = '0'
    forward = '1'
    backward = '2'
    strafeleft = '3'
    straferight = '4'
    forwardleft = '5'
    forwardright = '6'
    backwardleft = '7'
    backwardright = '8'
    turnleft = '9'
    turnright = '10'
    shiftup = '11'
    shiftdown = '12'       

#Pixy setup
pixy.init ()
pixy.change_prog ("color_connected_components");
blocks = BlockArray(100)
frame = 0

#Serial setup
ser = serial.Serial('/dev/ttyACM0',115200,timeout=.1)
ser.flush()
time.sleep(1)
byte_to_send = '0'

ser = serial.Serial('/dev/ttyACM1',115200,timeout=.1)
ser.flush()
time.sleep(1)

#Gyro setup
anglecurrent = 0

#Class setup

camera = Camera()
gyro = Gyroscope()
coord = Coordinates()
drive = Drive()

#kick = Kick()

#RaspberryPi interface setup
gpiostart = 22
gpioreset = 27
gpiocolorset = 17
gpioledblue = 18
gpioledgreen = 23
startButton = Button(gpiostart)
resetButton = Button(gpioreset)
colorsetButton = Button(gpiocolorset)
blueLED = LED(gpioledblue)
greenLED = LED(gpioledgreen)

#Radio setup
node_id = 2
network_id = 10
recipient_id = 1
coord_list = []

#Radio pins
MISO = 21
MOSI = 19
SCLK = 23
CEO = 24

#Field switch condition
fieldRight = True
playerBlue = True

#Pixy origin is upper left of image
#Top left is 0,0
#Top right is 315,0
#Bottom left is 0,207
            
#"middle" for x is 157or158
#We should use range of 10 so 153-163
                    
#playing field resolution is 479 x by 347 y

#Given coordinates
top_left = [500,250]
top_right = [21, 350]
bottom_left = [500, 3]
bottom_right = [21.3]
home = []
goal = [500, 174]
ballDistanceTerminate = 80

automationState = 1

reset = False

def main():
    while 1:
        global anglecurrent
        global fieldRight
        global reset
        go_to = [100, 100]
        SerialThread()
        RadioThread()
        while (not reset):
            if (checkReset(reset)):
                break
            use_list = coord_list.copy()
            if len(use_list) == 6 and anglecurrent != 0: #Waits for thread data to update
                while (not reset):
                    playerBlue, fieldRight = initialCalibration(use_list)
                    if playerBlue:
                        temp = [coord_list[2], coord_list[3]]
                        home = temp.copy()
                    else:
                        temp = [coord_list[0], coord_list[1]]
                        home = temp.copy()
                    
                    if (checkReset(reset)):
                        break
                    
                    print('Initial Angle Calibration')
                    byte_to_send = drive.forward
                    pos1 = coord.Player(coord_list.copy())
                    time.sleep(.25)
                    byte_to_send = drive.stop
                    pos2 = coord.Player(coord_list.copy())
                    anglecurrent = gyro.calibrate(pos1,pos2,drive.forward)
                    adjust_angle()
                    print('Calibration Complete')
                    
                    if (checkReset(reset)):
                        break

                    while (not reset):
                        
                        print('Angle Calibration')
                        byte_to_send = drive.forward
                        pos1 = coord.Player(coord_list.copy())
                        time.sleep(.25)
                        byte_to_send = drive.stop
                        pos2 = coord.Player(coord_list.copy())
                        anglecurrent = gyro.calibrate(pos1,pos2,drive.forward)
                        adjust_angle()
                        print('Calibration Complete')
                        
                        if (checkReset(reset)):
                            break
                        byte_to_send = drive.stop
                        player = coord.Player(coord_list)
                        #print('player')
                        #print(player)
                        ball = coord.Ball(coord_list)
                        if ball[0] < player[0]:
                            goHome(home)
                            byte_to_send = drive.stop
                            if (checkReset(reset)):
                                break
                        elif (player[0] > 480) or (player[0] < 31) or (player[1] > 340) or (player[1] < 13):
                            goHome(home)
                            byte_to_send = drive.stop
                            if (checkReset(reset)):
                                break
                        else:
                            goToBall()
                            byte_to_send = drive.stop
                            if (checkReset(reset)):
                                break
                            chaseBall()
                            byte_to_send = drive.stop
                            if (checkReset(reset)):
                                break
                            goHome(home)
                            byte_to_send = drive.stop
                        if (checkReset(reset)):
                            break


def initialCalibration(home):
    global playerBlue
    global fieldRight
    global reset
    global start
    time.sleep(1)
    reset = False
    print('Calibrating...')
    start = False
    i = 0
    print('Select a color')
    while (start == False):
        if (checkReset(reset)):
            break
        if not startButton.is_pressed:
            start = True
        if not colorsetButton.is_pressed:
            i += 1
            if (i % 2) == 0:
                playerBlue = True
                print('Blue')
                blueLED.on()
                greenLED.off()
                sleep(0.15)
            else:
                playerBlue = False
                print('Green')
                blueLED.off()
                greenLED.on()
                sleep(0.15)
    
    print('Robot Activated')
    #This will change based on which side each color represnts
    #if playerColor: <- put this in radio recieve portion
    if playerBlue:
        if coord_list[2] < 240:
            fieldRight = True
        else:
            fieldRight = False
    else:
        if coord_list[0] < 240:
            fieldRight = True
        else:
            fieldRight = False
        
    ser = serial.Serial('/dev/ttyACM0',115200,timeout=.1)
    ser = serial.Serial('/dev/ttyACM1',115200,timeout=.1)
    serdata = ser.readline().decode('utf-8').rstrip()
    ser.write(bytes((byte_to_send+'\n').encode('utf-8')))
    time.sleep(1)
    
    return playerBlue, fieldRight
            
def centerOnBall(anglecurrent):
    global byte_to_send
    xPixy = 0
    print('Centering On Ball')
    #while (not camera.isCentered(xPixy)): CHANGE THIS
    byte_to_send = '0'
    xPixy = camera.get()
    if camera.isDetected():
        print('ball found')
        if xPixy < 133:
                ##byte_to_send = drive.strafeleft()
            byte_to_send = drive.turnleft
            return byte_to_send
        elif xPixy > 183:
                #byte_to_send = drive.straferight()
            byte_to_send = drive.turnright
            return byte_to_send
        else:
                #byte_to_send = drive.stop()
            #anglecurrent = gyro.read(serialUpdate(drive.forward),anglecurrent)
            return drive.stop
                #time.sleep(1)        
    else:
        print('Nothing Found')
        return drive.stop    
    return drive.stop

def chaseBall():
    global coord_list
    global anglecurrent
    global byte_to_send
    use_list = coord_list.copy()
    player = coord.Player(use_list)
    while (player[0] < 480) and (player[1] < 330) and (player[1] > 23):
        if (checkReset(reset)):
            break
        player = coord.Player(use_list)
        if camera.isDetected():
            if (checkReset(reset)):
                break
            print('Centering')
            byte_to_send = centerOnBall(anglecurrent)
            time.sleep(0.25)
            byte_to_send = drive.stop
            if (checkReset(reset)):
                break
            print('Chasing')
            #anglecurrent = gyro.read(serialUpdate(drive.shiftup),anglecurrent)
            byte_to_send = drive.forward
            time.sleep(0.5)
            byte_to_send = drive.stop
            #kickBall()
            if (checkReset(reset)):
                break
            #automationState = 1 CHANGE THIS OR IT WONT WORK
        else:
            print('Searching for Ball')
            byte_to_send = drive.turnleft
            if (checkReset(reset)):
                break
        
        use_list = coord_list.copy()
        player = coord.Player(use_list)
    

def checkReset(reset):
    global byte_to_send
    if not resetButton.is_pressed or (reset == True):
        byte_to_send = drive.stop
        reset = True
        print('Resetting...')
        return reset

def goToBall():
    global coord_list
    global byte_to_send
    atTarget = False
    while (atTarget == False):
        #print('going to ball')
        if (checkReset(reset)):
            break
        atTarget = path_plan(coord.Ball(coord_list.copy()), coord.Player(coord_list.copy()))
    print('at ball')
        
def goHome(home):
    #print('going home')
    #print('home: ')
    #print(home)
    global byte_to_send
    atTarget = False
    while (atTarget == False):
        #print('going home')
        if (checkReset(reset)):
            break
        atTarget = path_plan(home, coord.Player(coord_list.copy()))
    print('at home')

def path_plan(coord_goto, player): #returns if its at desired location
    #print('moving')
    global byte_to_send
    global anglecurrent
    use_list = player.copy()
    #print('go to: ')
    #print(coord_goto)
    #print('current position: ')
    #print(player)
    if len(use_list) != 2:
        #print('a')
        return False
    if len(use_list) == 2:
        #print(len(use_list))
        #start_pos = [use_list[0], use_list[1]]
        pos1 = player.copy()
        x_dir = int(abs(coord_goto[0]) - abs(use_list[0]))
        y_dir = int(abs(coord_goto[1]) - abs(use_list[1]))
        if (abs(x_dir) < ballDistanceTerminate) and (abs(y_dir) < ballDistanceTerminate):
            byte_to_send = '0'
            print('at target')
            return True
        else:
            if abs(x_dir) > abs(y_dir):
                if x_dir > 0:
                    byte_to_send = drive.forward
                    #print('going forward')
                else:
                    byte_to_send = drive.backward
                    #print('going backward')
            else:
                if fieldRight:
                    if y_dir < 0:
                        byte_to_send = drive.strafeleft
                        #print('going left')
                    else:
                        byte_to_send = drive.straferight
                        #print('going left')
                else:
                    if y_dir > 0:
                        byte_to_send = drive.strafeleft
                        #print('going left')
                    else:
                        byte_to_send = drive.straferight
                        #print('going left')
                    
            time.sleep(0.1) #vector magnitude
            
            pos2 = coord.Player(coord_list)
            
            anglecurrent = gyro.calibrate(pos1,pos2,byte_to_send)
            
            if abs(anglecurrent) > 5:
                adjust_angle()
            
            #print("path: planned")
            return False
    else:
        print('wtf')
        return False
        
def adjust_angle():
    print('adjusting angle')
    global anglecurrent
    global byte_to_send
    angle = anglecurrent
    if (abs(angle) > 5):
        print(abs(angle))
        while (abs(angle) > 5):
            if (checkReset(reset)):
                break
            if angle < 0:
                byte_to_send = drive.turnleft
                time.sleep(0.01)
                byte_to_send = drive.stop
                time.sleep(0.01)
                angle = anglecurrent
                #print('turning left')
            else:
                byte_to_send = drive.turnright
                time.sleep(0.01)
                byte_to_send = drive.stop
                time.sleep(0.01)
                angle = anglecurrent
                #print('turning right')
            
if __name__=='__main__':
    main()
    
    #gyroscope debug
    '''
    byte_to_send = drive.stop
    SerialThread()
    while not reset:
        checkReset(reset)
        time.sleep(1)
        adjust_angle()
    '''   
