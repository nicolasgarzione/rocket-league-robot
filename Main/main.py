''' Import custom defined libraries '''
from myimports import *

''' Ensures GPIO pins are set to correct mode '''
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

''' Initializes Pixy2 camera'''
pixy.init ()
pixy.change_prog ("color_connected_components")

''' Defines Pixy2 block class for reciving data from camera '''
class Blocks (Structure):
  _fields_ = [ ("m_signature", c_uint),
    ("m_x", c_uint),
    ("m_y", c_uint),
    ("m_width", c_uint),
    ("m_height", c_uint),
    ("m_angle", c_uint),
    ("m_index", c_uint),
    ("m_age", c_uint) ]        

'''Serial thread to read data from Arduino Mega #1'''          
class SerialThread(Thread):
    def __init__(self):
        ''' Allows thread to run endlessly'''
        Thread.__init__(self)
        self.daemon = True
        self.start()

    def run(self):
        ''' Constantly updates current angle of gyroscope '''
        global anglecurrent

        ''' Initialization of serial connection '''
        ser = serial.Serial('/dev/ttyACM0',115200,timeout=.1)
        time.sleep(1)

        while True:
            ''' Prevents errors in reading serial data '''
            try:
                ''' Reading gyroscope data '''
                serdata = ser.readline().decode('utf-8').rstrip()
            except:
                serdata = ''

            ''' Writing motor command data'''
            ser.write(bytes((byte_to_send+'\n').encode('utf-8')))
            
            ser.flush()
            
            ''' Calculates angle from gyroscope data and updates relevant variable '''
            if serdata != '':
                zgyrorad = float(serdata)/16384
                zgyrodeg = 57.2963 * (zgyrorad + .01125)
                anglepast = zgyrodeg
                anglecurrent = anglecurrent + anglepast

'''Serial thread to read data from Arduino Mega #2'''
class RadioThread(Thread):
    def __init__(self):
        ''' Allows thread to run endlessly'''
        Thread.__init__(self)
        self.daemon = True
        self.start()

    def run(self):
        ''' Constantly updates current position of balls on playing field '''
        global coord_list

        temp_list = []
        i = 0

        ''' Initialization of serial connection '''
        ser = serial.Serial('/dev/ttyACM1',115200,timeout=.1)
        time.sleep(1)

        ''' Starts clock to synce with radio data recieving rate'''
        start_time = time.time()

        while True:
            ''' Reading coordinate data '''
            serdata = ser.readline().decode('utf-8').rstrip()
            if (time.time()-start_time > 0.5):
                ''' Looks for serial delimiter to start storing data '''
                if serdata == 'a':
                    i = 0

                    ''' Ensures only a full set of data is stored in global variable '''
                    if len(temp_list) == 6:
                        ''' Flips x axis depending on initial side of the field robot is on'''
                        if not fieldRight: 
                            temp_list[0] = -temp_list[0] + 521
                            temp_list[2] = -temp_list[2] + 521
                            temp_list[4] = -temp_list[4] + 521
                            
                        coord_list = temp_list.copy()
                        
                    temp_list.clear()
                    time.sleep(0.002)
                else: 
                    ''' Adds coordinate data to buffer '''
                    if serdata is not '' and serdata.isdigit():
                        time.sleep(0.002)
                        temp_list.append(int(serdata))
                        i += 1
                    else:
                        temp_list.append(0)
                        i += 1

''' Class containing camera commands '''          
class Camera:
    ''' Returns whether camera can see ball '''
    def isDetected(self):
        count = pixy.ccc_get_blocks (1, blocks)
        return (count>0)
    
    ''' Obtains x coordinate of ball in camera's view'''
    def get(self):
        count = pixy.ccc_get_blocks (1, blocks)
        xPixy = blocks[0].m_x
        return xPixy     
    
    ''' Returns whether the ball is in the cneter of the camera '''
    def isCentered(self,xPixy):
        return (153<xPixy<163)

''' Class containing gyroscope commands '''
class Gyroscope:
    ''' Function to calibrate gyroscope '''
    def calibrate(self, old, new, direction):
        ''' Direction of travel used to obtain positional change (vector)'''
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

        ''' Prevents dividing by zero '''
        if (old[0] == new[0]):
            return 
        
        ''' Overwrites old angle with correct angle '''
        return (calibrate+math.atan((old[1]-new[1])/(new[0]-old[0]))) 

''' Class containing coordinate commands '''
class Coordinates:
    global coord_list
    global playerBlue
    global fieldRight
    
    ''' Returns coordinates of player '''
    def Player(self, coord_list):
        use_list = coord_list.copy()
        if playerBlue:
            return [use_list[2], use_list[3]]
        else:
            return [use_list[0], use_list[1]]

    ''' Returns coordinates of ball '''  
    def Ball(self, coord_list):
        use_list = coord_list.copy()
        return [use_list[4], use_list[5]]

''' Class containing drive commands '''  
class Drive:
    ''' Commmands corresponding to Arduino code '''
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

''' Pixy setup '''
pixy.init ()
pixy.change_prog ("color_connected_components");
blocks = BlockArray(100)
frame = 0

''' Serial setup '''

''' Arduino #1 '''
ser = serial.Serial('/dev/ttyACM0',115200,timeout=.1)
ser.flush()
time.sleep(1)
byte_to_send = '0'

''' Arduino #2 '''
ser = serial.Serial('/dev/ttyACM1',115200,timeout=.1)
ser.flush()
time.sleep(1)

''' Gyroscope setup '''
anglecurrent = 0

''' Global class initialization '''

camera = Camera()
gyro = Gyroscope()
coord = Coordinates()
drive = Drive()

''' RPi button/LED interface setup '''
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

''' Radio setup '''
#node_id = 2
#network_id = 10
#recipient_id = 1
#coord_list = []

#Radio pins
#MISO = 21
#MOSI = 19
#SCLK = 23
#CEO = 24

''' Field switch conditions '''
fieldRight = True
playerBlue = True

''' 
Pixy origin is upper left of image
Top left is 0,0
Top right is 315,0
Bottom left is 0,207 
"Middle" for x is 157 or 158
'''
    
''' 
Given playing field coordinates 
NW Corner: 499, 377
NE Corner: 21, 358
SE Corner: 16, 3
SW Corner: 517, 3
'''

''' Adjusted coordinates '''
top_left = [500,250]
top_right = [21, 350]
bottom_left = [500, 3]
bottom_right = [21.3]
home = []
goal = [500, 174]
ballDistanceTerminate = 20

reset = False

def main():
    while 1:
        global anglecurrent
        global fieldRight
        global reset
        global byte_to_send

        ''' Thread initialization '''
        SerialThread()
        RadioThread()

        while (not reset):

            ''' This code is included every time reset button may be needed '''
            if (checkReset(reset)): 
                break

            use_list = coord_list.copy()
            
            ''' Waits until thread data is actively being sent '''
            if len(use_list) == 6 and anglecurrent != 0: 

                while (not reset):

                    ''' Calibrates robot using initial position '''
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

                        ''' Current positions of player and ball'''
                        player = coord.Player(coord_list)
                        ball = coord.Ball(coord_list)

                        if ball[0] < player[0]: 
                            ''' If player is in front of ball (unable to score goal) return home'''
                            goHome(home)
                            byte_to_send = drive.stop
                            if (checkReset(reset)):
                                break

                        elif (player[0] > 480) or (player[0] < 31) or (player[1] > 340) or (player[1] < 13):
                            ''' If player is out of bounds return home'''
                            goHome(home)
                            byte_to_send = drive.stop
                            if (checkReset(reset)):
                                break

                        else:
                            ''' Go to general position of ball '''
                            goToBall()
                            byte_to_send = drive.stop
                            if (checkReset(reset)):
                                break

                            ''' Utilize camera to chase ball '''
                            chaseBall()
                            byte_to_send = drive.stop
                            if (checkReset(reset)):
                                break

                            ''' Go home '''
                            goHome(home)
                            byte_to_send = drive.stop

                        if (checkReset(reset)):
                            break

''' Initial calibration run once at beginning of robot cycle to set relevant variables'''
def initialCalibration(home):
    global playerBlue
    global fieldRight
    global reset
    global start

    time.sleep(1)

    reset = False
    start = False

    i = 0

    print('Calibrating...')

    print('Select a color')
    while (start == False):

        if (checkReset(reset)):
            break
        if not startButton.is_pressed:
            ''' Holds robot in stasis until activated with start button '''
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
    
    ''' Changes which coordinate to read for player position '''
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
    
    ''' Refreshes serial connections '''
    ser = serial.Serial('/dev/ttyACM0',115200,timeout=.1)
    ser = serial.Serial('/dev/ttyACM1',115200,timeout=.1)
    ser.write(bytes((byte_to_send+'\n').encode('utf-8')))
    time.sleep(1)
    
    return playerBlue, fieldRight

''' Checks if reset button has been pressed, returns robot to stasis if so'''
def checkReset(reset):
    global byte_to_send
    if not resetButton.is_pressed or (reset == True):
        byte_to_send = drive.stop
        reset = True
        print('Resetting...')
        return reset

''' Plans path to ball '''
def goToBall():
    global coord_list
    global byte_to_send

    atTarget = False

    print('going to ball')

    while (atTarget == False):

        if (checkReset(reset)):
            break

        atTarget = path_plan(coord.Ball(coord_list.copy()), coord.Player(coord_list.copy()))

    print('at ball')

''' Plans path to home '''      
def goHome(home):
    global byte_to_send

    atTarget = False

    print('going home')

    while (atTarget == False):

        if (checkReset(reset)):
            break

        atTarget = path_plan(home, coord.Player(coord_list.copy()))

    print('at home')

''' Custom path planning algorithim '''
def path_plan(coord_goto, player): 
    global byte_to_send
    global anglecurrent

    use_list = player.copy()
    
    if len(use_list) != 2:
        return False

    if len(use_list) == 2:
        
        pos1 = player.copy()

        ''' Magnitude of difference in position '''
        x_dir = int(abs(coord_goto[0]) - abs(use_list[0]))
        y_dir = int(abs(coord_goto[1]) - abs(use_list[1]))

        ''' If at desired location, terminate path planning'''
        if (abs(x_dir) < ballDistanceTerminate) and (abs(y_dir) < ballDistanceTerminate):
            byte_to_send = '0'
            print('at target')
            return True
        else:

            if abs(x_dir) > abs(y_dir):

                if x_dir > 0:
                    byte_to_send = drive.forward
                else:
                    byte_to_send = drive.backward
            else:

                if fieldRight:

                    if y_dir < 0:
                        byte_to_send = drive.strafeleft
                    else:
                        byte_to_send = drive.straferight
                else:

                    if y_dir > 0:
                        byte_to_send = drive.strafeleft
                    else:
                        byte_to_send = drive.straferight
                    
            time.sleep(0.1) #vector magnitude
            
            pos2 = coord.Player(coord_list)
            
            ''' Calibrates gyroscope '''
            anglecurrent = gyro.calibrate(pos1,pos2,byte_to_send)
            
            ''' If robot drifted in angle, correct it '''
            if abs(anglecurrent) > 5:
                adjust_angle()
            
            return False
    else:
        print('wtf')
        return False

''' Corrects robot angle so it always points in one direction '''     
def adjust_angle():
    global anglecurrent
    global byte_to_send

    angle = anglecurrent

    print('adjusting angle')

    if (abs(angle) > 5):

        while (abs(angle) > 5):

            if (checkReset(reset)):
                break

            if angle < 0:
                byte_to_send = drive.turnleft
                time.sleep(0.01)
                byte_to_send = drive.stop
                time.sleep(0.01)
                angle = anglecurrent
            else:
                byte_to_send = drive.turnright
                time.sleep(0.01)
                byte_to_send = drive.stop
                time.sleep(0.01)
                angle = anglecurrent

''' Chases ball using camera '''
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

            byte_to_send = drive.forward
            time.sleep(0.5)
            byte_to_send = drive.stop

            if (checkReset(reset)):
                break

        else:
            print('Searching for Ball')

            byte_to_send = drive.turnleft

            if (checkReset(reset)):
                break
        
        use_list = coord_list.copy()

        player = coord.Player(use_list)

def centerOnBall(anglecurrent):
    global byte_to_send

    xPixy = 0
    byte_to_send = '0'

    print('Centering On Ball')

    xPixy = camera.get()

    if camera.isDetected():

        print('ball found')

        if xPixy < 133:

            byte_to_send = drive.turnleft
            return byte_to_send
        elif xPixy > 183:

            byte_to_send = drive.turnright
            return byte_to_send
        else:

            return drive.stop        
    else:

        print('Nothing Found')

        return drive.stop    
              
if __name__=='__main__':
    main() 
