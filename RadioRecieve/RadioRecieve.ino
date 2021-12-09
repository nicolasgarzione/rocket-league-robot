/* REV 1 */

/* RadioRecieve **********************************************************
 * Code to allow Arduino Mega to parse RFM69 radio data and send it to RPi
 *************************************************************************/

/* Include the RFM69 and SPI libraries */
#include <SPI.h>
#include <RFM69.h>

/* Addresses specific to the soccer arena */
#define NETWORKID     10  // Must be the same for all nodes
#define MYNODEID      2   // My node ID
#define TONODEID      1   // Origin node ID

/* RFM69 frequency module used */
#define FREQUENCY     RF69_915MHZ

/* AES encryption */
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

/* ACKnowledge when sending messages */
#define USEACK        false // Request ACKs or not

/* Library object from RFM69 module */
RFM69 radio;

#define PACKSIZE 25 // Maximum Packetsize is 25,if 4 digit for x and y, and static characters
#define GrnID 1 // #defined constants for clearer code
#define BluID 2
#define YelID 3

/* Global variables to store radio data */
int gx, gy, bx, by, yx, yy; 

char buildInt[4];

void setup()
{
  /* Opens a serial channel to communicate with RPi */
  Serial.begin(115200);  

  /* Clears array used to build integers */
  sprintf(buildInt, "    "); 

  /* Using RFM69 library, the radio is initialized */
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); 

  /* Optional encryption */
  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
}

void loop()
{
 /**************
  * SENDING DATA
  **************/

  int to_print[6] = {gx, gy, bx, by, yx, yy};

  /* Unique Byte to send before all others to sync serial data on the RPI side of the code */
  Serial.println('a');
  
  for(int i = 0; i < 6; i++)
  {
    /* Bytes are printed serially and recieved by the RPi */
    Serial.println(to_print[i]);
  }
  delay(5);

 /****************
  * RECEIVING DATA
  ****************/
  
  int j=0;
  char data[PACKSIZE];

  /* Clear data array to make room for new data */
  for (j=0; j<PACKSIZE; j++) data[j]=0; 

  /* Data is found */
  if (radio.receiveDone()) { 

    /* The actual message is contained in the DATA array, and is DATALEN bytes in size: */
    for (byte i = 0; i < radio.DATALEN; i++){
      data[i]=(char)radio.DATA[i];
      j=i;
      }
    
   /**************
    * PARSING DATA
    **************/

    j=0;
    while (j<PACKSIZE) {  
      if(data[j]=='G') {  
        /* For green */
        pullCoords(data,j,GrnID); // Green is always first in the packet
        break;                    
      } else if (data[j]=='B') { 
        /* For blue */
        pullCoords(data,j,BluID); 
        j++;                      
      } else if (data[j]=='Y') { 
        /* For Yellow */
        pullCoords(data,j,YelID); 
        break;                    // Yellow is the last in the packet      
      } else {
        j++;                      
      }
    }
  }
}

/* pullCords(...) *****************************************************************
 *     This function is called for each color to parse the data contained within its
 *     contents. This is necessary to be able to send useful data to the RPi.
 **********************************************************************************/

void pullCoords(char packet[], int letterLoc, int color) {
    /* Array locations of bracket start character, comma and bracket end character */
    int BrS,Comma,BrE = 0;
    int i=0;

    /* The start bracket will be immediately after the color letter indicator */
    BrS=letterLoc+1; 
    for (i = BrS+1; i < PACKSIZE; i++) {
      /* Finds the location of the comma that divides x and y coordinates */
      if(packet[i]==',') Comma=i;  
      if(packet[i]=='>') {
        /* Find the location of the end bracket */
        BrE=i;
        break;  
      }
    }

    /* Clear the buildInt array  */
    sprintf(buildInt, "    ");  

    /* Works through the chars that represent the x coordinate and copy them into the buildInt array */
    for (i = 0; i < (Comma-(BrS+1)); i++) {  
      buildInt[i]=packet[(BrS+1+i)];         
    }

    /* Terminates the buildInt array to make it easy on the converter */
    buildInt[(Comma-(BrS+1))]='\0';  

    /* Convert the buildInt array to an integer and store in the correspoding color x coordinate */
    switch (color) {                 
      case GrnID: 
        /* Green x coord */
        gx=atoi(buildInt); 
        break;
      case BluID: 
        /* Blue x coord */
        bx=atoi(buildInt); 
        break;
      case YelID: 
        /* Yellow x coord */
        yx=atoi(buildInt); 
        break;
    }

    /* Clear the buildInt array  */
    sprintf(buildInt, "    ");  

    /* Works through the chars that represent the y coordinate and copy them into the buildInt array */
    for (i = 0; i < (BrE-(Comma+1)); i++) {  
      buildInt[i]=packet[(Comma+1+i)];         
    }

    /* Terminate the buildInt array to make it easy on the converter */
    buildInt[(BrE-(Comma+1))]='\0';  
    
    /*  Convert the buildInt array to an integer and store in the correspoding color y coordinate */
    switch (color) {                 
      case GrnID: 
        /* Green y coord */
        gy=atoi(buildInt); 
        break;
      case BluID: 
        /* Blue y coord */
        by=atoi(buildInt); 
        break;
      case YelID: 
        /* Yellow y coord */
        yy=atoi(buildInt); 
        break;
    }
}
