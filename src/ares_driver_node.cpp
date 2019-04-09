/* knex_driver_node:
 * This ROS node drives the k'nex test rover, by taking in joy commands
 * and converting to motor speed/direction outputs at the serial port.
 * An Arduino Uno recieves the serial commands, and drives the motors.
 * 
 * ROS inuput messages:
 * 	/joy (sensor_msgs/Joy.h)
 * ROS output messages:
 * 	none
 * Other inputs:
 * 	none
 * Other outputs:
 * 	serial data to Arduino Uno
 * 	Protocol: UART
 * 	Packet organization: PID|DATA|CRC -> "GO"|direction(b'000000rl'),motorL,motorR|
 * 		direction bits are low for forward, and high for backward
 * 		motorL and motorR are uint8_t data bytes, where 0=stop and 255=full speed
 *
 * Helpful Links:
 * 	https://answers.ros.org/question/12181/how-to-use-sensor_msgsjoy-instead-of-joystick_driversjoy/
 *
 * Author Derek Workman / Seth Horne
 * Email: derek.workman@aggiemail.usu.edu
 */


/*****Include Files*******/
#include <ros/ros.h>		//Always include for a ROS node
#include <sensor_msgs/Joy.h>	//Include for control messages from game controller
#include <serial/serial.h>	//Include for serial output to Arduino motor driver
#include <math.h>		//For abs() and possibly other mathimatical opperations required
#include <string.h>		//for passing the serial port name
#include <iostream>		//for temporary debug

/******Definitions********/
//indecies for serial packet
#define _G	0	//Packet ID 'G'
#define _O	1	//Packet ID 'O'
#define _MODE	2
#define _SIZE	3
#define _CTRL   4
#define _AUTO   5
#define L0      6
#define L1      7
#define L2      8
#define	R0      9
#define	R1      10
#define	R2      11
//#define DIR     12
#define CRC     12


//#define DIR	3	//Direction Byte b'000000lr'
//#define MOTOR_L	4	//Left motor speed 0 to 255
//#define MOTOR_R 5	//Right motor speed 0 to 255
//#define CRC	6	//Cyclic Redundancy Check, contains the sum of all data bits.
			//	this is used for error checking

/******MODES**************/
#define M_DRIVE	0	//PID value for drive mode

//Constant Sizes
#define N	CRC+1	//Total number of bytes in serial packet
#define N_PID	_MODE+1	//Number of bytes that make up the PID
#define N_CRC	1	//Number of CRC bytes
#define N_DATA  N-(_SIZE+1)-N_CRC	//Number of data bytes
#define N_WHEELS_ONE_SIDE 3

/*Bit masking
#define DIR_R0_BIT	0x01	//Bit address for left wheel direction
#define DIR_R1_BIT	0x02
#define DIR_R2_BIT	0x04
#define DIR_L0_BIT	0x08	//Bit address for right wheel direction
#define DIR_L1_BIT	0x10
#define	DIR_L2_BIT	0x20

#define DIR_R_BITS	0		//indecies
#define DIR_L_BITS	N_WHEELS_ONE_SIDE

/*Bit mask all*/
//#define DIR_R		0x07
//#define DIR_L		0x38
//#define DIR_FORWARD	1
//#define DIR_REVERSE 0

//Strings
#define SERIAL_PORT_NAME	"/dev/uart2ppm"	//Consider making this a ROS parameter so that it can be altered

//Indecies for joy message
#define LEFT_STICK	1	//index for Joy message array
#define RIGHT_STICK	4	//indec for Joy message array

//Safety Timeout
#define TIMEOUT		1.000	//Motor timeout after x seconds

/****Global Variables*****/
uint8_t packet[N]	= {0};	//contains the packet to send via serial
serial::Serial ser;	//The serial object
double lastTime = 0;	//for checking joy timeout

/****Global Const********* Not used
const uint8_t bitMask[2*N_WHEELS_ONE_SIDE] = {
	DIR_L0_BIT,
	DIR_L1_BIT,
	DIR_L2_BIT,
	DIR_R0_BIT,
	DIR_R1_BIT,
	DIR_R2_BIT
	};

/*Lookup Table*/
const uint8_t LOOKUP_TABLE[256] = 
{
     0

,
     2

,
     5

,
     7

,
     9

,
    11

,
    14

,
    16

,
    18

,
    20

,
    22

,
    24

,
    26

,
    28

,
    30

,
    32

,
    34

,
    35

,
    37

,
    39

,
    41

,
    42

,
    44

,
    46

,
    47

,
    49

,
    50

,
    52

,
    54

,
    55

,
    56

,
    58

,
    59

,
    61

,
    62

,
    63

,
    65

,
    66

,
    67

,
    69

,
    70

,
    71

,
    72

,
    74

,
    75

,
    76

,
    77

,
    78

,
    79

,
    80

,
    81

,
    82

,
    83

,
    84

,
    85

,
    86

,
    87

,
    88

,
    89

,
    90

,
    91

,
    92

,
    93

,
    94

,
    94

,
    95

,
    96

,
    97

,
    98

,
    98

,
    99

,
   100

,
   101

,
   101

,
   102

,
   103

,
   103

,
   104

,
   105

,
   105

,
   106

,
   107

,
   107

,
   108

,
   109

,
   109

,
   110

,
   110

,
   111

,
   111

,
   112

,
   113

,
   113

,
   114

,
   114

,
   115

,
   115

,
   116

,
   116

,
   117

,
   117

,
   117

,
   118

,
   118

,
   119

,
   119

,
   120

,
   120

,
   120

,
   121

,
   121

,
   122

,
   122

,
   122

,
   123

,
   123

,
   123

,
   124

,
   124

,
   124

,
   125

,
   125

,
   125

,
   126

,
   126

,
   126

,
   127

,
   127

,
   127

,
   127

,
   128

,
   128

,
   128

,
   129

,
   129

,
   129

,
   130

,
   130

,
   130

,
   131

,
   131

,
   131

,
   132

,
   132

,
   132

,
   133

,
   133

,
   134

,
   134

,
   134

,
   135

,
   135

,
   136

,
   136

,
   137

,
   137

,
   137

,
   138

,
   138

,
   139

,
   139

,
   140

,
   140

,
   141

,
   141

,
   142

,
   143

,
   143

,
   144

,
   144

,
   145

,
   145

,
   146

,
   147

,
   147

,
   148

,
   149

,
   149

,
   150

,
   151

,
   151

,
   152

,
   153

,
   153

,
   154

,
   155

,
   156

,
   156

,
   157

,
   158

,
   159

,
   160

,
   160

,
   161

,
   162

,
   163

,
   164

,
   165

,
   166

,
   167

,
   168

,
   169

,
   170

,
   171

,
   172

,
   173

,
   174

,
   175

,
   176

,
   177

,
   178

,
   179

,
   180

,
   182

,
   183

,
   184

,
   185

,
   187

,
   188

,
   189

,
   191

,
   192

,
   193

,
   195

,
   196

,
   198

,
   199

,
   200

,
   202

,
   204

,
   205

,
   207

,
   208

,
   210

,
   212

,
   213

,
   215

,
   217

,
   219

,
   220

,
   222

,
   224

,
   226

,
   228

,
   230

,
   232

,
   234

,
   236

,
   238

,
   240

,
   243

,
   245

,
   247

,
   249

,
   252

,
   255
};

/******Main Program*******/

//Function to initialize the serial port
bool InitSerial(uint32_t baud, std::string portName) {
    serial::Timeout timeout = serial::Timeout(0, 0, 0, 0, 0);
    serial::bytesize_t bytesize = serial::eightbits;
    serial::parity_t parity = serial::parity_none;
    serial::stopbits_t stopbits = serial::stopbits_one;
    serial::flowcontrol_t flowcontrol = serial::flowcontrol_none;

    try{
        ser.setPort(portName);
        ser.setBaudrate(baud);
        ser.setTimeout(timeout);
        ser.setBytesize(bytesize);
        ser.setParity(parity);
        ser.setStopbits(stopbits);
        ser.setFlowcontrol(flowcontrol);
        ser.open();
    }
    catch (serial::SerialException e) {
        ROS_FATAL("Failed to connect to the Motor Controller, %s.", e.what());
        ros::shutdown();	//Halt the ROS system when critical components are not functioning
        return false;
    }
    return true;
}

//add data array for CRC
uint8_t GetCRC(uint8_t* buf, size_t n) {

  uint8_t bitCount = 0;
  
  for(uint8_t i = 0; i < n; i++) {
    bitCount += buf[i]&0x01;  //count the first bit if set
    for(uint8_t j = 1; j < 8; j++) {
      bitCount += (buf[i] >> j)&0x01; //count the rest of the set bits
    }
  }
  return bitCount;
}

//ROS callback for joy topic
//	Motor drive data is updated here
void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    lastTime = ros::Time::now().toSec();	//update last time variable

//    packet[DIR] = 0;    //clear packet before starting, because it makes me feel better
//	if(msg->axes[LEFT_STICK] >= 0) packet[DIR] = DIR_R;	//Set Left Direction bits if moving forward
//	if(msg->axes[RIGHT_STICK] >= 0) packet[DIR] = packet[DIR]|DIR_L;
	for(uint8_t i = 0; i < N_WHEELS_ONE_SIDE; i++) {
	//	packet[L0+i] = ((float)127.5*(0.33*msg->axes[LEFT_STICK]+1));
	//	packet[R0+i] = ((float)127.5*(0.33*msg->axes[RIGHT_STICK]+1));
	//Using exponential lookup table instead of equasion
		uint8_t index = ((float)127.5*(msg->axes[LEFT_STICK]+1));
		packet[L0+i] = LOOKUP_TABLE[index];
		index = ((float)127.5*(msg->axes[RIGHT_STICK]+1));
		packet[R0+i] = LOOKUP_TABLE[index];
	}

    uint8_t data[N_DATA+1];	//data length will be N subtract "GO" and CRC byte
    for(uint8_t i = 0; i < (N_DATA+1); i++) {
        data[i] = packet[i+N_PID];
    }
    packet[CRC] = GetCRC(data, N_DATA+1);	//add three for N_DATA = 0x09
}

//Main
int main(int argc, char **argv) {

    ros::init(argc,argv,"hab_sim");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joy",1,JoyCallback);

    ros::Rate rate(20);	//repeat transmission at 20Hz

    packet[_G] = 'G';	//initialize packet ID
    packet[_O] = 'O';	//initialize packet ID
    packet[_MODE] = M_DRIVE;
    packet[_SIZE] = N_DATA;
    packet[_CTRL] = 0xFF;  //Hold in Drive mode for testing
    packet[_AUTO] = 0x00;  //Hold in manualmode for testing

    InitSerial(9600, SERIAL_PORT_NAME);	//initialize serial port with 9600 baud rate

    while(ros::ok()) {
        if((ros::Time::now().toSec() - lastTime) > TIMEOUT) {	//if a new packet has not been recieved within the amount of seconds specified by TIMEOUT
            for(uint8_t i = L0; i < N-N_CRC; i++) {                //      then stop the motors
                packet[i] = 127;
            }
	    packet[CRC] = GetCRC(&packet[_SIZE-1], N_DATA+1);
        }
	//show packet being sent
	std::cout << " | " << packet[0] << " | " << packet[1] << " | ";
	for(uint8_t i = 2; i < N; i++) {
	    std::cout << ((unsigned int)packet[i]) << " | ";
        }
        std::cout << '\n';

        ser.write(packet, N);	//Send Packet over serial
        ros::spinOnce();	//Watch for callbacks
        rate.sleep();		//sleep between data transmissions
    }
    ser.close();
    return 0;
}
