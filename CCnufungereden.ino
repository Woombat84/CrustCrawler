/*
   Simple Dynamixel MX sketch for Teensy/Arduino

   In this example we are going to set up a connection to a RS-485 based
   Dynamixel bus on Serial1. The physical serial port Serial1 is on pins 0 and 1
   on the Teensy 3.5. We will also set up a transmit enable pin on pin 2 to
   enable transmitting with the RS-485 transceiver board. The Serial1 port will
   need to be set to the appropriate baud rate of the servos; 57.6 kbaud is
   the factory default, 115.2 kbaud and 1Mbaud is also often used.

   When the Dynamixel port is set up, the program continously pings a servo on
   ID 1 and returns the result on the USB serial port (Serial).

   The ping function that is being used is very similar to other functions
   defined for e.g. setting and getting the position of the servo. To
   experiment, just substitute the ping with the functions definded above.

   A note on the functions. Basically, what the functions do is to call a set of
   low-level functions in order to construct a proper formatted package to be
   sent over the serial port to the Dynamixel using the Dynamixel protocol 2.0
   (http://emanual.robotis.com/docs/en/dxl/protocol2/). Such functions are
   sometimes called convenience functions. We have only implemented convenience
   functions for a small number of often-used commands. If you need to send some
   other command, you can implement the convenience function yourself, using the
   existing convenience functions as templates and
   (http://emanual.robotis.com/docs/en/dxl/mx/mx-64-2) as reference.

   If you are using an Arduino Mega, uncomment the #define ARDUINOMEMUSE line
   below.
*/

/*
   Copyright (c) 2018 Jens Dalsgaard Nielsen
   Copyright (c) 2018 Karl Damkjær Hansen

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#include <Arduino.h>

// Using Arduino Mega? Uncomment this:
// #define ARDUINOMEMUSE

// Buffers for transmitting (tx) and receiving (rx).
// NB: If you transmit or receive packages longer than 30 bytes, then change the
// length definitions below.
#define TX_BUFFER_LEN 30
#define RX_BUFFER_LEN 30
uint8_t tx_buffer[TX_BUFFER_LEN];
uint8_t rx_buffer[RX_BUFFER_LEN];


// Low-level functions for setting individual bytes in the buffers.
void putInt8t(int8_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = value;
}

int8_t getInt8t(uint8_t* buffer, size_t pos)
{
  return buffer[pos];
}

void putUint8t(uint8_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = value;
}

uint8_t getUint8t(uint8_t* buffer, size_t pos)
{
  return buffer[pos];
}

void putInt16t(int16_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = (uint8_t)(value & 0x00ff);
  buffer[pos + 1] = (uint8_t)(value >> 8);
}

void putUint16t(uint16_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = (uint8_t)(value & 0x00ff);
  buffer[pos + 1] = (uint8_t)(value >> 8);
}

int16_t getInt16t(uint8_t* buffer, size_t pos)
{
  int16_t v = 0;
  v = buffer[pos + 1];
  v = v << 8;
  v = v | buffer[pos];
  return v;
}

uint16_t getUint16t(uint8_t* buffer, size_t pos)
{
  uint16_t v = 0;
  v = buffer[pos + 1];
  v = v << 8;
  v = v | buffer[pos];
  return v;
}

void putInt32t(int32_t val, uint8_t* buffer, size_t pos)
{
  for (int16_t i = 0; i < 4 ; i++) {
    buffer[pos + i] = (uint8_t)(val & 0x000000ff);
    val = val >> 8;
  }
}

int32_t getInt32t(uint8_t* buffer, size_t pos)
{
  int32_t v = 0;
  for (int16_t i = 3; i > -1 ; --i) {
    v = v << 8;
    v = v | (int32_t)buffer[pos + i];
  }
  return v;
}


// Dynamixel Protocol 2.0
// The protocol defines a header with fixed positions for the instruction and
// length fields:
#define DXL_LENGTH_POS 5
#define DXL_INSTRUCTION_POS 7

inline size_t getPackageLength(uint8_t* buffer)
{
  return getUint16t(buffer, DXL_LENGTH_POS);
}

// Cyclic Redundancy Check (CRC)
// The protocol uses CRC to check for faults in the transmissions, see:
// https://en.wikipedia.org/wiki/Cyclic_redundancy_check. Specifically
// Dynamixel uses a variant of the CRC16 algorithm used in MODBUS also called
// CRC-16-IBM, see http://emanual.robotis.com/docs/en/dxl/crc/.
#ifdef ARDUINOMEMUSE
const uint16_t  crc_table[] PROGMEM = {
#else
const uint16_t  crc_table[] = {
#endif

  0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
  0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
  0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
  0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
  0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
  0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
  0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
  0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
  0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
  0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
  0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
  0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
  0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
  0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
  0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
  0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
  0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
  0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
  0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
  0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
  0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
  0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
  0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
  0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
  0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
  0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
  0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};


uint16_t calculateCrc(uint16_t crc_accum, uint8_t* data_blk_ptr, size_t data_blk_size)
{
  size_t i, j;

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
#ifdef ARDUINOMEMUSE
    crc_accum = (crc_accum << 8) ^  pgm_read_word_near(crc_table + i);
#else
    crc_accum = (crc_accum << 8) ^ crc_table[i];
#endif
  }
  return crc_accum;
}

// Add CRC to a buffer
// Calculates the CRC and appends it to the buffer.
//
// @param buffer    The buffer to calculate CRC for.
// @param data_size The number of bytes in the buffer to calculate the CRC for.
//                  Incidentally, this is also the position in the array where
//                  the CRC will be added.
void addCrc(uint8_t* buffer, size_t data_size)
{
  uint16_t crc;
  crc = calculateCrc(0, buffer, data_size);
  putInt16t(crc, tx_buffer, data_size);
}

// Check the CRC of a package
// Calculates the CRC of the data in the buffer and compares it to the received
// CRC checksum.
//
// @param buffer the buffer holding the package.
// @param pos the position of the first CRC byte in the buffer.
// @return true if the CRC check is successful.
boolean checkCrc(uint8_t* buffer, int16_t pos)
{
  uint16_t incomming_crc = getUint16t(buffer, pos);
  uint16_t calculated_crc = calculateCrc(0, buffer, pos);
  return (calculated_crc == incomming_crc);
}

void setHdrAndID(uint8_t id)
{
  tx_buffer[0] = 0xff;
  tx_buffer[1] = 0xff;
  tx_buffer[2] = 0xfd;
  tx_buffer[3] = 0x00;
  tx_buffer[4] = id;
}


void dumpPackage(uint8_t *buffer)
{
  size_t l = getPackageLength(buffer) + 7;
  for (size_t i = 0; i < l; i++) {
    //Serial.print((int)buffer[i], HEX); Serial.print(" ");
  }
  //Serial.println("");
}

// Transmit the package in the tx_buffer
void transmitPackage()
{
  size_t pgk_length = getPackageLength(tx_buffer) + 7;
  Serial1.write(tx_buffer, pgk_length);
}

// Try to receive a package
//
// @param timeout milliseconds to wait for a reply.
// @returns true if a package was received and the CRC checks out.
bool receivePackage(size_t timeout = 100)
{
  elapsedMillis since_start = 0;
  size_t bytecount = 0;
  size_t remaining_read = 1;
  while (remaining_read > 0 && since_start < timeout)
  {
    if (Serial1.available())
    {
      uint8_t incomming_byte = Serial1.read();
      switch (bytecount)
      {
        case 0:
        case 1: if (incomming_byte == 0xFF) {
            rx_buffer[bytecount] = incomming_byte;
            ++bytecount;
          } else {
            bytecount = 0;
          }
          break;
        case 2: if (incomming_byte == 0xFD)
          {
            rx_buffer[bytecount] = incomming_byte;
            ++bytecount;
          } else {
            bytecount = 0;
          }
          break;
        case 3:
        case 4:
        case 5: rx_buffer[bytecount] = incomming_byte;
          ++bytecount;
          break;
        case 6: rx_buffer[bytecount] = incomming_byte;
          remaining_read = getPackageLength(rx_buffer);
          ++bytecount;
          break;
        default: rx_buffer[bytecount] = incomming_byte;
          ++bytecount;
          --remaining_read;
          break;
      }
    }
  }
  if (remaining_read == 0)
  {
    //dumpPackage(rx_buffer, getPackageLength(rx_buffer));
    return checkCrc(rx_buffer, bytecount - 2);
  }
  else
  {
    return false;
  }
}

// Send a READ instruction to the servo
void sendReadInstruction(uint8_t id, uint16_t from_addr, uint16_t data_length)
{
  /* Read instruction package
    0     1     2     3     4    5      6     7     8      9       10      11      12    13
    H1    H2    H3    RSRV  ID   LEN1   LEN2  INST  PARAM1 PARAM2  PARAM3  PARAM4  CRC1  CRC2
    0xFF  0xFF  0xFD  0x00  0x01  0x07  0x00  0x02  0x84   0x00    0x04    0x00    0x1D  0x15
  */
  setHdrAndID(id);
  putUint16t(7, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x02, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(from_addr, tx_buffer, 8);
  putUint16t(data_length, tx_buffer, 10);
  addCrc(tx_buffer, 12);

  transmitPackage();
}

// Send a 1 byte READ instruction to the servo
void sendWriteInstruction(uint8_t id, uint16_t address, uint8_t data)
{
  setHdrAndID(id);
  putUint16t(6, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x03, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(address, tx_buffer, 8);
  putUint8t(data, tx_buffer, 10);
  addCrc(tx_buffer, 11);

  transmitPackage();
}

// Send a 4 byte READ instruction to the servo
void sendWriteInstruction(uint8_t id, uint16_t address, int32_t data)
{
  /* Read instruction package
    0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15
    H1   H2   H3   RSRV ID   LEN1 LEN2 INST P1   P2   P3   P4   P5   P6   CRC1 CRC2
    0xFF 0xFF 0xFD 0x00 0x01 0x09 0x00 0x03 0x74 0x00 0x00 0x02 0x00 0x00 0xCA 0x89
  */
  setHdrAndID(id);
  putUint16t(9, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x03, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(address, tx_buffer, 8);
  putInt32t(data, tx_buffer, 10);
  addCrc(tx_buffer, 14);

  transmitPackage();
}


// Ping a servo
// Pings the servo and waits for a response. If none is received within the
// timeout, the ping fails. There is no check as to whether the response is
// appropriate. When sending a ping to broadcast (0xFE), several packages may be
// received. The user should look into the received package to see if the result
// is satisfactory.
//
// @param  id      the servo ID to ping.
// @param  timeout milliseconds before timing out.
// @return true if response is received.
bool doPing(uint8_t id, size_t timeout = 100)
{
  /* Ping Instruction Packet
    0     1     2     3     4     5     6     7     8     9
    H1    H2    H3    RSRV  ID    LEN1  LEN2  INST  CRC1  CRC2
    0xFF  0xFF  0xFD  0x00  0x01  0x03  0x00  0x01  0x19  0x4E
  */
  setHdrAndID(id);
  putUint16t(3, tx_buffer, DXL_LENGTH_POS); // length is at pos 5 and 6
  putUint8t(0x01, tx_buffer, DXL_INSTRUCTION_POS); // ping instruction (0x01) at pos 7
  addCrc(tx_buffer, 8); // CRC at pos 8 and 9
  transmitPackage();

  bool package_received = receivePackage(timeout);

  return package_received;
}
// Set the goal position of a servo
// Reads the position of a servo, if no response is read, the function does not
// modify the value of the position variable.
//
// @param id the servo ID to get the position of.
// @param position a reference to store the read position in.
// @return true if the servo responded.
bool readMovingSpeed(uint8_t id, int32_t& MovingSpeed)
{
  sendReadInstruction(id, 128, 4);
  if (receivePackage(100))
  {
    MovingSpeed = getInt32t(rx_buffer, 9);
    return true;
  }
  else
  {
    return false;
  }
}
// Set the goal position of a servo
// Reads the position of a servo, if no response is read, the function does not
// modify the value of the position variable.
//
// @param id the servo ID to get the position of.
// @param position a reference to store the read position in.
// @return true if the servo responded.
bool readPosition(uint8_t id, int32_t& position)
{
  sendReadInstruction(id, 132, 4);
  if (receivePackage(100))
  {
    position = getInt32t(rx_buffer, 9);
    return true;
  }
  else
  {
    return false;
  }
}
// Send the Velocity of a servo

bool SendVel(uint8_t id, int32_t& speed)
{
  sendWriteInstruction(id, 100, speed);
  if (receivePackage(100))
  {
    return true;
  }
  else
  {
    return false;
  }
}
// @param id       the servo ID to set the position of.
// @param position the velocity to go to.
// @return true if the servo responded.
bool Pgain(uint8_t id, int32_t& p_gain)
{
  sendWriteInstruction(id, 24, p_gain);
  if (receivePackage(100))
  {
    return true;
  }
  else
  {
    return false;
  }
}
// Read the position of a servo
bool setGoalPosition(uint8_t id, int32_t position)
{
  sendWriteInstruction(id, 116, position);
  if (receivePackage(100))
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Enable torque on the servo
// The torque must be enabled for the motor to move.

bool torqueEnable(uint8_t id, bool enable)
{
  uint8_t enable_data = 0x01;
  if (! enable)
  {
    enable_data = 0x00;
  }
  sendWriteInstruction(id, 64, enable_data);
  if (receivePackage(100))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void WriteData(int& id, int& thetaGoTo, int32_t& speedToHave) {
  if (SendVel(id, speedToHave))
  {

    dumpPackage(tx_buffer);

    dumpPackage(rx_buffer);
  }

  if (setGoalPosition(id, thetaGoTo))
  {
    dumpPackage(tx_buffer);
    dumpPackage(rx_buffer);
  }
}
int ReadData(int& id, int32_t& thetaNow , int32_t& turnSpeedNow) {
  if (readMovingSpeed(id, turnSpeedNow)) {
    dumpPackage(tx_buffer);
    dumpPackage(rx_buffer);

  }
  //Serial.println();
  if (readPosition(id, thetaNow))
  {
    //Serial.print("Read package sent: ");
    dumpPackage(tx_buffer);
    //Serial.print("Servo position: "); Serial.println(pos,DEC);
    //Serial.print("Package received: ");
    dumpPackage(rx_buffer);
  }
};
void torqueEnabler(int& id) {
  if (torqueEnable(id, true))
  {
    //Serial.print("Torque Enable package sent: ");
    dumpPackage(tx_buffer);
    //Serial.print("Package received: ");
    dumpPackage(rx_buffer);
  }
}
// defining a struct for a joint.
struct servodata {
  int servo_id;
  int theta_go_to;
  int32_t theta_now;
  int32_t speed_to_have;
  int32_t turn_speed_now;
};
//defining a struct obj
typedef struct servodata ServoData;
// making an array from the struct obj with the size off 6, but 0 is not in use
ServoData Joint[5];
//prototyping each function 
void ReadUpdateCCData();
void WriteUpdateCCdata();
void MotorSwitch(int &MotorNR, int &ZZ, bool &timeCehk, long &Time);
void AngleSelect(int32_t &ArrNow, int &MotorNR, int &ArrGoal, int EMG_1, int EMG_2, int32_t &PWM, bool &on);
void printingPlot();
void emgCom(int &sumx, int &sumy, int &sumz, int &sumemg1, int &sumemg2 );
//intzialing the variabels
int Xacc = 0; int Yacc = 0; int Zacc = 0; int EMG1 = 0; int EMG2 = 0;
int GB[5] = {Xacc, Yacc, Zacc, EMG1, EMG2};
int motorNr = 1; int FingerNR = 5;

bool timeChk = false; bool StopS = false;

long Tid = 0;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);
  Serial1.transmitterEnable(2);
  Joint[1].servo_id = 1;
  Joint[2].servo_id = 2;
  Joint[3].servo_id = 3;
  Joint[4].servo_id = 4;
  Joint[5].servo_id = 5;
  Joint[1].speed_to_have = 262;
  Joint[2].speed_to_have = 262;
  Joint[3].speed_to_have = 262;
  Joint[4].speed_to_have = 262;
  Joint[5].speed_to_have = 262;
  Joint[1].theta_go_to = 2000;
  Joint[2].theta_go_to = 2000;
  Joint[3].theta_go_to = 2000;
  Joint[4].theta_go_to = 2000;
  Joint[5].theta_go_to = 2000;
  torqueEnabler(Joint[1].servo_id);
  torqueEnabler(Joint[2].servo_id);
  torqueEnabler(Joint[3].servo_id);
  torqueEnabler(Joint[4].servo_id);
  torqueEnabler(Joint[5].servo_id);
  WriteUpdateCCdata();
  ReadUpdateCCData();

}

void loop() {
  // Calling the greybox and stores the data from it.
  emgCom(GB[1], GB[2], GB[3], GB[4], GB[5]);
  //Selecting a motor ubon the Z accleration
  MotorSwitch(motorNr, GB[3], timeChk, Tid);
  if (motorNr == 4) 
  {
    ReadData(Joint[motorNr].servo_id, Joint[motorNr].theta_now , Joint[motorNr].turn_speed_now);
    ReadData(Joint[FingerNR].servo_id, Joint[FingerNR].theta_now , Joint[FingerNR].turn_speed_now);
  }
  else 
  {
    ReadData(Joint[motorNr].servo_id, Joint[motorNr].theta_now , Joint[motorNr].turn_speed_now);
  }
  //Angle selecet
  AngleSelect(Joint[motorNr].theta_now, motorNr, Joint[motorNr].theta_go_to, GB[4], GB[5], Joint[motorNr].speed_to_have, StopS);
  //PWM selector

  //Writing to the specific motor
  if (motorNr == 4)
  {
    WriteData(Joint[motorNr].servo_id, Joint[motorNr].theta_go_to, Joint[motorNr].speed_to_have);
    WriteData(Joint[FingerNR].servo_id, Joint[motorNr].theta_go_to, Joint[motorNr].speed_to_have);
  }
  else
  {
    WriteData(Joint[motorNr].servo_id, Joint[motorNr].theta_go_to, Joint[motorNr].speed_to_have);
  }
  printingPlot();
  Serial3.println(motorNr);
  delay(10);
}
void printingPlot() {
  Serial.print(motorNr); Serial.print(','); Serial.print(GB[3]); Serial.print(','); Serial.print(GB[4]); Serial.print(','); Serial.print(GB[5]); Serial.print(',');
  Serial.print(Joint[1].theta_now); Serial.print(','); Serial.print(Joint[2].theta_now); Serial.print(','); Serial.print(Joint[3].theta_now); Serial.print(',');
  Serial.print(Joint[4].theta_now); Serial.print(','); Serial.println(Joint[5].theta_now);
  delay(1);
  return;
}
void emgCom(int &sumx, int &sumy, int &sumz, int &sumemg1, int &sumemg2 ) {
  byte disbyte[22];
  bool chkRec = false;
  while (chkRec == false)
  {
    if (Serial2.available() >= 24)
    {
      if (Serial2.read() == 126) 
      {
        for (int i = 0; i <= 22; i++)
        {
          disbyte[i] = Serial2.read();
        }
        int chkSum = 0;
        for (int i = 2; i <= 22; i++) {
          chkSum = chkSum + disbyte[i];
        }
        if (lowByte(chkSum) == 0xFF)
        {
          sumz = disbyte[13] + (disbyte[12] * 256);
          sumy = disbyte[15] + (disbyte[14] * 256);
          sumx = disbyte[17] + (disbyte[16] * 256);
          sumemg1 = disbyte[19] + (disbyte[18] * 256);
          sumemg2 = disbyte[21] + (disbyte[20] * 256);
          if ( sumemg1 <= 1024 && sumemg2 <= 1024 )
          {
            chkRec = true;
          }
          else{}
        }
      }
    }
  }
  return;
};
void MotorSwitch(int &MotorNR, int &ZZ, bool &timeCehk, long &Time) {
  int ZLimitUpper = 800;
  int ZLimitLower = 500;
  long TimeLimit = 1500;
  long t = millis();
  if ((t - Time) >= TimeLimit)
  {
    timeCehk = false;
  }
  else
  {
    timeCehk = true; 
  }
  if ((ZZ > ZLimitUpper) && (timeCehk == false))
  {
    Time = millis();
    MotorNR = MotorNR + 1;
    if (MotorNR >= 5)
    {
      MotorNR = 1;
    }
  }
  if ((ZZ < ZLimitLower) && (timeCehk == false))
  {
    MotorNR = MotorNR - 1;
    Time = millis();
    if (MotorNR <= 0)
    {
      MotorNR = 4;
    }
  }
  return;
};

void AngleSelect(int32_t &ArrNow, int &MotorNR, int &ArrGoal, int EMG_1, int EMG_2, int32_t &PWM, bool &on) {
  int thredH = 50;
  int thredL = 50;
  int AngleChange = 10;
  if (MotorNR == 1)
  {
    if (EMG_1 >= thredH && EMG_2 <= thredL)
    {
      ArrGoal = ArrGoal + AngleChange;
      PWM = 360;
      on = true;
    }
    else if (EMG_2 >= thredH && EMG_1 <= thredL)
    {
      ArrGoal = ArrGoal - AngleChange;
      PWM = 360;
      on = true;
    }
    else
    {
      if (on == true)
      {
        ArrGoal = ArrNow;
      }
      on = false;
    }
  }

  else 
  {
    if (EMG_1 >= thredH && EMG_2 <= thredL) 
    {
      ArrGoal = ArrGoal + AngleChange;
      PWM = 360;
      on = true;
    }
    else if (EMG_2 >= thredH && EMG_1 <= thredL)
    {
      ArrGoal = ArrGoal - AngleChange;
      PWM = 262;
      on = true;
    }
    else
    {
      if (on == true)
      {
        ArrGoal = ArrNow;
        on = false;
      }
    }
  }
  return;
};
void  WriteUpdateCCdata() {
  WriteData(Joint[1].servo_id, Joint[1].theta_go_to, Joint[1].speed_to_have);
  WriteData(Joint[2].servo_id, Joint[2].theta_go_to, Joint[2].speed_to_have);
  WriteData(Joint[3].servo_id, Joint[3].theta_go_to, Joint[3].speed_to_have);
  WriteData(Joint[4].servo_id, Joint[4].theta_go_to, Joint[4].speed_to_have);
  WriteData(Joint[5].servo_id, Joint[5].theta_go_to, Joint[5].speed_to_have);
  return;
};
void ReadUpdateCCData() {
  ReadData(Joint[1].servo_id, Joint[1].theta_now , Joint[1].turn_speed_now);
  ReadData(Joint[2].servo_id, Joint[2].theta_now , Joint[2].turn_speed_now);
  ReadData(Joint[3].servo_id, Joint[3].theta_now , Joint[3].turn_speed_now);
  ReadData(Joint[4].servo_id, Joint[4].theta_now , Joint[4].turn_speed_now);
  ReadData(Joint[5].servo_id, Joint[5].theta_now , Joint[5].turn_speed_now);
  return;
};
