// Code from: http://www.lucidarme.me/?p=5057

// A4 = SDA on Sensor
// A5 = SCL on Sensor 

// See online schematic sheet: https://www.google.com/search?q=arduino+nano+sheet&client=ubuntu&hs=Emt&channel=fs&tbm=isch&imgil=FhjJXJGxSH0HiM%253A%253B_jVw9HNtBEH47M%253Bhttp%25253A%25252F%25252Favrchip.com%25252Farduino-nano-datasheet-and-tutorial%25252F&source=iu&pf=m&fir=FhjJXJGxSH0HiM%253A%252C_jVw9HNtBEH47M%252C_&usg=__2BsRa7xSwGqmFeFvP-PzWHOYFlo%3D&biw=1366&bih=673&ved=0ahUKEwiL6cr5oLrQAhUD1GMKHYdWCYUQyjcINQ&ei=OR8zWMvoHIOojwOHraWoCA#imgrc=FhjJXJGxSH0HiM%3A

#include <Wire.h>
 
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
#define    M_PI 3.14159265
 
 
 
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
 
// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
 
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
 
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
 
 
}
 
 
long int cpt=0;
// Main loop, read and display data
void loop()
{
 
  // _______________
  // ::: Counter :::
 
  // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print ("\t");
 
 
  
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 
 
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
 
 
  // Create 16 bits values from 8 bits data
 
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];
 
    // Display values
 
  signed int Ax = (signed int)(((signed int)ax)*3.9);
  signed int Ay = (signed int)(((signed int)ay)*3.9);
  signed int Az = (signed int)(((signed int)az)*3.9);
  
  
  signed int pitch = 180*atan(Ax/sqrt(Ay*Ay+Az*Az))/M_PI;
  signed int roll = 180*atan(Ay/sqrt(Ax*Ax+Az*Az))/M_PI;
  signed int yaw = 180*atan(Az/sqrt(Ax*Ax+Ay*Ay))/M_PI;
  
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(yaw);
  
  
  /*
  // Accelerometer
  Serial.print (Ax,DEC); 
  Serial.print ("\t");
  Serial.print (Ay,DEC);
  Serial.print ("\t");
  Serial.print (Az,DEC);  
  Serial.print ("\t");
  */
  
 
  // End of line
  Serial.println("");
  delay(100);    
}
