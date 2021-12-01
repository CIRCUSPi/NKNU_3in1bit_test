;/*****************************************************
 * Version: 1.0
 * date: 2021/9/22
 * NKNU 3in1:bit test program
 * requirement library:
 *      u8g2 by olikraus                    https://github.com/olikraus/u8g2
 *      FaBo9Axis_MPU9250 by FaBoPlatform   https://github.com/FaBoPlatform/FaBo9AXIS-MPU9250-Library
 *      ESP32Servo by madhephaestus         https://madhephaestus.github.io/ESP32Servo/annotated.html    
 */

//#define DEBUG

const uint16_t Fcy[12] = {
  //   C        C#       D        Eb       E        F       F#        G       G#        A       Bb        B
      4186,    4435,    4699,    4978,    5274,    5588,    5920,    6272,    6645,    7040,    7459,    7902
  };

const unsigned long test_color[] = {
     0x00FF00, 0xFF0000, 0x0000FF, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0xFFFFFF, 0x000000
};
#include <Wire.h>
#include <U8g2lib.h>

#include <FaBo9Axis_MPU9250.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C     u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
FaBo9Axis   fabo_9axis;
int         enable_mpu9250;

float           ax, ay, az;
float           gx, gy, gz;
float           mx, my, mz;
float           temp;
int             nknu_temperature, nknu_light, nknu_mic;
unsigned long   ws2812_color[3];

#if (defined ARDUINO_AVR_UNO) || (defined ARDUINO_AVR_NANO)
#define PIN_BUZZ        11  //P0
#define PIN_BUTTON_A    12   //P5
#define PIN_BUTTON_B    13  //P11
#define BEATS_DELAY     350

#elif defined ARDUINO_ARCH_ESP32
#include <ESP32Servo.h>
#define PIN_BUZZ        14  //P0
#define PIN_BUTTON_A    2   //P5
#define PIN_BUTTON_B    36  //P11
#define BEATS_DELAY     350

#else   //7697
#define PIN_BUZZ        19  //P0
#define PIN_BUTTON_A    0   //P5
#define PIN_BUTTON_B    7  //P11
#define BEATS_DELAY     350

#endif

#define UPDATE_OLED_INTERVAL        200
#define READ_MPU9250_INTERVAL       200
#define READ_SENSOR_INTERVAL        100

void setup()
{
#ifdef ARDUINO_ARCH_ESP32
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
#endif

    pinMode(PIN_BUTTON_A, INPUT);
    pinMode(PIN_BUTTON_B, INPUT);
    pinMode(PIN_BUZZ, OUTPUT);
    
    Serial.begin(115200);
    Wire.begin(); // join i2c bus (address optional for master)

    if (fabo_9axis.begin()) {
        enable_mpu9250 = 1;
    } else {
        enable_mpu9250 = 0;
    }

    u8g2.begin();
    
}

byte x = 0;
#define PIC32_I2C_ADDR          0x1C
#define PIC32_TEST_REG          0x02
#define PIC32_REG_MAX           32

void buzz_tester() {
    tone(PIN_BUZZ, Fcy[5], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[3], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[3], BEATS_DELAY * 2); 
 
    tone(PIN_BUZZ, Fcy[4], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[2], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[2], BEATS_DELAY * 2); 
    
    tone(PIN_BUZZ, Fcy[1], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[2], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[3], BEATS_DELAY);
    tone(PIN_BUZZ, Fcy[4], BEATS_DELAY);
    
    tone(PIN_BUZZ, Fcy[5], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[5], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[5], BEATS_DELAY * 2); 
    
    
    tone(PIN_BUZZ, Fcy[5], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[3], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[3], BEATS_DELAY * 2); 
 
    tone(PIN_BUZZ, Fcy[4], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[2], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[2], BEATS_DELAY * 2); 
    
    tone(PIN_BUZZ, Fcy[1], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[3], BEATS_DELAY); 
    tone(PIN_BUZZ, Fcy[5], BEATS_DELAY);
    tone(PIN_BUZZ, Fcy[5], BEATS_DELAY);
    tone(PIN_BUZZ, Fcy[1], BEATS_DELAY * 4); 
}

void write_i2c_regs()
{
    int addr = 0;
    static int c  = 0;
    
    Serial.println("\n\n##Write Registers");

    Wire.beginTransmission(PIC32_I2C_ADDR);
    
    Wire.write(addr); 
    for (int cnt = c; cnt < c+32; cnt++) {
        Wire.write(cnt); 
    }
    
    Wire.endTransmission();    // stop transmitting

    c = c << 1;
    c |= 1; 
    if (c+32 >= 256) {
        c = 0;
    }
}

#define I2C_REG_TEMPERATURE     0x00
#define I2C_REG_LIGHT           0x02
#define I2C_REG_MIC             0x12
#define I2C_REG_WS2812          0x05
#define I2C_REG_LEDUPDATE       0x0E

long read_i2c_reg(int reg, int len)
{
    long     temp=0;
    
    Wire.beginTransmission(PIC32_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(PIC32_I2C_ADDR, len);
    while (Wire.available()) {
        temp = (temp<<8) | Wire.read();
    }
    return temp;
}

int read_temperature()
{
    return (int)read_i2c_reg(I2C_REG_TEMPERATURE, 2);
}

int read_light()
{
    return (int)read_i2c_reg(I2C_REG_LIGHT, 2);
}

int read_mic()
{
    return (int)read_i2c_reg(I2C_REG_MIC, 2);
}

void read_i2c_regs()
{
    int addr = 0;
    int cnt = 0;
    char outstr[32];
    
    Serial.print("\n\n##Read I2C Registers");
    Wire.beginTransmission(PIC32_I2C_ADDR);
    Wire.write(addr);
    Wire.endTransmission();
    
    Wire.requestFrom(PIC32_I2C_ADDR, 32);
    while (Wire.available()) { // slave may send less than requested
        
        if (cnt%8 == 0) {
            sprintf(outstr, "\n REG %02X:", cnt);
            Serial.print(outstr);
        }
        cnt++;
        char c = Wire.read(); // receive a byte as character
        sprintf(outstr, " %02X ", c);
        Serial.print(outstr);         // print the character
    }
}

void write_i2c_ws2812(unsigned long *color, int num)
{
    int cnt = 0;
    char outstr[32];
    byte w_data;

#ifdef DEBUG  
    Serial.println("\n\n ## write ws2812");
#endif

    Wire.beginTransmission(PIC32_I2C_ADDR);
    Wire.write(I2C_REG_WS2812);
    while (num) {
        for (int ii = 0; ii < 3; ii++) {
            w_data = color[cnt] >> 8*ii;
            Wire.write(w_data);
#ifdef DEBUG            
            sprintf(outstr, "cnt %d: %X (%X)\t", cnt, w_data, color[cnt]);
            Serial.print(outstr);
#endif            
        }
#ifdef DEBUG            
        Serial.println("");
#endif
        cnt++;
        num--;
    }
    Wire.endTransmission();
}

void update_i2c_ws2812()
{
    int cnt = 0;
    
    Wire.beginTransmission(PIC32_I2C_ADDR);
    Wire.write(I2C_REG_LEDUPDATE);
    Wire.write(1);
    Wire.endTransmission();
}

void read_mpu9250() {
    
    fabo_9axis.readAccelXYZ(&ax,&ay,&az);
    fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
    fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
    fabo_9axis.readTemperature(&temp);

#ifdef DEBUG
    Serial.print("ax: ");
    Serial.print(ax);
    Serial.print(" ay: ");
    Serial.print(ay);
    Serial.print(" az: ");
    Serial.println(az);

    Serial.print("gx: ");
    Serial.print(gx);
    Serial.print(" gy: ");
    Serial.print(gy);
    Serial.print(" gz: ");
    Serial.println(gz);

    Serial.print("mx: ");
    Serial.print(mx);
    Serial.print(" my: ");
    Serial.print(my);
    Serial.print(" mz: ");
    Serial.println(mz);

    Serial.print("temp: ");
    Serial.println(temp);
#endif 
}

void update_OLED()
{

    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_profont12_tr);

        u8g2.setCursor(0, 10);
        u8g2.print(String(F("MPU9250 X    Y     Z")).c_str());
        
        u8g2.setCursor(0, 20);
        u8g2.print(String(String(F("ACC:")) + ax).c_str());
        u8g2.setCursor(60, 20);
        u8g2.print(String(ay).c_str());
        u8g2.setCursor(96, 20);
        u8g2.print(String(az).c_str());

        u8g2.setCursor(0, 30);
        u8g2.print(String(String(F("GRY:")) + gx).c_str());
        u8g2.setCursor(60, 30);
        u8g2.print(String(gy).c_str());
        u8g2.setCursor(95, 30);
        u8g2.print(String(gz).c_str());
        
        u8g2.setCursor(0, 40);
        u8g2.print(String(String(F("MAG:")) + mx).c_str());
        u8g2.setCursor(60, 40);
        u8g2.print(String(my).c_str());
        u8g2.setCursor(95, 40);
        u8g2.print(String(mz).c_str());

        u8g2.setCursor(0, 50);
        u8g2.print(String(String(F("T:")) + nknu_temperature).c_str());

        u8g2.setCursor(40, 50);
        u8g2.print(String(String(F("L:")) + nknu_light).c_str());

        u8g2.setCursor(80, 50);
        u8g2.print(String(String(F("M:")) + nknu_mic).c_str());

        u8g2.setCursor(0, 60);
        u8g2.print(String(String(F("COLOR: 0x")) + String(ws2812_color[0], HEX)).c_str());
        
        u8g2.sendBuffer();
    } while ( u8g2.nextPage() );
    
}

void loop()
{
    static byte    update_ws2812 = 0, cnt = 0;
    char    debug_str[64];
    static unsigned long    update_oled_ms = 0;
    static unsigned long    read_mpu9250_ms = 0;
    static unsigned long    read_sensors_ms = 0;

    
    if (read_sensors_ms < millis()) {
        read_sensors_ms = millis() + READ_SENSOR_INTERVAL;
        nknu_temperature = read_temperature();
        nknu_light = read_light();
        nknu_mic = read_mic();
    }

#ifdef DEBUG
    sprintf(debug_str, "temp: %d, \tlight: %d, \tmic: %d", nknu_temperature, nknu_light, nknu_mic);
    Serial.println(debug_str);;
#endif

    if (update_oled_ms < millis()) {
        update_oled_ms = millis() + UPDATE_OLED_INTERVAL;
        update_OLED();
    }

    if (enable_mpu9250) {
        if (read_mpu9250_ms < millis()) {
            read_mpu9250_ms < millis() + READ_MPU9250_INTERVAL;
            read_mpu9250();
        }
    }

    if (update_ws2812) {
        write_i2c_ws2812(ws2812_color, 3);
        update_i2c_ws2812();
        delay(75);
        update_ws2812 = 0;
#ifdef DEBUG 
        read_i2c_regs();
#endif
    }

    if (digitalRead(PIN_BUTTON_A) == LOW) {
        buzz_tester();
    }

    if (digitalRead(PIN_BUTTON_B) == LOW) {
        // simple check for debounce
        delay(50);
        if (digitalRead(PIN_BUTTON_B) == LOW) {
            if (test_color[cnt] != 0) {
                cnt++;
            } else {
                cnt = 0;
            }
            
            ws2812_color[0] = ws2812_color[1] = ws2812_color[2] = test_color[cnt];
            update_ws2812 = 1;
        }
    }
}
