//NOTE: this example was tested on ESP32, it will need some gps serial port adaptions for other Arduino platforms. (Not all Arduino Serial classes were created equal!!!)

#define RX_PIN 13
#define TX_PIN 27

#include <qqqlab_GPS_UBLOX.h>
#include <qqqlab_AutoBaud.h>

class GPS_UBLOX : public AP_GPS_UBLOX {
public:
  HardwareSerial *gps_serial;
 
  void begin(HardwareSerial *gps_serial) {
    this->gps_serial = gps_serial;
  }

  //interface
  void I_setBaud(int baud)                      override {gps_serial->begin(baud);}
  inline int I_availableForWrite()              override {return gps_serial->availableForWrite();}
  inline int I_available()                      override {return gps_serial->available();}
  inline int I_read(uint8_t* data, size_t len)  override {return gps_serial->read(data, len);}
  inline int I_write(uint8_t* data, size_t len) override {return gps_serial->write(data, len);}
  inline uint32_t I_millis()                    override {return ::millis();}
  void I_print(const char *str)                 override {Serial.print("[AP_GPS_UBLOX] "); Serial.print(str);}
} gps;



void setup() {
  Serial.begin(115200);
  while(!Serial);

  //initial GPS baud rate to try
  int baud = 230400;

  //optional auto-baud to speed up gps connection
  baud = autobaud(RX_PIN);
  
  Serial.printf("Initial GPS baud rate:%d\n", baud);

  //start GPS Serial
  Serial1.begin(baud, SERIAL_8N1, RX_PIN, TX_PIN);

  //start GPS
  gps.rate_ms = 100;   //optional - gps update rate in milliseconds (default 100)
  gps.save_config = 2, //optional - save config  0:Do not save config, 1:Save config, 2:Save only when needed (default 2)
  gps.gnss_mode = 0;   //optonial - GNSS system(s) to use  Bitmask: 1:GPS, 2:SBAS, 4:Galileo, 8:Beidou, 16:IMES, 32:QZSS, 64:GLONASS (default 0=leave as configured)
  gps.begin(&Serial1);
}



void loop() {
  static uint32_t ts = millis();
  
  //update GPS (call at least 10 times per second)
  gps.update();

  //print GPS state
  if(millis() - ts > 1000) {
    ts= millis();
    Serial.printf("tow:%d dt:%d sats:%d lat:%d lng:%d alt:%d hacc:%d vacc:%d fix:%d\n"
    , (int)gps.state.time_week_ms
    , (int)gps.timing.average_delta_us
    , (int)gps.state.num_sats
    , (int)gps.state.lat
    , (int)gps.state.lng
    , (int)gps.state.alt
    , (int)gps.state.horizontal_accuracy
    , (int)gps.state.vertical_accuracy   
    , (int)gps.state.status      
    );
  }
}