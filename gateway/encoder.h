#include <SPI.h>

//******************************************************************************************
// VS1053Ogg class definition. *
//******************************************************************************************
class VS1053Ogg {
private:
  uint8_t cs_pin;    // Pin where CS line is connected
  uint8_t dcs_pin;   // Pin where DCS line is connected
  uint8_t dreq_pin;  // Pin where DREQ line is connected
  uint8_t reset_pin; // Pin where RESET line is connected
  // SCI Register
  const uint8_t SCI_MODE = 0x0;
  const uint8_t SCI_BASS = 0x2;
  const uint8_t SCI_CLOCKF = 0x3;
  const uint8_t SCI_AUDATA = 0x5;
  const uint8_t SCI_WRAM = 0x6;
  const uint8_t SCI_WRAMADDR = 0x7;
  const uint8_t SCI_HDAT0 = 0x8;
  const uint8_t SCI_HDAT1 = 0x9;
  const uint8_t SCI_AIADDR = 0xA;
  const uint8_t SCI_VOL = 0xB;
  const uint8_t SCI_AICTRL0 = 0xC;
  const uint8_t SCI_AICTRL1 = 0xD;
  const uint8_t SCI_AICTRL2 = 0xE;
  const uint8_t SCI_AICTRL3 = 0xF;
  const uint8_t SCI_num_registers = 0xF;
  // SCI_MODE bits
  const uint8_t SM_RESET = 2;   // Bitnumber in SCI_MODE soft reset
  const uint8_t SM_CANCEL = 3;  // Bitnumber in SCI_MODE cancel song
  const uint8_t SM_TESTS = 5;   // Bitnumber in SCI_MODE for tests
  const uint8_t SM_SDINEW = 11; // Bitnumber in SCI_MODE always on
  const uint8_t SM_ADPCM = 12;  // Bitnumber in SCI_MODE for recording
  const uint8_t SM_LINE1 = 14;  // Bitnumber in SCI_MODE for Line input
  SPISettings VS1053_SPI;       // SPI settings for this slave
  uint16_t plg_sa;              // Result load image, startaddress plugin (0x34)
  bool activeflag = false;      // Active or not
protected:
  inline void await_data_request() const {
    delayMicroseconds(10);
    while (!digitalRead(dreq_pin)) {
      yield(); // Very short delay
    }
  }

  inline void control_mode_on() const {
    SPI.beginTransaction(VS1053_SPI); // Prevent other SPI users
    digitalWrite(dcs_pin, HIGH);      // Bring slave in control mode
    digitalWrite(cs_pin, LOW);
  }

  inline void control_mode_off() const {
    digitalWrite(cs_pin, HIGH); // End control mode
    SPI.endTransaction();       // Allow other SPI users
  }

  inline void data_mode_on() const {
    SPI.beginTransaction(VS1053_SPI); // Prevent other SPI users
    digitalWrite(cs_pin, HIGH);       // Bring slave in data mode
    digitalWrite(dcs_pin, LOW);
  }

  inline void data_mode_off() const {
    digitalWrite(dcs_pin, HIGH); // End data mode
    SPI.endTransaction();        // Allow other SPI users
  }

  uint16_t read_register(uint8_t _reg);
  void write_register(uint8_t _reg, uint16_t _value);
  bool write_register(uint8_t _reg, uint16_t _value, uint16_t timeout);
  void wram_write(uint16_t address, uint16_t data);
  uint16_t wram_read(uint16_t address);
  uint16_t SpiLoadImage(String imgfile);
  void LoadUserCode(String cmdfile);

public:
  // Constructor.  Only sets pin values.  Doesn't touch the chip.  Be sure to
  // call begin()!
  VS1053Ogg(uint8_t _cs_pin, uint8_t _dcs_pin, uint8_t _dreq_pin,
            uint8_t _reset_pin);
  void begin();    // Begin operation.  Sets pins correctly,
                   // and prepares SPI bus.
  void activate(); // Activate conversion
  bool active();   // Return active state
  void printDetails(
      const char *header); // Print configuration details to serial output.
  void softReset();        // Do a soft reset
  bool testComm(const char *header); // Test communication with module
  int getRecordingTime();            // Read recording time

  inline bool data_request() const { return (digitalRead(dreq_pin) == HIGH); }

  inline uint16_t available() // Return number of bytes in VS1053 fifo
  {
    return read_register(SCI_HDAT1);
  }

  inline uint16_t read() // Get next word from VS1053 fifo
  {
    return read_register(SCI_HDAT0);
  }
};