//******************************************************************************************
//*  Ogg-encoder -- Encodes the analog input of the VS1053 and make it available
// via WiFi. *
//*  Source code is suitable for ESP8266. *
//*  After reset, the stream can be found at <IP>:8080. *
//*  Be sure to have the file "v44k1q05.img" in the SPIFFS. *
//*  Issues: Sometimes the conversion will not start. Sometimes after 5 tries. *
//*          Delay time about 2.5 seconds. *
//*          Client disconnect not always seen. *
//******************************************************************************************
// ESP8266 libraries used:
//  - ESP8266WiFi
//  - ESP8266mDNS
//  - SPI
//  - FS
//  - ArduinoOTA
//
// Wiring:
// NodeMCU  GPIO    Pin to program  Wired to VS1053      Wired to rest
// -------  ------  --------------  -------------------  ---------------------
// D0       GPIO16  16              pin 1 DCS            -
// D1       GPIO5    5              pin 2 CS             LED on nodeMCU
// D2       GPIO4    4              pin 4 DREQ           -
// D3       GPIO0    0 FLASH        -                    -
// D4       GPIO2    2              pin 3 RESET          -
// D5       GPIO14  14 SCLK         pin 5 SCK            -
// D6       GPIO12  12 MISO         pin 7 MISO           -
// D7       GPIO13  13 MOSI         pin 6 MOSI           -
// D8       GPIO15  15              -                    -
// D9       GPI03    3 RXD0         -                    Reserved serial input
// D10      GPIO1    1 TXD0         -                    Reserved serial output
// -------  ------  --------------  -------------------  ---------------------
// GND      -        -              pin 8 GND            Power supply
// VCC 3.3  -        -              -                    LDO 3.3 Volt
// VCC 5 V  -        -              pin 9 5V             Power supply
//
//
// 09-05-2016, ES: First set-up.
// 12-05-2016, ES: Added dummy html Ogg header.
// 18-05-2016, ES: Reset of VS1053 to GPIO2, restart after disconnect
// 19-05-2016, ES: Changed wait for DREQ on VS1053 read/write registers
// 24-05-2016, ES: Correct http-header
//
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <SPI.h>
#include <Ticker.h>
#include <WiFiClient.h>
#include <stdio.h>
#include <string.h>

#include "encoder.h"

extern "C" {
#include "user_interface.h"
}

// Digital I/O used
// Pins for VS1053 module
#define VS1053_CS 5
#define VS1053_DCS 16
#define VS1053_DREQ 4
#define VS1053_RESET 2

// Size of buffer
#define BUFFSIZ 1400
// Debug buffer size
#define DEBUG_BUFFER_SIZE 100

//******************************************************************************************
// Global data section. *
//******************************************************************************************
enum datamode_t { INIT, HEADER, DATA, METADATA }; // State for datastream
// Global variables
// String ssid;              // SSID of selected WiFi network
int DEBUG = 1; // Debug on or off
// WiFiServer server(8080);  // Ogg stream server object
// WiFiClient serverClient;  // Just one listener
Ticker tckr;              // For timing 10 sec
char sbuf[400];           // For debug lines
int gconv = 0;            // Total converted
uint8_t dbuffer[BUFFSIZ]; // Buffer with Ogg data
//

const char *oggheader = "ICY 200 OK\r\n"
                        "Content-Type: audio/ogg\r\n"
                        "icy-name:Ogg-encoder by Ed Smallenburg\r\n"
                        "icy-br:Quality 2\r\n\r\n";
//

//******************************************************************************************
// End of global data section. *
//******************************************************************************************

char *dbgprint(const char *format, ...); // Forward declaration

//******************************************************************************************
// VS1053Ogg stuff.  Based on maniacbug library. *
//******************************************************************************************

//******************************************************************************************
// VS1053Ogg class implementation. *
//******************************************************************************************

VS1053Ogg::VS1053Ogg(uint8_t _cs_pin, uint8_t _dcs_pin, uint8_t _dreq_pin,
                     uint8_t _reset_pin)
    : cs_pin(_cs_pin), dcs_pin(_dcs_pin), dreq_pin(_dreq_pin),
      reset_pin(_reset_pin) {}

uint16_t VS1053Ogg::read_register(uint8_t _reg) {
  uint16_t result;

  await_data_request(); // Wait for DREQ to be HIGH again
  control_mode_on();
  SPI.write(3);    // Read operation
  SPI.write(_reg); // Register to write (0..0xF)
  // Note: transfer16 does not seem to work
  result = (SPI.transfer(0xFF) << 8) | // Read 16 bits data
           (SPI.transfer(0xFF));
  control_mode_off();
  return result;
}

void VS1053Ogg::write_register(uint8_t _reg, uint16_t _value) {
  await_data_request();
  control_mode_on();
  SPI.write(2);        // Write operation
  SPI.write(_reg);     // Register to write (0..0xF)
  SPI.write16(_value); // Send 16 bits data
  control_mode_off();
}

bool VS1053Ogg::write_register(uint8_t _reg, uint16_t _value,
                               uint16_t timeout) {
  // Same function, but with time-out
  while (!digitalRead(dreq_pin) && --timeout) {
    delay(1); // Short delay
  }
  control_mode_on();
  SPI.write(2);        // Write operation
  SPI.write(_reg);     // Register to write (0..0xF)
  SPI.write16(_value); // Send 16 bits data
  control_mode_off();
  if (timeout == 0) {
    dbgprint("VS1053 time-out writing register %04X with value %04X", _reg,
             _value);
    return false;
  }
  return true;
}

void VS1053Ogg::wram_write(uint16_t address, uint16_t data) {
  write_register(SCI_WRAMADDR, address);
  write_register(SCI_WRAM, data);
}

uint16_t VS1053Ogg::wram_read(uint16_t address) {
  write_register(SCI_WRAMADDR, address); // Start reading from WRAM
  return read_register(SCI_WRAM);        // Read back result
}

int VS1053Ogg::getRecordingTime() // Read recording time
{
  // Recording time is at ram location 8 (LSB) and 9 (MSB)
  return wram_read(0x9) << 16 | wram_read(0x8);
}

bool VS1053Ogg::testComm(const char *header) {
  // Test the communication with the VS1053 module.  The result wille be
  // returned. If DREQ is low, there is problably no VS1053 connected.  Pull the
  // line HIGH in order to prevent an endless loop waiting for this signal.  The
  // rest of the software will still work, but readbacks from VS1053 will fail.
  int i; // Loop control
  uint16_t r1, r2, cnt = 0;
  uint16_t delta = 300; // 3 for fast SPI

  if (!digitalRead(dreq_pin)) {
    dbgprint("VS1053 not properly installed!");
    // Allow testing without the VS1053 module
    pinMode(dreq_pin, INPUT_PULLUP); // DREQ is now input with pull-up
    return false;                    // Return bad result
  }
  // Further TESTING.  Check if SCI bus can write and read without errors.
  // We will use the volume setting for this.
  // Will give warnings on serial output if DEBUG is active.
  // A maximum of 20 errors will be reported.
  if (strstr(header, "Fast")) {
    delta = 3; // Fast SPI, more loops
  }
  dbgprint(header); // Show a header
  for (i = 0; (i < 0xFFFF) && (cnt < 20); i += delta) {
    write_register(SCI_VOL, i);         // Write data to SCI_VOL
    r1 = read_register(SCI_VOL);        // Read back for the first time
    r2 = read_register(SCI_VOL);        // Read back a second time
    if (r1 != r2 || i != r1 || i != r2) // Check for 2 equal reads
    {
      dbgprint("VS1053 error retry SB:%04X R1:%04X R2:%04X", i, r1, r2);
      cnt++;
      delay(10);
    }
    yield(); // Allow ESP firmware to do some bookkeeping
  }
  return (cnt == 0); // Return the result
}

bool VS1053Ogg::active() {
  return activeflag; // Get active state
}

void VS1053Ogg::activate() {
  for (int i = 0; i < 2; i++) {
    dbgprint("Starting conversion");
    if (write_register(SCI_AIADDR, plg_sa,
                       200)) // Activate recording, timeout 200 msec
    {
      dbgprint("Conversion started");
      activeflag = true; // Remember state
      break;
    }
  }
  if (!active()) // Success?
  {
    dbgprint("Error starting conversion!"); // No, report
  }
}

void VS1053Ogg::begin() {
  pinMode(dreq_pin, INPUT); // DREQ is an input
  pinMode(cs_pin, OUTPUT);  // The SCI and SDI signals
  pinMode(dcs_pin, OUTPUT);
  pinMode(reset_pin, OUTPUT);  // The reset pin
  digitalWrite(dcs_pin, HIGH); // Start HIGH for SCI en SDI
  digitalWrite(cs_pin, HIGH);
  digitalWrite(reset_pin, HIGH); // No reset yet
  delay(100);
  dbgprint("Reset VS1053...");
  digitalWrite(reset_pin, LOW); // Reset pin low
  delay(1000);
  dbgprint("End reset VS1053...");
  digitalWrite(reset_pin, HIGH); // Back to normal again
  delay(500);
  // Init SPI in slow mode ( 1 MHz )
  VS1053_SPI = SPISettings(100000, MSBFIRST, SPI_MODE0);
  // printDetails ( "Right after reset/startup" ) ;
  delay(20);
  // printDetails ( "20 msec after reset" ) ;
  testComm("Slow SPI, Testing VS1053 read/write registers...");
  // The should not start in midi mode.  We force GPIO[0] and GPIO[1] to LOW:
  wram_write(0xC017, 3); // GPIO DDR = 3
  wram_write(0xC019, 0); // GPIO ODATA = 0
  VS1053_SPI = SPISettings(4000000, MSBFIRST, SPI_MODE0);
  testComm("Fast SPI, Testing VS1053 read/write registers again...");
  write_register(SCI_VOL, 0x1010); // Set volume
  softReset();                     // Do a soft reset
  delay(100);
  // The next clocksetting allows SPI clocking at 5 MHz, 4 MHz is safe then.
  write_register(SCI_CLOCKF, 0xC000); // Clock settings just below 55.3 MHz
  write_register(SCI_BASS, 0);        // BASS/TREBLE settigs off
  // Disable all interrupts except the SCI interrupt by writing 0x2 to VS1053’s
  // internal register VS1053_INT_ENABLE.
  wram_write(0xC01A, 2); // 2 to INT_ENABLE
  dbgprint("Loading Ogg plugin");
  plg_sa = SpiLoadImage("/v44k1q05.img");     // Load profile
  dbgprint("Result of load is 0x%X", plg_sa); // Should be 0x34
  if (plg_sa != 0x34) {
    dbgprint("ERROR: Result of should be is 0x34");
    delay(4000);
    ESP.restart();
  }
  // Set bits as instructed (Line input)
  // write_register ( SCI_MODE, _BV ( SM_SDINEW ) | _BV ( SM_LINE1 ) | _BV (
  // SM_ADPCM ) ) ;
  // Set bits as instructed (Microphone input)
  write_register(SCI_MODE, _BV(SM_SDINEW) | _BV(SM_ADPCM), 100);
  write_register(SCI_AICTRL0, 1024, 100); //
  write_register(SCI_AICTRL1, 1024, 100); //
  write_register(SCI_AICTRL2, 0, 100);    //
  write_register(SCI_AICTRL3, 0, 100);    // Miscellaneous bits
  delay(100);
}

void VS1053Ogg::softReset() {
  write_register(SCI_MODE, _BV(SM_SDINEW) | _BV(SM_RESET));
  delay(10);
  await_data_request();
}

void VS1053Ogg::printDetails(const char *header) {
  uint16_t regbuf[16];
  uint8_t i;

  dbgprint(header);
  dbgprint("REG   Contents");
  dbgprint("---   -----");
  for (i = 0; i <= SCI_num_registers; i++) {
    regbuf[i] = read_register(i);
  }
  for (i = 0; i <= SCI_num_registers; i++) {
    delay(5);
    dbgprint("%3X - %5X", i, regbuf[i]);
  }
}

//  SpiLoadImage() loads an image from a file into the memory
//  of VS1053(), then returns the start address to the caller. If it
//  fails, it will return 0xFFFFU.
//  The file format is as follows:
//  The file starts with three characters, "P&H".
//  The rest of the file are followed by records that are in the
//  following format:
//  Tp  L1 L0  A1 A0  D0 D1 D2 ....
//  Tp: The type of the record. Values 0..3 are valid:
//  - 0 = Code (Instructions)
//  - 1 = X memory
//  - 2 = Y memory
//  - 3 = Execute (start address)
//  (L1<<8) + L0: Length of the record in bytes. Always an even number.
//  (A1<<8) + A0: Start address of the record. If the record is of type
//  Execute, then this is the execution start address. It is also the
//  end of the file.
//  Dx: Data. Read this two bytes at a time and send to VS1053 in a
//  big-endian fashion, as shown in the code below.

uint16_t VS1053Ogg::SpiLoadImage(String imgfile) {
  File fp;
  uint16_t type;
  static uint16_t offsets[3] = {0x8000U, 0x0U, 0x4000U};
  uint16_t len, addr, data;
  uint16_t numerr = 0; // Number of errors

  fp = SPIFFS.open(imgfile, "r");
  if (fp) {
    if (fp.read() != 'P' || fp.read() != '&' || fp.read() != 'H') {
      fp.close();
      return 0xFFFF;
    }
  }
  while ((type = fp.read()) >= 0) {
    //  offsets are: Code (I), X Mem, Y Mem when written through SCI_WRAM.
    //  See VS1053 datasheet's documentation for SCI register SCI_WRAMADDR for
    //  details.
    if (type >= 4) {
      fp.close();
      return 0xFFFF; // Error return
    }
    len = (uint16_t)fp.read() << 8;
    len |= fp.read() & ~1;
    addr = (uint16_t)fp.read() << 8;
    addr |= fp.read();
    if (type == 3) {
      fp.close();
      return addr; // Execute record: we can now return with the execute address
    }
    // Set address
    write_register(SCI_WRAMADDR, addr + offsets[type], 10);
    // Write data
    do {
      data = (uint16_t)fp.read() << 8;
      data |= fp.read();
      if (!write_register(SCI_WRAM, data, 10)) {
        if (numerr++ > 10) {
          fp.close();
          return 0xFFFF;
        }
      }
    } while ((len -= 2));
  }
  fp.close();
  return 0xFFFF;
}

// VS1053b flac + latm patch
void VS1053Ogg::LoadUserCode(String cmdfile) {
  File fp;             // File object
  String line;         // Line from file
  uint8_t reg;         // Register to write
  uint16_t data;       // Data to write
  uint16_t numerr = 0; // Number of errors
  int progress = 0;    // To show progress

  fp = SPIFFS.open(cmdfile, "r");
  if (!fp) {
    dbgprint("File open failed");
    return;
  }
  while (fp.available()) {
    line = fp.readStringUntil('\n'); // Read next line
    if (line.startsWith("W 2 "))     // Serious line?
    {
      reg = atoi(line.substring(4).c_str());
      data = atoi(line.substring(6).c_str());
      if ((++progress % 100) == 0) {
        dbgprint("%d words patched", progress);
        dbgprint("Writing reg %02X with %04X", reg, data);
      }
      if (!write_register(reg, data, 10)) {
        if (numerr++ > 10) {
          break;
        }
      }
    }
  }
  fp.close();
}

// The object for the MP3 player
VS1053Ogg ogg(VS1053_CS, VS1053_DCS, VS1053_DREQ, VS1053_RESET);

//******************************************************************************************
// End VS1053Ogg stuff. *
//******************************************************************************************

//******************************************************************************************
//                                  D B G P R I N T *
//******************************************************************************************
// Send a line of info to serial output.  Works like vsprintf(), but checks the
// BEDUg flag.* Print only if DEBUG flag is true.  Always returns the the
// formatted string.             *
//******************************************************************************************
char *dbgprint(const char *format, ...) {
  static char sbuf[DEBUG_BUFFER_SIZE]; // For debug lines
  va_list varArgs;                     // For variable number of params

  va_start(varArgs, format);                      // Prepare parameters
  vsnprintf(sbuf, sizeof(sbuf), format, varArgs); // Format the message
  va_end(varArgs);                                // End of using parameters
  if (DEBUG)                                      // DEBUG on?
  {
    Serial.print("D: ");  // Yes, print prefix
    Serial.println(sbuf); // and the info
  }
  return sbuf; // Return stored string
}

//******************************************************************************************
//                             G E T E N C R Y P T I O N T Y P E *
//******************************************************************************************
// Read the encryption type of the network and return as a 4 byte name *
//*********************4********************************************************************
/*
const char *getEncryptionType(int thisType)
{
  switch (thisType)
  {
  case ENC_TYPE_WEP:
    return "WEP ";
  case ENC_TYPE_TKIP:
    return "WPA ";
  case ENC_TYPE_CCMP:
    return "WPA2";
  case ENC_TYPE_NONE:
    return "None";
  case ENC_TYPE_AUTO:
    return "Auto";
  }
  return "????";
}

//******************************************************************************************
//                                L I S T N E T W O R K S *
//******************************************************************************************
// List the available networks and select the strongest. * Acceptable networks
// are those who have a "SSID.pw" file in the SPIFFS.                  *
//******************************************************************************************
void listNetworks()
{
  int maxsig = -1000; // Used for searching strongest WiFi signal
  int newstrength;
  byte encryption;        // TKIP(WPA)=2, WEP=5, CCMP(WPA)=4, NONE=7, AUTO=8
  const char *acceptable; // Netwerk is acceptable for connection
  int i, j;
  bool found;  // True if acceptable network found
  String path; // Full filespec to see if SSID is an acceptable one

  // scan for nearby networks:
  dbgprint("* Scan Networks *");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1)
  {
    dbgprint("Couldn't get a wifi connection");
    return;
  }
  // print the list of networks seen:
  sprintf(sbuf, "Number of available networks: %d", numSsid);
  dbgprint(sbuf);

  // Print the network number and name for each network found and
  // find the strongest acceptable network
  for (i = 0; i < numSsid; i++)
  {
    acceptable = ""; // Assume not acceptable
    path = String("/") + WiFi.SSID(i) + String(".pw");
    newstrength = WiFi.RSSI(i);
    if (found = SPIFFS.exists(path)) // Is this SSID acceptable?
    {
      acceptable = "Acceptable";
      if (newstrength > maxsig) // This is a better Wifi
      {
        maxsig = newstrength;
        ssid = WiFi.SSID(i); // Remember SSID name
      }
    }
    encryption = WiFi.encryptionType(i);
    sprintf(sbuf, "%2d - %-25s Signal: %3d dBm Encryption %4s  %s", i + 1,
            WiFi.SSID(i).c_str(), WiFi.RSSI(i), getEncryptionType(encryption),
            acceptable);
    dbgprint(sbuf);
  }
  dbgprint("--------------------------------------");
}

//******************************************************************************************
//                               C O N N E C T W I F I *
//******************************************************************************************
// Connect to WiFi using passwords available in the SPIFFS. *
//******************************************************************************************
void connectwifi()
{
  String path; // Full file spec
  String pw;   // Password from file
  File pwfile; // File containing password for WiFi

  path = String("/") + ssid + String(".pw"); // Form full path
  pwfile = SPIFFS.open(path, "r");           // File name equal to SSID
  pw = pwfile.readStringUntil('\n');         // Read password as a string
  pw.trim();                                 // Remove CR
  WiFi.begin(ssid.c_str(), pw.c_str());      // Connect to selected SSID
  sprintf(sbuf, "Try WiFi %s",
          ssid.c_str());                           // Message to show during
WiFi connect if (WiFi.waitForConnectResult() != WL_CONNECTED) // Try to connect
  {
    dbgprint("WiFi Failed!");
    return;
  }
  sprintf(sbuf, "IP = %d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1],
          WiFi.localIP()[2], WiFi.localIP()[3]);
  dbgprint(sbuf);
}

//******************************************************************************************
//                                   O T A S T A R T *
//******************************************************************************************
// Update via WiFi has been started by Arduino IDE. *
//******************************************************************************************
void otastart() { dbgprint("OTA Started"); }

//******************************************************************************************
//                                  T I M E R 1 0 *
//******************************************************************************************
// Will be executed every 10 sec.  Reset all if problem detected. * Be sure NOT
// to call "yield" or "delay"!                                                 *
//******************************************************************************************
void timer10()
{
  bool bcon, bact;
  const char *str1 = "connected";
  const char *str2 = "active ";
  static uint16_t errcnt = 0;

  bcon = serverClient && serverClient.connected(); // TCP connected or not
  if (!bcon)
  {
    str1 = "not active";
  }
  bact = ogg.active();
  if (!bact)
  {
    str2 = "not active";
  }
  dbgprint("%d bytes converted, %s, encoder %s", gconv, str1, str2);
  if (bact && !bcon)
  {
    if (++errcnt == 3)
    {
      ESP.restart();
    }
  }
}

//******************************************************************************************
//                                   S E T U P *
//******************************************************************************************
// Setup for the program. *
//******************************************************************************************
void setup() {
  FSInfo fs_info;  // Info about SPIFFS
  Dir dir;         // Directory struct for SPIFFS
  File f;          // Filehandle
  String filename; // Name of file found in SPIFFS

  Serial.begin(115200); // For debug
  Serial.println();
  system_update_cpu_freq(160); // Set to 80/160 MHz
  SPIFFS.begin();              // Enable file system
  // Show some info about the SPIFFS
  SPIFFS.info(fs_info);
  sprintf(sbuf, "FS Total %d, used %d", fs_info.totalBytes, fs_info.usedBytes);
  dbgprint(sbuf);
  dir = SPIFFS.openDir("/"); // Show files in FS
  while (dir.next())         // All files
  {
    f = dir.openFile("r");
    filename = dir.fileName();
    sprintf(sbuf, "%-32s - %6d", // Show name and size
            filename.c_str(), f.size());
    dbgprint(sbuf);
  }
  WiFi.mode(WIFI_STA); // This ESP is a station
  wifi_station_set_hostname((char *)"Ogg-decoder");
  SPI.begin();  // Init SPI bus
  sprintf(sbuf, // Some memory info
          "Starting Ogg-recorder Version 25-05-2016...  Free memory %d",
          system_get_free_heap_size());
  dbgprint(sbuf);
  sprintf(sbuf, // Some sketch info
          "Sketch size %d, free size %d", ESP.getSketchSize(),
          ESP.getFreeSketchSpace());
  dbgprint(sbuf);
  listNetworks(); // Search for strongest WiFi network
  sprintf(sbuf, "Selected network: %-25s", ssid.c_str());
  dbgprint(sbuf);
  connectwifi(); // Connect to WiFi network
  MDNS.begin("Ogg-encoder");
  ArduinoOTA.setHostname("Ogg-encoder"); // Set the hostname
  ArduinoOTA.onStart(otastart);
  ArduinoOTA.begin(); // Allow update over the air
  server.begin();     // Start server
  server.setNoDelay(true);
  memset(dbuffer, 0, sizeof(dbuffer));
  ogg.begin();                // Initialize VS1053 player
  tckr.attach(10.0, timer10); // Start timer10 every 10 sec
  dbgprint("Waiting for telnet connection, exit with ^] and 'quit'...");
}

//******************************************************************************************
//                                   L O O P *
//******************************************************************************************
// *
//******************************************************************************************
/*
void loop() {
  uint16_t data; // Word from VS1053 fifo
  int i;         // Loop control
  int nb;        // Number of bytes in fifo
  String req;    // Request from client

  if (server.hasClient()) {
    if (!serverClient || !serverClient.connected()) {
      if (serverClient) {
        serverClient.stop();
        dbgprint("Telnet Client Stop");
      }
      serverClient = server.available();
      Serial.println("New Telnet client");
      ogg.activate();
      serverClient.write(oggheader,
                         strlen(oggheader)); // Send a dummy ogg header
    }
  }
  if (serverClient.available()) {
    serverClient.read(); // Discard data from Client
  }
  // Now handle converted data if any
  if (ogg.active()) {
    nb = ogg.available() * 2; // Data available in bytes
    if (nb > (BUFFSIZ / 10))  // Enough data available?
    {
      if (nb > BUFFSIZ) // Limit buffer size
      {
        nb = BUFFSIZ;
      }
      gconv += nb; // Save global for debugging
      for (i = 0; i < nb; i += 2) {
        data = ogg.read();            // Read from VS1053 fifo
        dbuffer[i] = data >> 8;       // MSB
        dbuffer[i + 1] = data & 0xFF; // LSB
      }
      serverClient.write((char *)&dbuffer[0], nb); // Send to client
    }
  }
  ArduinoOTA.handle(); // Check for OTA
}
*/