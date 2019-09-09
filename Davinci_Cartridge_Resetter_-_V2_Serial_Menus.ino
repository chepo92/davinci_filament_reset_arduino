/* Davinci Firmware Updater - jlaughter mod
 *  V1 - Added additional fields and cartirdge support, and updated for newer firmware (tested on 1.0 Pro v1.1.9)
 *  V2 - Serial Menus, ability to read config and selectively modify values
 */

/*
Da Vinci EEPROM update Copyright (C) 2014 by Oliver Fueckert <oliver@voltivo.com>
Increment Serial code - contributed by Matt
UNI/O Library Copyright (C) 2011 by Stephen Early <steve@greenend.org.uk>
Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:
The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.  */

/************************************************************
Pinout looking at the pads on the EEPROM board
-------------------\
|                   \
|  GND   SCIO   +5V  \
|                    | 
----------------------
 
*************************************************************/
#ifndef _NANODEUNIO_LIB_H
#define _NANODEUNIO_LIB_H

#if ARDUINO >= 100
  #include <Arduino.h> // Arduino 1.0
#else
  #include <WProgram.h> // Arduino 0022
#endif

#define NANODE_MAC_DEVICE 0xa0
#define NANODE_MAC_ADDRESS 0xfa

#define CODE 0x00 //1 Byte
#define MATERIAL 0x01 //1 Byte
#define COLOR 0x02  //2 Bytes
#define DATE 0x05  //4 Bytes
#define TOTALLEN 0x08 //4 Bytes
#define NEWLEN 0x0C //4 Bytes
#define HEADTEMP 0x10 //2 Bytes
#define BEDTEMP 0x12  //2Bytes
#define MLOC 0x14 //2 Bytes
#define DLOC 0x16 //2 Bytes
#define SN 0x18   //12 Bytes
#define CRC 0x24  //2 Bytes
#define LEN2 0x34 //4 Bytes, printer writes this 3x for a total of 12 bytes in firmware 1.1.9

class NanodeUNIO {
 private:
  byte addr;
 public:
  NanodeUNIO(byte address);

  boolean read(byte *buffer,word address,word length);
  boolean start_write(const byte *buffer,word address,word length);
  boolean enable_write(void);
  boolean disable_write(void);
  boolean read_status(byte *status);
  boolean write_status(byte status);
  boolean await_write_complete(void);
  boolean simple_write(const byte *buffer,word address,word length);
};

#endif /* _NANODEUNIO_LIB_H */

#define UNIO_STARTHEADER 0x55
#define UNIO_READ        0x03
#define UNIO_CRRD        0x06
#define UNIO_WRITE       0x6c
#define UNIO_WREN        0x96
#define UNIO_WRDI        0x91
#define UNIO_RDSR        0x05
#define UNIO_WRSR        0x6e
#define UNIO_ERAL        0x6d
#define UNIO_SETAL       0x67

#define UNIO_TSTBY 600
#define UNIO_TSS    10
#define UNIO_THDR    5
#define UNIO_QUARTER_BIT 10
#define UNIO_FUDGE_FACTOR 5

#if defined(__AVR__)
  #define UNIO_OUTPUT() do { DDRD |= 0x80; } while (0)
  #define UNIO_INPUT() do { DDRD &= 0x7f; } while (0)
#else
  #define UNIO_PIN  10
  #define UNIO_OUTPUT() pinMode(UNIO_PIN, OUTPUT)
  #define UNIO_INPUT() pinMode(UNIO_PIN, INPUT);

void sei() {
  enableInterrupts();
}
void cli() {
  disableInterrupts();
}
#endif

static void set_bus(boolean state) {
#if defined(__AVR__)
  PORTD=(PORTD&0x7f)|(!!state)<<7;
#else
  digitalWrite(UNIO_PIN, state);
#endif
}

static boolean read_bus(void) {
#if defined(__AVR__)
  return !!(PIND&0x80);
#else
  return digitalRead(UNIO_PIN);
#endif
}
static void unio_inter_command_gap(void) {
  set_bus(1);
  delayMicroseconds(UNIO_TSS+UNIO_FUDGE_FACTOR);
}

static void unio_standby_pulse(void) {
  set_bus(0);
  UNIO_OUTPUT();
  delayMicroseconds(UNIO_TSS+UNIO_FUDGE_FACTOR);
  set_bus(1);
  delayMicroseconds(UNIO_TSTBY+UNIO_FUDGE_FACTOR);
}

static volatile boolean rwbit(boolean w) {
  boolean a,b;
  set_bus(!w);
  delayMicroseconds(UNIO_QUARTER_BIT);
  a=read_bus();
  delayMicroseconds(UNIO_QUARTER_BIT);
  set_bus(w);
  delayMicroseconds(UNIO_QUARTER_BIT);
  b=read_bus();
  delayMicroseconds(UNIO_QUARTER_BIT);
  return b&&!a;
}

static boolean read_bit(void) {
  boolean b;
  UNIO_INPUT();
  b=rwbit(1);
  UNIO_OUTPUT();
  return b;
}

static boolean send_byte(byte b, boolean mak) {
  for (int i=0; i<8; i++) {
    rwbit(b&0x80);
    b<<=1;
  }
  rwbit(mak);
  return read_bit();
}

static boolean read_byte(byte *b, boolean mak) {
  byte data=0;
  UNIO_INPUT();
  for (int i=0; i<8; i++) {
    data = (data << 1) | rwbit(1);
  }
  UNIO_OUTPUT();
  *b=data;
  rwbit(mak);
  return read_bit();
}

static boolean unio_send(const byte *data,word length,boolean end) {
  for (word i=0; i<length; i++) {
    if (!send_byte(data[i],!(((i+1)==length) && end))) return false;
  }
  return true;
}

static boolean unio_read(byte *data,word length)  {
  for (word i=0; i<length; i++) {
    if (!read_byte(data+i,!((i+1)==length))) return false;
  }
  return true;
}

static void unio_start_header(void) {
  set_bus(0);
  delayMicroseconds(UNIO_THDR+UNIO_FUDGE_FACTOR);
  send_byte(UNIO_STARTHEADER,true);
}

NanodeUNIO::NanodeUNIO(byte address) {
  addr=address;
}

#define fail() do { sei(); return false; } while (0)

boolean NanodeUNIO::read(byte *buffer,word address,word length) {
  byte cmd[4];
  cmd[0]=addr;
  cmd[1]=UNIO_READ;
  cmd[2]=(byte)(address>>8);
  cmd[3]=(byte)(address&0xff);
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,4,false)) fail();
  if (!unio_read(buffer,length)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::start_write(const byte *buffer,word address,word length) {
  byte cmd[4];
  if (((address&0x0f)+length)>16) return false; // would cross page boundary
  cmd[0]=addr;
  cmd[1]=UNIO_WRITE;
  cmd[2]=(byte)(address>>8);
  cmd[3]=(byte)(address&0xff);
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,4,false)) fail();
  if (!unio_send(buffer,length,true)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::enable_write(void) {
  byte cmd[2];
  cmd[0]=addr;
  cmd[1]=UNIO_WREN;
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,2,true)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::disable_write(void) {
  byte cmd[2];
  cmd[0]=addr;
  cmd[1]=UNIO_WRDI;
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,2,true)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::read_status(byte *status) {
  byte cmd[2];
  cmd[0]=addr;
  cmd[1]=UNIO_RDSR;
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,2,false)) fail();
  if (!unio_read(status,1)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::write_status(byte status) {
  byte cmd[3];
  cmd[0]=addr;
  cmd[1]=UNIO_WRSR;
  cmd[2]=status;
  unio_standby_pulse();
  cli();
  unio_start_header();
  if (!unio_send(cmd,3,true)) fail();
  sei();
  return true;
}

boolean NanodeUNIO::await_write_complete(void) {
  byte cmd[2];
  byte status;
  cmd[0]=addr;
  cmd[1]=UNIO_RDSR;
  unio_standby_pulse();
  do {
    unio_inter_command_gap();
    cli();
    unio_start_header();
    if (!unio_send(cmd,2,false)) fail();
    if (!unio_read(&status,1)) fail();
    sei();
  } while (status&0x01);
  return true;
}

boolean NanodeUNIO::simple_write(const byte *buffer,word address,word length) {
  word wlen;
  while (length>0) {
    wlen=length;
    if (((address&0x0f)+wlen)>16) {
      wlen=16-(address&0x0f);
    }
    if (!enable_write()) return false;
    if (!start_write(buffer,address,wlen)) return false;
    if (!await_write_complete()) return false;
    buffer+=wlen;
    address+=wlen;
    length-=wlen;
  }
  return true;
}

static void status(boolean r)
{
  if (r) Serial.println(F("(success)"));
  else Serial.println(F("(failure)"));
}

static void dump_eeprom(byte *buf,word address,word length)
{
//  byte buf[128];
  char lbuf[80];
  char *x;
  int i,j;

  NanodeUNIO unio(NANODE_MAC_DEVICE);
  
  memset(buf,0,128);
  status(unio.read(buf,address,length));
  
  for (i=0; i<128; i+=16) {
    x=lbuf;
    sprintf(x,"%02X: ",i);
    x+=4;
    for (j=0; j<16; j++) {
      sprintf(x,"%02X",buf[i+j]);
      x+=2;
    }
    *x=32;
    x+=1;
    for (j=0; j<16; j++) {
      if (buf[i+j]>=32 && buf[i+j]<127) *x=buf[i+j];
      else *x=46;
      x++;
    }
    *x=0;
    Serial.println(lbuf);
  }
}

int led = LED_BUILTIN;
byte sr;
NanodeUNIO unio(NANODE_MAC_DEVICE);
// define and clear the contents buffer
byte contents[128];

// Set global defaults for editable values and clear global edit flags
//Materials
boolean editflag_mt = false;
unsigned char edit_mt[] = {0x00}; //default null

// color
boolean editflag_fcolor = false;
unsigned char edit_fcolor[] = {0x00,0x00}; //default null

// extruder temp
boolean editflag_et = false;
unsigned char edit_et[] = {0x00,0x00}; //default null

// bed temp
boolean editflag_bt = false;
unsigned char edit_bt[] = {0x00,0x00}; //default null

// Serial Number
boolean editflag_sn = false;
unsigned char edit_sn[13] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,'\0'};

// Total Cartridge Capacity
boolean editflag_fcap = false;
// little endian format, in mm (200000 = 200m)
unsigned char edit_fcap[] = {0x00,0x00,0x00,0x00}; //default null

// Remaining Filament Length
boolean editflag_frem = false;
// little endian format, in mm (200000 = 200m)
unsigned char edit_frem[] = {0x00,0x00,0x00,0x00}; //default null

// misc globals
char response[25];
int count_edits = 0; //number of edited values found

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  while(!Serial);
  delay(250);

  //clear the contents buffer
  memset(contents,0,128);
}

String decode_material(int materialcode) {
  // decode a material value and return a String for the material name
  // appears that they use ascii letters for their entries, 0x41 = A = ABS
  //  41-A ABS  46-F FLEX  47-G PETG  50-P PLA    51-Q PLA
  //  53-S ASA  54-T TOUGH PLA  55-U UVCR   56-V WATER SOLUBLE
  //  59-Y NYLON    
  switch (materialcode) {
    case 0x41:   
      return F("ABS");
      break;
    case 0x46:   
      return F("FLEX");
      break;
    case 0x47:   
      return F("PETG");
      break;
    case 0x50:   
      return F("PLA");
      break;
    case 0x51:   
      return F("PLA");
      break;
    case 0x53:   
      return F("ASA");
      break;
    case 0x54:   
      return F("TOUGH PLA");
      break;
    case 0x55:   
      return F("UVCR");
      break;
    case 0x56:   
      return F("WATER SOLUBLE");
      break;
    case 0x59:   
      return F("NYLON");
      break;
    default:
      return F("Unknown Material");
      break; 
    }
} // decode_material

String decode_color(int colorcode) {
  // decode a color value and return a String for the color name
  // 31 GREY
  // 32 CLEAR RED      33 CLEAR            34 BOTTLE GREEN     35 NEON MAGENTA
  // 36 STEEL BLUE     37 SUN ORANGE       41 PURPLE           42 BLUE
  // 43 NEON TANGERINE 44 VIRDITY          45 OLIVINE          46 GOLD
  // 47 GREEN          48 NEON GREEN       49 SNOW WHITE       4A NEON YELLOW
  // 4B BLACK          4C VIOLET           4D GRAPE PURPLE     4E PURPURIN
  // 4F CLEAR YELLOW   50 CLEAR GREEN      51 CLEAR TANGERINE  52 RED
  // 53 CYBER YELLOW   54 TANGERINE        55 CLEAR BLUE       56 CLEAR PURPLE
  // 57 WHITE          58 CLEAR MAGENTA    59 YELLOW           5A NATURE

  switch(colorcode) {
    case 0x31:   
      return F("GREY");
      break;
    case 0x32:   
      return F("CLEAR RED");
      break;
    case 0x33:   
      return F("CLEAR");
      break;
    case 0x34:   
      return F("BOTTLE GREEN");
      break;
    case 0x35:   
      return F("NEON MAGENTA");
      break;
    case 0x36:   
      return F("STEEL BLUE");
      break;
    case 0x37:   
      return F("SUN ORANGE");
      break;
    case 0x38:   
      return F("PEARL WHITE");
      break;
    case 0x41:   
      return F("PURPLE");
      break;
    case 0x42:   
      return F("BLUE");
      break;
    case 0x43:   
      return F("NEON TANGERINE");
      break;
    case 0x44:   
      return F("VIRDITY");
      break;
    case 0x45:   
      return F("OLIVINE");
      break;
    case 0x46:   
      return F("GOLD");
      break;
    case 0x47:   
      return F("GREEN");
      break;
    case 0x48:   
      return F("NEON GREEN");
      break;
    case 0x49:   
      return F("SNOW WHITE");
      break;
    case 0x4A:   
      return F("NEON YELLOW");
      break;
    case 0x4B:   
      return F("BLACK");
      break;
    case 0x4C:   
      return F("VIOLET");
      break;
    case 0x4D:   
      return F("GRAPE PURPLE");
      break;
    case 0x4E:   
      return F("PURPURIN");
      break;
    case 0x4F:   
      return F("CLEAR YELLOW");
      break;
    case 0x50:   
      return F("CLEAR GREEN");
      break;
    case 0x51:   
      return F("CLEAR TANGERINE");
      break;
    case 0x52:   
      return F("RED");
      break;
    case 0x53:   
      return F("CYBER YELLOW");
      break;
    case 0x54:   
      return F("TANGERINE");
      break;
    case 0x55:   
      return F("CLEAR BLUE");
      break;
    case 0x56:   
      return F("CLEAR PURPLE");
      break;
    case 0x57:   
      return F("WHITE");
      break;
    case 0x58:   
      return F("CLEAR MAGENTA");
      break;
    case 0x59:   
      return F("YELLOW");
      break;
    case 0x5A:   
      return F("NATURE");
      break;
    default:
      return F("Undefined Color");
      break; 
  }
}

void display_current_values() {
    int i = 0;
    Serial.println(F("Current Values"));
    Serial.print(F("  Material: 0x"));
    Serial.print(contents[MATERIAL] < 16 ? "0" : "");
    Serial.print(contents[MATERIAL], HEX);
    Serial.print(F(" "));
    Serial.print(decode_material(contents[MATERIAL]));   
    Serial.println();
    
    Serial.print(F("  Color: 0x"));
    Serial.print(contents[COLOR] < 16 ? "0" : "");
    Serial.print(contents[COLOR], HEX);
    Serial.print(F(" "));
    Serial.print(decode_color(contents[COLOR]));   
    Serial.println();
    
    Serial.print(F("  Date: 0x"));
    for (i=3; i>=0; i--) {
      Serial.print(contents[DATE+i] < 16 ? "0" : "");
      Serial.print(contents[DATE+i], HEX);
    }
    Serial.println();
    
    Serial.print(F("  Extruder Temp: 0x"));
    for (i=1; i>=0; i--) {
      Serial.print(contents[HEADTEMP+i] < 16 ? "0" : "");
      Serial.print(contents[HEADTEMP+i], HEX);
    }
    Serial.print(F(" "));
    Serial.print(contents[HEADTEMP] + (uint32_t)(contents[HEADTEMP+1] << 8), DEC);
    Serial.print(F("°C"));
    Serial.println();
        
    Serial.print(F("  Bed Temp: 0x"));
    for (i=1; i>=0; i--) {
    Serial.print(contents[BEDTEMP+i] < 16 ? "0" : "");
    Serial.print(contents[BEDTEMP+i], HEX);
    }
    Serial.print(F(" "));
    Serial.print(contents[BEDTEMP] + (uint32_t)(contents[BEDTEMP+1] << 8), DEC);
    Serial.print(F("°C"));
    Serial.println();
    
    Serial.print(F("  Serial Number: 0x"));
    for (i=11; i>=0; i--) {
    Serial.print(contents[SN+i] < 16 ? "0" : "");
    Serial.print(contents[SN+i], HEX);
    }
    Serial.println();

    Serial.print(F("  Filament Capacity: 0x"));
    for (i=3; i>=0; i--) {
      Serial.print(contents[TOTALLEN+i] < 16 ? "0" : "");
      Serial.print(contents[TOTALLEN+i], HEX);
    }
    Serial.print(F(" "));
    Serial.print((((uint32_t)contents[TOTALLEN]) + ((uint32_t)contents[TOTALLEN+1] << 8) + ((uint32_t)contents[TOTALLEN+2] << 16) + ((uint32_t)contents[TOTALLEN+3] << 24))/1000.0, 2);
    Serial.print(F("M")); 
    Serial.println();
    
    Serial.print(F("  Filament Remaining: 0x"));
    for (i=3; i>=0; i--) {
      Serial.print(contents[LEN2+i] < 16 ? "0" : "");
      Serial.print(contents[LEN2+i], HEX);
    }
    Serial.print(F(" "));
    Serial.print((((uint32_t)contents[LEN2]) + ((uint32_t)contents[LEN2+1] << 8) + ((uint32_t)contents[LEN2+2] << 16) + ((uint32_t)contents[LEN2+3] << 24))/1000.0, 2);
    Serial.print(F("M"));   
    Serial.println();  
    Serial.println();
}

void display_edited_values() {
  int i = 0;
  count_edits = 0; // reset the edit counter
    if (editflag_mt) {
      count_edits++;
      Serial.print(F("  Edited Material: 0x"));
      Serial.print(edit_mt[0] < 16 ? "0" : "");
      Serial.print(edit_mt[0], HEX);
      Serial.print(F(" "));
      Serial.print(decode_material(edit_mt[0]));   
      Serial.println();
      } // if editflag
      
    if (editflag_fcolor) {
      count_edits++;
      Serial.print(F("  Edited Color: 0x"));
      Serial.print(edit_fcolor[0] < 16 ? "0" : "");
      Serial.print(edit_fcolor[0], HEX);
      Serial.print(F(" "));
      Serial.print(decode_color(edit_fcolor[0]));   
      Serial.println();
      } // if editflag
      
    if (editflag_et) {
      count_edits++;
      Serial.print("  Edited Extruder Temp: 0x");
      Serial.print(edit_et[1] < 16 ? "0" : "");
      Serial.print(edit_et[1], HEX);
      Serial.print(edit_et[0] < 16 ? "0" : "");
      Serial.print(edit_et[0], HEX);
      Serial.print(F(" "));
      Serial.print(edit_et[0] + (uint32_t)(edit_et[1] << 8), DEC);
      Serial.print(F("°C"));
      Serial.println();
      } // if editflag
          
    if (editflag_bt) {
      count_edits++;
      Serial.print(F("  Edited Bed Temp: 0x"));
      Serial.print(edit_bt[1] < 16 ? "0" : "");
      Serial.print(edit_bt[1], HEX);
      Serial.print(edit_bt[0] < 16 ? "0" : "");
      Serial.print(edit_bt[0], HEX);
      Serial.print(F(" "));
      Serial.print(edit_bt[0] + (uint32_t)(edit_bt[1] << 8), DEC);
      Serial.print(F("°C"));
      Serial.println();
    } // if editflag
    
    if (editflag_sn) {
      count_edits++;
      Serial.print(F("  Edited Serial Number: 0x"));
      for (i=11; i>=0; i--) {
      Serial.print(edit_sn[i] < 16 ? "0" : "");
      Serial.print(edit_sn[i], HEX);
      }
      Serial.println();
      } // if editflag
      
    if (editflag_fcap) {
      count_edits++;
      Serial.print(F("  Edited Filament Capacity: 0x"));
      for (i=3; i>=0; i--) {
      Serial.print(edit_fcap[i] < 16 ? "0" : "");
      Serial.print(edit_fcap[i], HEX);
      }
      Serial.print(F(" "));
    
      Serial.print((((uint32_t)edit_fcap[0]) + ((uint32_t)edit_fcap[1] << 8) + ((uint32_t)edit_fcap[2] << 16) + ((uint32_t)edit_fcap[3] << 24))/1000.0, 2);
      Serial.print(F("M")); 
      Serial.println(); 
      } // if editflag
      
    if (editflag_frem) {
      count_edits++;
      Serial.print(F("  Edited Filament Remaining: 0x"));
      for (i=3; i>=0; i--) {
      Serial.print(edit_frem[i] < 16 ? "0" : "");
      Serial.print(edit_frem[i], HEX);
      }
      Serial.print(F(" "));
      Serial.print((((uint32_t)edit_frem[0]) + ((uint32_t)edit_frem[1] << 8) + ((uint32_t)edit_frem[2] << 16) + ((uint32_t)edit_frem[3] << 24))/1000.0, 2);
      Serial.print(F("M"));   
      Serial.println();
      } // if editflag
}

void read_chip() {
  //loop looking for a chip for 15 seconds
  int i = 0;
  Serial.println();
  Serial.print(F("Looking for Da Vinci EEPROM CHIP (15 sec)"));    
    do {
    Serial.print(".");
    digitalWrite(led, LOW); 
    delay(500);
    digitalWrite(led, HIGH);
    ++i;
  } while(!unio.read_status(&sr) && i < 30);

  if (i<30) {
    Serial.println();
    Serial.println(F("Da Vinci EEPROM found..."));
    Serial.println(F("Reading the Davinci EEPROM Contents..."));
    dump_eeprom(contents,0,128);
    Serial.println();    
    display_current_values();
    }
    else {
    Serial.println();
    Serial.println(F("No EEPROM chip found before timeout."));
    Serial.println();
  }
}

void view_edit_chip() {
  int i = 0;
  int j = 0;
  boolean boolExit = false;
  uint32_t temp_ul = 0; //used to parse input
  char composedByte[2];
  
  while(!boolExit) {
    Serial.println();
    Serial.println(F("View/Edit Cartridge Data"));
    Serial.println(F("================================"));
    display_current_values();
    display_edited_values();
      
    // Display the View/Edit Menu
    // flush the serial buffer
    while(Serial.available()){
      delay(100);
      Serial.read();
    }
    
    // Display the menu
    Serial.println();
    Serial.println(F("1. Edit Material \t 2. Edit Color \t 3. Edit Extruder Temp \t 4. Edit Bed Temp"));
    Serial.println(F("5. Edit Serial Number \t 6. Edit Filament Capacity \t 7. Edit Filament Remaining"));
    Serial.println(F("8. Clear all edits"));
    Serial.println(F("Choose an item to edit or 0 to exit:"));
    Serial.println();
  
    // Get the menu input
    serial_getline(); 

    switch (response[0]) {
      case '0':
         // Exit selected
         boolExit = true;
         break;

      case '1':
        //edit material.  single byte value
        if (!editflag_mt) {
          Serial.println();
          Serial.println(F("No edited value for material is currently set."));
          }
        Serial.println(F("Options: 41 = ABS, 46 = Flex, 47 = PETG, 50 = PLA, 51 = PLA"));
        Serial.println(F("Options: 53 = ASA, 54 = Tough PLA, 55 = UVCR, 56 = Water Soluble"));
        Serial.println(F("Options: 59 = Nylon"));
        Serial.println(F("Enter a new value for material (1 byte, Hex): "));

        serial_getline();
        temp_ul = strtoul(response,0,16);
        if (temp_ul > 0 and temp_ul <= 255) {
          Serial.print(F("Parsed hex: "));
          Serial.println(temp_ul, HEX);
          edit_mt[0] = (int)temp_ul;
          editflag_mt = true;
        }
        else {
          Serial.print(F("Invalid Value."));
        }
        Serial.println();
        break;
        
      case '2':
        //edit color, two bytes but upper (1) is always 0
        if (!editflag_fcolor) {
          Serial.println();
          Serial.println(F("No edited value for color is currently set."));
          }
        Serial.println(F("Options: "));
        Serial.println(F(" 31 GREY           32 CLEAR RED        33 CLEAR"));
        Serial.println(F(" 34 BOTTLE GREEN   35 NEON MAGENTA     36 STEEL BLUE"));
        Serial.println(F(" 37 SUN ORANGE     38 PEARL WHITE      41 PURPLE           42 BLUE"));
        Serial.println(F(" 43 NEON TANGERINE 44 VIRDITY          45 OLIVINE          46 GOLD"));
        Serial.println(F(" 47 GREEN          48 NEON GREEN       49 SNOW WHITE       4A NEON YELLOW"));
        Serial.println(F(" 4B BLACK          4C VIOLET           4D GRAPE PURPLE     4E PURPURIN"));
        Serial.println(F(" 4F CLEAR YELLOW   50 CLEAR GREEN      51 CLEAR TANGERINE  52 RED"));
        Serial.println(F(" 53 CYBER YELLOW   54 TANGERINE        55 CLEAR BLUE       56 CLEAR PURPLE"));
        Serial.println(F(" 57 WHITE          58 CLEAR MAGENTA    59 YELLOW           5A NATURE"));
        
        Serial.println(F("Enter a new value for color (1 byte, Hex): "));

        serial_getline();
        temp_ul = strtoul(response,0,16);
        if (temp_ul > 0 and temp_ul <= 255) {
          Serial.print(F("Parsed hex: "));
          Serial.println(temp_ul, HEX);
          edit_fcolor[0] = (int)temp_ul;
          edit_fcolor[1]=0; //always 0
          editflag_fcolor = true;
        }
        else {
          Serial.print(F("Invalid Value."));
        }
        Serial.println();
        break;
        
      case '3':
        //edit extruder temp, 2 bytes
        if (!editflag_et) {
          Serial.println();
          Serial.println(F("No edited value for extruder temp is currently set."));
          }
        Serial.println(F("Enter a new extruder temp value from 40 to 260 (°C): "));
        
        serial_getline();
        temp_ul = strtoul(response,0,10);
        if (temp_ul >= 40 and temp_ul <= 260) {
          Serial.print(F("Parsed hex: "));
          Serial.println(temp_ul, HEX);
          edit_et[0] = temp_ul & 0xff;
          edit_et[1] = (temp_ul >> 8) & 0xff;
          editflag_et = true;
        }
        else {
          Serial.print(F("Invalid Value."));
        }
        Serial.println();
        break;
        
      case '4':
        //edit bed temp, 2 bytes
        if (!editflag_bt) {
          Serial.println();
          Serial.println(F("No edited value for bed temp is currently set."));
          }
        Serial.println(F("Enter a new bed temp value from 40 to 100 (°C): "));
        
        serial_getline();
        temp_ul = strtoul(response,0,10);
        if (temp_ul >= 40 and temp_ul <= 100) {
          Serial.print(F("Parsed hex: "));
          Serial.println(temp_ul, HEX);
          edit_bt[0] = temp_ul & 0xff;
          edit_bt[1] = (temp_ul >> 8) & 0xff;
          editflag_bt = true;
        }
        else {
          Serial.print(F("Invalid Value."));
        }
        Serial.println();
        break;
        
      case '5':
        //edit serial number temp, 2 bytes
        if (!editflag_sn) {
          Serial.println();
          Serial.println(F("No edited value for serial number is currently set."));
          }
        Serial.println(F("Enter a new serial number (12 bytes, hex): "));
        
        serial_getline();
        Serial.print(F("Parsed hex: "));
        j = 0;
        for (i=11; i>=0; i--) {
          composedByte[0] = response[j];
          composedByte[1] = response[j+1];
          edit_sn[i] = strtoul(composedByte,0,16);
           Serial.print(edit_sn[i], HEX);
           j+=2;
        }
        Serial.println();
        editflag_sn = true;
        Serial.println();
        break;
        
      case '6':
        //edit filament capacity, 4 bytes
        if (!editflag_fcap) {
          Serial.println();
          Serial.println(F("No edited value for filament capacity is currently set."));
          }
        Serial.println(F("Enter a new filament cartridge capacity value (mm): "));
        serial_getline();
        temp_ul = strtoul(response,0,10) & 0xffffffff;
        Serial.print(F("Parsed hex: "));
          Serial.println(temp_ul, HEX);
          edit_fcap[0] = temp_ul & 0xff;
          edit_fcap[1] = (temp_ul >> 8) & 0xff;
          edit_fcap[2] = (temp_ul >> 16) & 0xff;
          edit_fcap[3] = (temp_ul >> 24) & 0xff;
          editflag_fcap = true;
        Serial.println();
        break;
        
      case '7':
        //edit filament remaining, 4 bytes
        if (!editflag_frem) {
          Serial.println();
          Serial.println(F("No edited value for filament remaining is currently set."));
          }
        Serial.println(F("Enter a new filament remaining value (mm): "));
        serial_getline();
        temp_ul = strtoul(response,0,10) & 0xffffffff;
        Serial.print(F("Parsed hex: "));
          Serial.println(temp_ul, HEX);
          edit_frem[0] = temp_ul & 0xff;
          edit_frem[1] = (temp_ul >> 8) & 0xff;
          edit_frem[2] = (temp_ul >> 16) & 0xff;
          edit_frem[3] = (temp_ul >> 24) & 0xff;
          editflag_frem = true;
        Serial.println();
        break;
        
      case '8':
        //clear all edits
        memset(edit_mt, 0, sizeof(edit_mt));
        editflag_mt=false;
        memset(edit_fcolor, 0, sizeof(edit_fcolor));
        editflag_fcolor=false;
        memset(edit_et, 0, sizeof(edit_et));
        editflag_et=false;
        memset(edit_bt, 0, sizeof(edit_bt));
        editflag_bt=false;
        memset(edit_sn, 0, sizeof(edit_sn));
        editflag_sn=false;
        memset(edit_fcap, 0, sizeof(edit_fcap));
        editflag_fcap=false;
        memset(edit_frem, 0, sizeof(edit_frem));
        editflag_frem=false;
        Serial.println(F("Edits cleared."));
        Serial.println();
        break;
        
      default:
        Serial.println(F("Invalid Option."));
      } // end case    
  } //while boolExit = false
} // view_edit_chip()

void write_chip() {
  int i=0;
  byte contents[128];

  Serial.println();
  Serial.println(F("Write Cartridge Data"));
  Serial.println(F("================================"));

  display_edited_values();

  if (count_edits>0) {
    //loop looking for a chip for 15 seconds
    i = 0;
    Serial.println();
    Serial.print(F("Looking for Da Vinci EEPROM CHIP (15 sec)"));    
    do {
      Serial.print(".");
      digitalWrite(led, LOW); 
      delay(500);
      digitalWrite(led, HIGH);
      ++i;
    } while(!unio.read_status(&sr) && i < 30);
  
    if (i<30) {
      Serial.println();
      Serial.println(F("Da Vinci EEPROM found..."));
      Serial.println(F("Updating EEPROM..."));
  
      if (editflag_mt) {
        status(unio.simple_write((const byte *)edit_mt,MATERIAL,1)); // Material
        status(unio.simple_write((const byte *)edit_mt,64 + MATERIAL,1)); // Material
        }
    
      if (editflag_fcolor) {
        status(unio.simple_write((const byte *)edit_fcolor,COLOR,1)); // Color
        status(unio.simple_write((const byte *)edit_fcolor,64 + COLOR,1)); // Color
        }
    
      if (editflag_et) {
        status(unio.simple_write((const byte *)edit_et,HEADTEMP,2)); // extruder temp
        status(unio.simple_write((const byte *)edit_et,64 + HEADTEMP,2)); // extruder temp
        }
    
      if (editflag_bt) {
        status(unio.simple_write((const byte *)edit_bt,BEDTEMP,2)); // bed temp
        status(unio.simple_write((const byte *)edit_bt,64 + BEDTEMP,2)); // bed temp
        }
    
      if (editflag_sn) {
        status(unio.simple_write((const byte *)edit_sn,SN,12)); //Serial Number
        status(unio.simple_write((const byte *)edit_sn,64 + SN,12)); //Serial Number
        }
    
      if (editflag_fcap) {
        status(unio.simple_write((const byte *)edit_fcap,TOTALLEN,4)); // Cartridge capacity
        status(unio.simple_write((const byte *)edit_fcap,NEWLEN,4)); // Cartridge capacity second copy
        status(unio.simple_write((const byte *)edit_fcap,64 + TOTALLEN,4)); // Cartridge capacity
        status(unio.simple_write((const byte *)edit_fcap,64 + NEWLEN,4)); // Cartridge capacity second copy
        }
    
      if (editflag_frem) {
        status(unio.simple_write((const byte *)edit_frem,LEN2,4)); //remaining length
        status(unio.simple_write((const byte *)edit_frem,LEN2 + 4,4)); //remaining length second copy, required for firmware v1.1.9
        status(unio.simple_write((const byte *)edit_frem,LEN2 + 8,4)); //remaining length third copy, required for firmware v1.1.9
        status(unio.simple_write((const byte *)edit_frem,64 + LEN2,4)); //remaining length
        status(unio.simple_write((const byte *)edit_frem,64 + LEN2 + 4,4)); //remaining length second copy, required for firmware v1.1.9
        status(unio.simple_write((const byte *)edit_frem,64 + LEN2 + 8,4)); //remaining length third copy, required for firmware v1.1.9
        }  
      Serial.println(F("Dumping Content after modification..."));  
      dump_eeprom(contents,0,128);
      Serial.println();
      } //not timed out
  else {
    Serial.println();
    Serial.println(F("No EEPROM chip found before timeout."));
    Serial.println();
    } // timeout
  } //edits > 0
  else { // no edits to write
      Serial.println(F("No edits to write."));
      Serial.println();
  }
} // write cartridge

void serial_getline() {
  char received;
  int position = 0;
  memset(response,0,25);
  
  while (received != '\n') {
    while (Serial.available() > 0) {
        received = Serial.read();
        response[position] = received;
        if (position < 24)position++;
    }
  }
  response[position] = '\0'; // null terminate
} //serial_getline()

void loop() {
// flush the serial buffer
  while(Serial.available()){
    delay(100);
    Serial.read();
    }

// Display the menu
  Serial.println();
  Serial.println(F("Davinci Cartridge EEPROM Utility"));
  Serial.println(F("================================"));
  Serial.println(F("1. Read EEPROM"));
  Serial.println(F("2. View/Edit Cartridge Data"));
  Serial.println(F("3. Write Cartridge Data"));
  Serial.println();
  Serial.print("Choose an Option:");
  
  // get the input
  serial_getline();
  Serial.println();

  switch (response[0]) {
    case '1':
      read_chip();
      break;
    case '2':
      view_edit_chip();
      break;
    case '3':
      write_chip();
      break;
    default:
      Serial.println(F("Invalid Option."));
    } // end case  
  } // end loop
