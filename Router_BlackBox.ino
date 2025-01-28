#define __PROG_TYPES_COMPAT__
#include <SerialPort.h>
#include <SPI.h>
#include <SdFat.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>

// port 0, 256 byte RX buffer, 0 byte TX buffer
SerialPort<0, 256, 0> NewSerial;

#define DEBUG 0

#define SD_CHIP_SELECT 10
#define LOCATION_FILE_NUMBER_LSB 0x03
#define LOCATION_FILE_NUMBER_MSB 0x04

const byte stat1 = 5;
const byte stat2 = 13;

#define ERROR_SD_INIT 3
#define ERROR_FILE_OPEN 9

SdFat sd;
long setting_uart_speed = 115200;

static char newFileName[13];

void setup(void) {
  pinMode(stat1, OUTPUT);

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();

  // Shut off unnecessary peripherals
  ADCSRA &= ~(1 << ADEN);
  ACSR = (1 << ACD);
  DIDR0 = 0x3F;
  DIDR1 = (1 << AIN1D) | (1 << AIN0D);

  power_twi_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_adc_disable();

  // Setup UART
  NewSerial.begin(setting_uart_speed);
  NewSerial.print(F("1"));

  // Setup SD & FAT
  if (!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) systemError(ERROR_SD_INIT);

  NewSerial.print(F("2"));
}

void loop(void) {
  appendFile(newLog());
}

char* newLog(void) {
  byte msb, lsb;
  unsigned int newFileNumber;

  lsb = EEPROM.read(LOCATION_FILE_NUMBER_LSB);
  msb = EEPROM.read(LOCATION_FILE_NUMBER_MSB);

  newFileNumber = (msb << 8) | lsb;

  if ((lsb == 255) && (msb == 255)) {
    newFileNumber = 0;
    EEPROM.write(LOCATION_FILE_NUMBER_LSB, 0x00);
    EEPROM.write(LOCATION_FILE_NUMBER_MSB, 0x00);
  }

  if (newFileNumber >= 65534) {
    NewSerial.print(F("!Too many logs!"));
    return nullptr;
  }

  while (1) {
    sprintf_P(newFileName, PSTR("LOG%05d.TXT"), newFileNumber);

    SdFile newFile;
    if (newFile.open(newFileName, O_CREAT | O_EXCL | O_WRITE)) break;

    if (newFile.open(newFileName, O_READ)) {
      if (newFile.fileSize() == 0) {
        newFile.close();
        return newFileName;
      }
      newFile.close();
    }

    newFileNumber++;
    if (newFileNumber >= 65534) {
      NewSerial.print(F("!Too many logs!"));
      return nullptr;
    }
  }

  newFileNumber++;
  byte newLsb = (byte)(newFileNumber & 0x00FF);
  byte newMsb = (byte)((newFileNumber & 0xFF00) >> 8);

  if (EEPROM.read(LOCATION_FILE_NUMBER_LSB) != newLsb)
    EEPROM.write(LOCATION_FILE_NUMBER_LSB, newLsb);
  if (EEPROM.read(LOCATION_FILE_NUMBER_MSB) != newMsb)
    EEPROM.write(LOCATION_FILE_NUMBER_MSB, newMsb);

  return newFileName;
}

byte appendFile(char* fileName) {
  SdFile workingFile;
  if (!workingFile.open(fileName, O_CREAT | O_APPEND | O_WRITE))
    systemError(ERROR_FILE_OPEN);

  if (workingFile.fileSize() == 0) {
    workingFile.rewind();
    workingFile.sync();
  }

  const int LOCAL_BUFF_SIZE = 64;
  byte buff[LOCAL_BUFF_SIZE];

  const unsigned int MAX_IDLE_TIME_MSEC = 500;
  unsigned long lastSyncTime = millis();

  NewSerial.print(F("<"));
  digitalWrite(stat1, HIGH);

  while (1) {
    byte charsToRecord = NewSerial.read(buff, sizeof(buff));
    if (charsToRecord > 0) {
      toggleLED(stat1);
      workingFile.write(buff, charsToRecord);
    } else if ((unsigned long)(millis() - lastSyncTime) > MAX_IDLE_TIME_MSEC) {
      workingFile.sync();
      digitalWrite(stat1, LOW);

      power_timer0_disable();
      power_spi_disable();
      sleep_mode();
      power_spi_enable();
      power_timer0_enable();

      lastSyncTime = millis();
    }
  }
  return 1;
}

void systemError(byte errorType) {
  NewSerial.print(F("Error "));
  if (errorType == ERROR_SD_INIT) {
    NewSerial.print(F("card.init"));
  } else if (errorType == ERROR_FILE_OPEN) {
    NewSerial.print(F("file.open"));
  }
  blinkError(errorType);
}

void blinkError(byte errorType) {
  while (1) {
    for (int x = 0; x < errorType; x++) {
      digitalWrite(stat1, HIGH);
      delay(200);
      digitalWrite(stat1, LOW);
      delay(200);
    }
    delay(2000);
  }
}

void toggleLED(byte pinNumber) {
  digitalWrite(pinNumber, !digitalRead(pinNumber));
}
