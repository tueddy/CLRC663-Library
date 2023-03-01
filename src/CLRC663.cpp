/*
 *
 * CLRC633 reader class
 * 
 * features:
 *  - can connected via SPI or I2C
 *  - read ISO-14443 UID's
 *  - read ISO-18693 UID's
 *  - support for ICode-SLIX password protected tags (privacy mode)
 *  - LPCD - low power card detection
 * 
 *  Copyright (c) 2023 tueddy (Dirk Carstensen) 
 *  
 *  Some header declarations and functions taken and extended from
 *  https://github.com/kenny5660/CLRC663.git
 */

#include "CLRC663.h"



// SPI constructor
CLRC663::CLRC663(SPIClass *SPI, int8_t cs, int8_t irq) {
  // set the transport
  _transport = MFRC630_TRANSPORT_SPI;
  // set SPI
  _spi = SPI;
  // Set the CS/SSEL pin 
  _cs = cs;
  pinMode(_cs, OUTPUT);
  // set IRQ pin for LPCD
  _irq = irq;
  if (_irq >= 0) {
    pinMode(_irq, INPUT);
  }  
  /* Disable I2C access */
  _wire = NULL;
  _i2c_addr = 0;
}


// I2C constructor
CLRC663::CLRC663(uint8_t i2c_addr, int8_t irq) {
  // Set the transport 
  _transport = MFRC630_TRANSPORT_I2C;
  // Set the I2C address 
  _i2c_addr = i2c_addr;
  // Set the I2C bus instance 
  _wire = &Wire;
  // set IRQ pin for LPCD
  _irq = irq;
  if (_irq >= 0) {
    pinMode(_irq, INPUT);
  }  
  // disable SPI
  _spi = NULL;
  _cs = -1;
}

// begin communication
void CLRC663::begin() {
    if (_transport == MFRC630_TRANSPORT_SPI) {
        // SPI
        _spi->begin();
    } else {
        // I2C
        byte pinSDA= SDA;
        byte pinSCL= SCL;
        _wire->begin(pinSDA, pinSCL);
    }
}

void CLRC663::begin(int sda, int scl) {
    // I2C
    byte pinSDA= sda;
    byte pinSCL= scl;
    _wire->begin(pinSDA, pinSCL);
}

void CLRC663::end() {
    if (_transport == MFRC630_TRANSPORT_SPI) {
        // SPI
        _spi->end();
    } else {
        // I2C
        _wire->end();
    }
}

// ---------------------------------------------------------------------------
// Register interaction functions.
// ---------------------------------------------------------------------------
uint8_t CLRC663::read_reg(uint8_t reg) {
  if (_transport == MFRC630_TRANSPORT_SPI) {
    // SPI transport
    uint8_t instruction_tx[2] = {uint8_t((reg << 1) | 0x01), 0};
    uint8_t instruction_rx[2] = {0};
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
    digitalWrite(_cs, LOW);
    instruction_rx[0] = _spi->transfer(instruction_tx[0]);
    instruction_rx[1] = _spi->transfer(instruction_tx[1]);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();    // release the SPI bus
    return instruction_rx[1];  // the second byte the returned value.
  } else {
    // I2C transport
    _wire->beginTransmission(_i2c_addr);
    _wire->write(reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
    return _wire->read();
  }
}


void CLRC663::write_reg(uint8_t reg, uint8_t value) {
  if (_transport == MFRC630_TRANSPORT_SPI) {
    // SPI transport
    uint8_t instruction_tx[2] = {uint8_t((reg << 1) | 0x00), value};
    uint8_t discard[2] = {0};
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
    digitalWrite(_cs, LOW);
    discard[0] = _spi->transfer(instruction_tx[0]);
    discard[1] = _spi->transfer(instruction_tx[1]);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();    // release the SPI bus
  } else {
    // I2C transport
    _wire->beginTransmission(_i2c_addr);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
  }
}




void CLRC663::write_regs(uint8_t reg, const uint8_t* values, uint8_t len) {
  uint8_t instruction_tx[len+1];
  uint8_t discard[len+1];
  instruction_tx[0] = (reg << 1) | 0x00;
  uint8_t i;
  for (i=0 ; i < len; i++) {
    instruction_tx[i+1] = values[i];
  }
  if (_transport == MFRC630_TRANSPORT_SPI) {
    // SPI transport
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
    digitalWrite(_cs, LOW);
    for (uint8_t i=0; i < (len+1); i++){
      discard[i] = _spi->transfer(instruction_tx[i]);
    }
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();    // release the SPI bus
  } else {
    // I2C transport
    _wire->beginTransmission(_i2c_addr);
    _wire->write(reg);
    for (i=0 ; i < len; i++) {
      _wire->write(values[i]);
    }
    _wire->endTransmission();
  }
}

void CLRC663::write_fifo(const uint8_t* data, uint16_t len) {
  uint8_t i;
  for (i=0 ; i < len; i++) {
      write_reg(MFRC630_REG_FIFODATA, data[i]);
  }
}

void CLRC663::read_fifo(uint8_t* rx, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
      rx[i] = read_reg(MFRC630_REG_FIFODATA);
  }    
}

void CLRC663::cmd_load_reg(uint16_t address, uint8_t regaddr, uint16_t length) {
  uint8_t parameters[4] = {uint8_t((uint8_t) (address >> 8)), uint8_t((uint8_t) (address & 0xFF)), regaddr, uint8_t(length)};
  flush_fifo();
  write_fifo(parameters, 4);
  write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_LOADREG);
}



void CLRC663::cmd_load_protocol(uint8_t rx, uint8_t tx) {
  uint8_t parameters[2] = {rx, tx};
  flush_fifo();
  write_fifo(parameters, 2);
  write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_LOADPROTOCOL);
}

void CLRC663::cmd_transceive(const uint8_t* data, uint16_t len) {
  cmd_idle();
  flush_fifo();
  write_fifo(data, len);
  write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_TRANSCEIVE);
}

void CLRC663::cmd_idle() {
  write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_IDLE);
}

// ---------------------------------------------------------------------------
// Utility functions.
// ---------------------------------------------------------------------------

void CLRC663::flush_fifo() {
  write_reg(MFRC630_REG_FIFOCONTROL, 1<<4);
}

uint16_t CLRC663::fifo_length() {
  // should do 512 byte fifo handling here.
  return read_reg(MFRC630_REG_FIFOLENGTH);
}

void CLRC663::clear_irq0() {
  write_reg(MFRC630_REG_IRQ0, (uint8_t) ~(1<<7));
}
void CLRC663::clear_irq1() {
  write_reg(MFRC630_REG_IRQ1, (uint8_t) ~(1<<7));
}
uint8_t CLRC663::get_irq0() {
  return read_reg(MFRC630_REG_IRQ0);
}
uint8_t CLRC663::get_irq1() {
  return read_reg(MFRC630_REG_IRQ1);
}

void CLRC663::print_block(const uint8_t* data, uint16_t len) {
  uint16_t i;
  for (i=0; i < len; i++) {
    MFRC630_PRINTF("%02X ", data[i]);
  }
}

// ---------------------------------------------------------------------------
// Timer functions
// ---------------------------------------------------------------------------
void CLRC663::activate_timer(uint8_t timer, uint8_t active) {
  write_reg(MFRC630_REG_TCONTROL, ((active << timer) << 4) | (1 << timer));
}

void CLRC663::timer_set_control(uint8_t timer, uint8_t value) {
  write_reg(MFRC630_REG_T0CONTROL + (5 * timer), value);
}
void CLRC663::timer_set_reload(uint8_t timer, uint16_t value) {
  write_reg(MFRC630_REG_T0RELOADHI + (5 * timer), value >> 8);
  write_reg(MFRC630_REG_T0RELOADLO + (5 * timer), 0xFF);
}
void CLRC663::timer_set_value(uint8_t timer, uint16_t value) {
  write_reg(MFRC630_REG_T0COUNTERVALHI + (5 * timer), value >> 8);
  write_reg(MFRC630_REG_T0COUNTERVALLO + (5 * timer), 0xFF);
}
uint16_t CLRC663::timer_get_value(uint8_t timer) {
  uint16_t res = read_reg(MFRC630_REG_T0COUNTERVALHI + (5 * timer)) << 8;
  res += read_reg(MFRC630_REG_T0COUNTERVALLO + (5 * timer));
  return res;
}


/*
 * perform a software reset of the reader
*/
void CLRC663::softReset(void){
  write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_SOFTRESET);
  delay(50);
}


//
// get CLRC663 version
//
uint8_t CLRC663::getVersion(){
  return read_reg(MFRC630_REG_VERSION);
} 

uint8_t CLRC663::read_iso14443_uid(uint8_t *uid) {
    uint16_t atqa = iso14443a_REQA();
    if (atqa != 0) {  // Are there any cards that answered?
        // Serial.println("iso14443a_REQA");
        uint8_t sak;
        // Select the card and discover its uid.
        return iso14443a_select(uid, &sak);
    } else {
        // no card detected
        return 0;
    }

}
uint8_t CLRC663::read_iso18693_uid(uint8_t *uid, uint8_t *password) {
  uint8_t status = ISO15693_readTag(uid);
  if ((status == 0) and password){
    // try again with password  
    if (ISO15693_disablePrivacyMode(password)) {
      status = ISO15693_readTag(uid);
    }
  }
  return status;
}


// ---------------------------------------------------------------------------
// LPCD functions
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
//  measure I and Q values
//  values can be measured with loaded OR unloaded antenna, this is the easiest way to test if the lpcd setup works in principle -- load or unload antenna and compare i/q values.
//  TODO: this should happend automatically from time to time since antenna circumstances might alter
// ---------------------------------------------------------------------------
void CLRC663::AN11145_start_IQ_measurement(uint8_t* i_val, uint8_t* q_val) {
  softReset();
  cmd_idle(); //idle mode as always before any config

  // disable IRQ0, IRQ1 interrupt sources
  clear_irq0();
  clear_irq1();
  write_reg(MFRC630_REG_IRQ0EN, 0x00);
  write_reg(MFRC630_REG_IRQ1EN, 0x00);
  flush_fifo(); // Flush FIFO
 
  //  actual LPCD_config
  //  process from an 11145
  write_reg(MFRC630_REG_LPCD_QMIN, QMIN_CALIB_VAL); // Set Qmin register
  write_reg(MFRC630_REG_LPCD_QMAX, QMAX_CALIB_VAL); // Set Qmax register
  write_reg(MFRC630_REG_LPCD_IMIN, IMIN_CALIB_VAL); // Set Imin register
  write_reg(MFRC630_REG_DRVMOD, 0x89);              // set DrvMode register
  // Execute trimming procedure
  write_reg(MFRC630_REG_T3RELOADHI, 0x00);    // Write default. T3 reload value Hi
  write_reg(MFRC630_REG_T3RELOADLO, 0x10);    // Write default. T3 reload value Lo
  write_reg(MFRC630_REG_T4RELOADHI, 0x00);    // Write min. T4 reload value Hi
  write_reg(MFRC630_REG_T4RELOADLO, 0x05);    // Write min. T4 reload value Lo
  write_reg(MFRC630_REG_T4CONTROL, 0xF8);     // Config T4 for AutoLPCD&AutoRestart.Set AutoTrimm bit.Start T4.
  write_reg(MFRC630_REG_LPCD_Q_RESULT, 0x00); // Clear LPCD result; dont put bit 6 since in this case we dont wait for another actual lpcd signal but just the idle measurement
  write_reg(MFRC630_REG_RCV, 0x52);           // Set Rx_ADCmode bit
  write_reg(MFRC630_REG_RXANA, RECEIVER_GAIN_MAX); // Raise receiver gain to maximum
  write_reg(MFRC630_REG_COMMAND, 0x01);       // bit 0 = lpcd start

  delay(100);                              // have to delay before going into idle again

	cmd_idle();								//Cancel any commands
	flush_fifo();							//Flush Fifo

  write_reg(MFRC630_REG_RCV, 0x12); // Clear Rx_ADCmode bit       
  // TODO: dynamically change values in use with formula in lpcd_start function (from an11145)
  *i_val = read_reg(MFRC630_REG_LPCD_I_RESULT);// & 0x3F;
  *q_val = read_reg(MFRC630_REG_LPCD_Q_RESULT);// & 0x3F;
/*
  Serial.println("IQ_measurement:");
  Serial.print("in-phase (I): ");
  Serial.println(i_val);
  Serial.print("quadrature (Q): ");
  Serial.println(q_val);
*/
}


/*
//only in use if LPCD is used in low power devices
void CLRC663::reset_LPCD(){
  CLRC_write_reg(0x00, 0x00);
  CLRC_write_reg(0x08, 0x00);
  CLRC_write_reg(0x09, 0x00);
  CLRC_write_reg(0x02, 0xb0);
  CLRC_write_reg(0x39, 0x00);
  CLRC_write_reg(0x38, 0x12); // Clear Rx_ADCmode bit
  CLRC_write_reg(0x23, 0x5f); // stop timer
}
*/

//#define CALIBRATION 1

//LPCD setup and go to standby. for nrf52840, the pushpull option is used and an irq has to be send as soon as a tag is detected. enable irq on nrf side if needed/wanted
void CLRC663::lpcd_start(uint8_t i_value, uint8_t q_value) {
  
  write_reg(MFRC630_REG_STATUS, 0);

  //calibration is supposed to happen once in a while but it works as long as the CALI value is set to 1 and lpcd restarts after every measurement
  #if CALIBRATION
    AN11145_start_IQ_measurement();
  #endif

  // set threshold for i & q window
  uint8_t th = 1;

  // calculation from page 16
  // https://www.nxp.com/docs/en/application-note/AN11145.pdf
  uint8_t qMin = (q_value - th) | (((i_value + th) & 0x30) << 2);
  uint8_t qMax = (q_value + th) | (((i_value + th) & 0x0c) << 4);
  uint8_t iMin = (i_value - th) | (((i_value + th) & 0x03) << 6);
/*
Serial.print("qMin: ");
Serial.println(qMin);
Serial.print("qMax: ");
Serial.println(qMax);
Serial.print("iMin: ");
Serial.println(iMin);
*/
  // write calibration values to respective registers
  write_reg(MFRC630_REG_LPCD_QMIN, qMin);    //values in qmin qmax imin come from calibration process, those are basically the borders withina lpcd burst doesnt detect anything
  write_reg(MFRC630_REG_LPCD_QMAX, qMax);
  write_reg(MFRC630_REG_LPCD_IMIN, iMin);    

  //set timers (3 and 4) -- start Timer4 with AUTORestart and AutoWakeup; no start of timer3 necessary
  // Prepare LPCD command, power down time 10[ms]. Cmd time 150[µsec].
  write_reg(0x1f, 0x07); // Write T3 reload value Hi
  write_reg(0x20, 0xf2); // Write T3 reload value Lo
  write_reg(0x24, 0x00); // Write T4 reload value H
  write_reg(0x25, 0x33); // Write T4 reload value Lo
  write_reg(MFRC630_REG_T4CONTROL, 0xdf);        //activate t4 timer, autorestart, 0b11 as frequency (datasheet)
  write_reg(MFRC630_REG_LPCD_Q_RESULT, 0x40);    //clear q_result with 0x40, bit 6 tells the chip to wait for the next lpcd signal 
  uint8_t mixadc = read_reg(0x38);
  mixadc |= 0x40;                                     //from nfc_reader_lib
  write_reg(0x38, mixadc);
  uint8_t rxanabackup = read_reg(0x39);
  write_reg(MFRC630_REG_RXANA, RECEIVER_GAIN_MAX);         // Raise receiver gain to maximum
  while (!( read_reg(0x23) == 0x9f))              //wait for t4 to start (bit 6 will clear)
    ;
  write_reg(0x39, rxanabackup);
  // cancel and flush fifo
  cmd_idle();
  flush_fifo();
  // clear irq
  clear_irq0();
  clear_irq1();
  write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_IDLE_IRQEN);                     //bit 4 = idleirq
  //Enable puhspull option of IRQ Pin, enable Pin itself, let LPCD propagate to global IRQ
  //might have to play with push pull option here. measure with oscillator if necessary
  write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_IRQ_PUSHPULL | MFRC630_IRQ1EN_LPCD_IRQEN | MFRC630_IRQ1EN_IRQ_PINEN);   
  //start mode and view short bursts @ antenna per oscillator
  write_reg(MFRC630_REG_COMMAND, 0x81);       //bit 7 = standby, bit 0 = lpcd 
}

// ICODE SLIX specific commands

/*
 * The GET RANDOM NUMBER command is required to receive a random number from the label IC. 
 * The passwords that will be transmitted with the SET PASSWORD,ENABLEPRIVACY and DESTROY commands 
 * have to be calculated with the password and the random number (see Section 9.5.3.2 "SET PASSWORD")
 */
bool CLRC663::ISO15693_getRandomNumber(uint8_t *randomData){

	cmd_idle();								//cancel any commands
	flush_fifo();							//clear the fifo
	clear_irq0();							//clear irq0
	clear_irq1();							//clear irq1

  // configure a timeout timer.
  uint8_t timer_for_timeout = 0;

  uint8_t get_random[] = {0x02, 0xB2, 0x04};
  cmd_transceive(get_random, sizeof(get_random));
  
	// clear interrupts
	clear_irq0();							//clear irq0
	clear_irq1();							//clear irq1

	// Enable IRQ0,IRQ1 interrupt sources
	write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_IDLE_IRQEN  | MFRC630_IRQ0EN_TX_IRQEN);
	write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_IRQ_PINEN  | MFRC630_IRQ1EN_TIMER1_IRQEN );

	  // block until transmission ending
	  uint8_t irq0_value = 0;
	  uint8_t irq1_value = 0;
	  uint32_t timeout= millis() ;
	  while (!((irq0_value & 0x08)== 0x08)) {
	    irq0_value = get_irq0();
	    if(millis()>(timeout+50)){
	    	break;
	    }
	  }
	  //Wait for timer1 underflow (irq1(0x02) or RxIrQ irq0(0x04;
	  irq0_value =0;
	  timeout= millis();
	  while ( ((irq1_value & 0x02) !=0x02)  && ((irq0_value & 0x04) !=0x04)){
		irq1_value = get_irq1();
		irq0_value = get_irq0();
	    if(millis() > (timeout+50)){
		    	break;
		 }
	  }

	// Check for error
	if((irq1_value & 0x02)){
		return false;								//return error!
	};

	// disable IRQ0,IRQ1
	write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_CLEAR);
	write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_CLEAR);

  uint8_t fifo_len = fifo_length();
  MFRC630_PRINTF("rx_len: %hhd\n", rx_len);
  if (fifo_len == 3) {  
    uint8_t buf[3];
    read_fifo(buf, 3);
    if ((buf[1] == 0) && (buf[2] == 0)) {
      return false;
    };
    randomData[0] = buf[1];
    randomData[1] = buf[2];
    return true;
  }
  return false;
}

/*
 * The SET PASSWORD command enables the different passwords to be transmitted to the label 
 * to access the different protected functionalities of the following commands. 
 * The SET PASSWORD command has to be executed just once for the related passwords if the label is powered
 */
bool CLRC663::ISO15693_setPassword(uint8_t identifier, uint8_t *password, uint8_t *random) {

	cmd_idle();								//cancel any commands
	flush_fifo();							//clear the fifo
	clear_irq0();							//clear irq0
	clear_irq1();							//clear irq1

  // configure a timeout timer.
  uint8_t timer_for_timeout = 0;

  // set password command
  uint8_t setPassword[8] = {0x02, 0xB3, 0x36, 0x04, 0x00, 0x00, 0x00, 0x00};
  setPassword[3] = identifier;
  setPassword[4] = password[0] ^ random[0];
  setPassword[5] = password[1] ^ random[1];
  setPassword[6] = password[2] ^ random[0];
  setPassword[7] = password[3] ^ random[1];
  cmd_transceive(setPassword, sizeof(setPassword));

	// clear interrupts
	clear_irq0();							//clear irq0
	clear_irq1();							//clear irq1

	// Enable IRQ0,IRQ1 interrupt sources
	write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_IDLE_IRQEN  | MFRC630_IRQ0EN_TX_IRQEN);
	write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_IRQ_PINEN  | MFRC630_IRQ1EN_TIMER1_IRQEN );

	  // block until transmission ending
	  uint8_t irq0_value = 0;
	  uint8_t irq1_value = 0;
	  uint32_t timeout= millis() ;
	  while (!((irq0_value & 0x08)== 0x08)) {
	    irq0_value = get_irq0();
	    if(millis()>(timeout+50)){
	    	break;
	    }
	  }
	  // Wait for timer1 underflow (irq1(0x02) or RxIrQ irq0(0x04;
	  irq0_value =0;
	  timeout= millis();
	  while ( ((irq1_value & 0x02) !=0x02)  && ((irq0_value & 0x04) !=0x04)){
		irq1_value = get_irq1();
		irq0_value = get_irq0();
	    if(millis() > (timeout+50)){
		    	break;
		 }
	  }
 
	// Check for error
	if((irq1_value & 0x02)){
//		return false;								//return error!
	};

	// disable IRQ0,IRQ1
	write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_CLEAR);
	write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_CLEAR);

  uint8_t fifo_len = fifo_length();
  // should return something here?
  return true;
}

// disable privacy mode for ICODE SLIX2 tag with given password
bool CLRC663::ISO15693_disablePrivacyMode(uint8_t *password) {
  // get a random number from the tag
  uint8_t random[]= {0x00, 0x00};
  if (!ISO15693_getRandomNumber(random)) {
    return false;
  }
  
  // set password to disable privacy mode (command 0x04)  
  return ISO15693_setPassword(0x04, password, random);
}

// ---------------------------------------------------------------------------
// ISO 14443A
// ---------------------------------------------------------------------------

uint16_t CLRC663::iso14443a_REQA() {
  return iso14443a_WUPA_REQA(MFRC630_ISO14443_CMD_REQA);
}
uint16_t CLRC663::iso14443a_WUPA() {
  return iso14443a_WUPA_REQA(MFRC630_ISO14443_CMD_WUPA);
}

uint16_t CLRC663::iso14443a_WUPA_REQA(uint8_t instruction) {
  cmd_idle();
  // AN1102_recommended_registers_no_transmitter(MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER);
  flush_fifo();

  // Set register such that we sent 7 bits, set DataEn such that we can send
  // data.
  write_reg(MFRC630_REG_TXDATANUM, 7 | MFRC630_TXDATANUM_DATAEN);

  // disable the CRC registers.
  write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);
  write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);

  write_reg(MFRC630_REG_RXBITCTRL, 0);

  // ready the request.
  uint8_t send_req[] = {instruction};

  // clear interrupts
  clear_irq0();
  clear_irq1();

  // enable the global IRQ for Rx done and Errors.
  write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_RX_IRQEN | MFRC630_IRQ0EN_ERR_IRQEN);
  write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_TIMER0_IRQEN);  // only trigger on timer for irq1

  // configure a timeout timer.
  uint8_t timer_for_timeout = 0;

  // Set timer to 221 kHz clock, start at the end of Tx.
  timer_set_control(timer_for_timeout, MFRC630_TCONTROL_CLK_211KHZ | MFRC630_TCONTROL_START_TX_END);
  // Frame waiting time: FWT = (256 x 16/fc) x 2 FWI
  // FWI defaults to four... so that would mean wait for a maximum of ~ 5ms

  timer_set_reload(timer_for_timeout, 1000);  // 1000 ticks of 5 usec is 5 ms.
  timer_set_value(timer_for_timeout, 1000);

  // Go into send, then straight after in receive.
  cmd_transceive(send_req, 1);
  MFRC630_PRINTF("Sending REQA\n");
  // block until we are done
  uint8_t irq1_value = 0;
  while (!(irq1_value & (1 << timer_for_timeout))) {
    irq1_value = get_irq1();
    if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {  // either ERR_IRQ or RX_IRQ
      break;  // stop polling irq1 and quit the timeout loop.
    }
  }
  MFRC630_PRINTF("After waiting for answer\n");
  cmd_idle();

  // if no Rx IRQ, or if there's an error somehow, return 0
  uint8_t irq0 = get_irq0();
  if ((!(irq0 & MFRC630_IRQ0_RX_IRQ)) || (irq0 & MFRC630_IRQ0_ERR_IRQ)) {
    MFRC630_PRINTF("No RX, irq1: %hhx irq0: %hhx\n", irq1_value, irq0);
    return 0;
  }

  uint8_t rx_len = fifo_length();
  uint16_t res;
  MFRC630_PRINTF("rx_len: %hhd\n", rx_len);
  if (rx_len == 2) {  // ATQA should answer with 2 bytes.
    read_fifo((uint8_t*) &res, rx_len);

    MFRC630_PRINTF("ATQA answer: ");
    print_block((uint8_t*) &res, 2);
    MFRC630_PRINTF("\n");
    return res;
  }
  return 0;
}

uint8_t CLRC663::iso14443a_select(uint8_t* uid, uint8_t* sak) {
  cmd_idle();
  // AN1102_recommended_registers_no_transmitter(MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER);
  flush_fifo();

  MFRC630_PRINTF("UID input: ");
  print_block(uid, 10);
  MFRC630_PRINTF("\n");

  MFRC630_PRINTF("\nStarting select\n");

  // we do not need atqa.
  // Bitshift to get uid_size; 0b00: single, 0b01: double, 0b10: triple, 0b11 RFU
  // uint8_t uid_size = (atqa & (0b11 << 6)) >> 6;
  // uint8_t bit_frame_collision = atqa & 0b11111;

  // enable the global IRQ for Rx done and Errors.
  write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_RX_IRQEN | MFRC630_IRQ0EN_ERR_IRQEN);
  write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_TIMER0_IRQEN);  // only trigger on timer for irq1

  // configure a timeout timer, use timer 0.
  uint8_t timer_for_timeout = 0;

  // Set timer to 221 kHz clock, start at the end of Tx.
  timer_set_control(timer_for_timeout, MFRC630_TCONTROL_CLK_211KHZ | MFRC630_TCONTROL_START_TX_END);
  // Frame waiting time: FWT = (256 x 16/fc) x 2 FWI
  // FWI defaults to four... so that would mean wait for a maximum of ~ 5ms

  timer_set_reload(timer_for_timeout, 1000);  // 1000 ticks of 5 usec is 5 ms.
  timer_set_value(timer_for_timeout, 1000);
  uint8_t cascade_level;
  for (cascade_level=1; cascade_level <= 3; cascade_level++) {
    uint8_t cmd = 0;
    uint8_t known_bits = 0;  // known bits of the UID at this level so far.
    uint8_t send_req[7] = {0};  // used as Tx buffer.
    uint8_t* uid_this_level = &(send_req[2]);
    // pointer to the UID so far, by placing this pointer in the send_req
    // array we prevent copying the UID continuously.
    uint8_t message_length;

    switch (cascade_level) {
      case 1:
        cmd = MFRC630_ISO14443_CAS_LEVEL_1;
        break;
      case 2:
        cmd = MFRC630_ISO14443_CAS_LEVEL_2;
        break;
      case 3:
        cmd = MFRC630_ISO14443_CAS_LEVEL_3;
        break;
    }

    // disable CRC in anticipation of the anti collision protocol
    write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);
    write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_OFF);


    // max 32 loops of the collision loop.
    uint8_t collision_n;
    for (collision_n=0; collision_n < 32; collision_n++) {
      MFRC630_PRINTF("\nCL: %hhd, coll loop: %hhd, kb %hhd long: ", cascade_level, collision_n, known_bits);
      print_block(uid_this_level, (known_bits + 8 - 1) / 8);
      MFRC630_PRINTF("\n");

      // clear interrupts
      clear_irq0();
      clear_irq1();


      send_req[0] = cmd;
      send_req[1] = 0x20 + known_bits;
      // send_req[2..] are filled with the UID via the uid_this_level pointer.

      // Only transmit the last 'x' bits of the current byte we are discovering
      // First limit the txdatanum, such that it limits the correct number of bits.
      write_reg(MFRC630_REG_TXDATANUM, (known_bits % 8) | MFRC630_TXDATANUM_DATAEN);

      // ValuesAfterColl: If cleared, every received bit after a collision is
      // replaced by a zero. This function is needed for ISO/IEC14443 anticollision (0<<7).
      // We want to shift the bits with RxAlign
      uint8_t rxalign = known_bits % 8;
      MFRC630_PRINTF("Setting rx align to: %hhd\n", rxalign);
      write_reg(MFRC630_REG_RXBITCTRL, (0<<7) | (rxalign<<4));


      // then sent the send_req to the hardware,
      // (known_bits / 8) + 1): The ceiled number of bytes by known bits.
      // +2 for cmd and NVB.
      if ((known_bits % 8) == 0) {
        message_length = ((known_bits / 8)) + 2;
      } else {
        message_length = ((known_bits / 8) + 1) + 2;
      }

      MFRC630_PRINTF("Send:%hhd long: ", message_length);
      print_block(send_req, message_length);
      MFRC630_PRINTF("\n");

      cmd_transceive(send_req, message_length);


      // block until we are done
      uint8_t irq1_value = 0;
      while (!(irq1_value & (1 << timer_for_timeout))) {
        irq1_value = get_irq1();
        // either ERR_IRQ or RX_IRQ or Timer
        if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {
          break;  // stop polling irq1 and quit the timeout loop.
        }
      }
      cmd_idle();

      // next up, we have to check what happened.
      uint8_t irq0 = get_irq0();
      uint8_t error = read_reg(MFRC630_REG_ERROR);
      uint8_t coll = read_reg(MFRC630_REG_RXCOLL);
      MFRC630_PRINTF("irq0: %hhX\n", irq0);
      MFRC630_PRINTF("error: %hhX\n", error);
      uint8_t collision_pos = 0;
      if (irq0 & MFRC630_IRQ0_ERR_IRQ) {  // some error occured.
        // Check what kind of error.
        // error = read_reg(MFRC630_REG_ERROR);
        if (error & MFRC630_ERROR_COLLDET) {
          // A collision was detected...
          if (coll & (1<<7)) {
            collision_pos = coll & (~(1<<7));
            MFRC630_PRINTF("Collision at %hhX\n", collision_pos);
            // This be a true collision... we have to select either the address
            // with 1 at this position or with zero
            // ISO spec says typically a 1 is added, that would mean:
            // uint8_t selection = 1;

            // However, it makes sense to allow some kind of user input for this, so we use the
            // current value of uid at this position, first index right byte, then shift such
            // that it is in the rightmost position, ten select the last bit only.
            // We cannot compensate for the addition of the cascade tag, so this really
            // only works for the first cascade level, since we only know whether we had
            // a cascade level at the end when the SAK was received.
            uint8_t choice_pos = known_bits + collision_pos;
            uint8_t selection = (uid[((choice_pos + (cascade_level-1)*3)/8)] >> ((choice_pos) % 8))&1;


            // We just OR this into the UID at the right position, later we
            // OR the UID up to this point into uid_this_level.
            uid_this_level[((choice_pos)/8)] |= selection << ((choice_pos) % 8);
            known_bits++;  // add the bit we just decided.

            MFRC630_PRINTF("uid_this_level now kb %hhd long: ", known_bits);
            print_block(uid_this_level, 10);
            MFRC630_PRINTF("\n");

          } else {
            // Datasheet of mfrc630:
            // bit 7 (CollPosValid) not set:
            // Otherwise no collision is detected or
            // the position of the collision is out of the range of bits CollPos.
            MFRC630_PRINTF("Collision but no valid collpos.\n");
            collision_pos = 0x20 - known_bits;
          }
        } else {
          // Can this ever occur?
          collision_pos = 0x20 - known_bits;
          MFRC630_PRINTF("No collision, error was: %hhx, setting collision_pos to: %hhx\n", error, collision_pos);
        }
      } else if (irq0 & MFRC630_IRQ0_RX_IRQ) {
        // we got data, and no collisions, that means all is well.
        collision_pos = 0x20 - known_bits;
        MFRC630_PRINTF("Got data, no collision, setting to: %hhx\n", collision_pos);
      } else {
        // We have no error, nor received an RX. No response, no card?
        return 0;
      }
      MFRC630_PRINTF("collision_pos: %hhX\n", collision_pos);

      // read the UID Cln so far from the buffer.
      uint8_t rx_len = fifo_length();
      uint8_t buf[5];  // Size is maximum of 5 bytes, UID[0-3] and BCC.

      read_fifo(buf, rx_len < 5 ? rx_len : 5);

      MFRC630_PRINTF("Fifo %hhd long: ", rx_len);
      print_block(buf, rx_len);
      MFRC630_PRINTF("\n");

      MFRC630_PRINTF("uid_this_level kb %hhd long: ", known_bits);
      print_block(uid_this_level, (known_bits + 8 - 1) / 8);
      MFRC630_PRINTF("\n");
      // move the buffer into the uid at this level, but OR the result such that
      // we do not lose the bit we just set if we have a collision.
      uint8_t rbx;
      for (rbx = 0; (rbx < rx_len); rbx++) {
        uid_this_level[(known_bits / 8) + rbx] |= buf[rbx];
      }
      known_bits += collision_pos;
      MFRC630_PRINTF("known_bits: %hhX\n", known_bits);

      if ((known_bits >= 32)) {
        MFRC630_PRINTF("exit collision loop: uid_this_level kb %hhd long: ", known_bits);
        print_block(uid_this_level, 10);
        MFRC630_PRINTF("\n");

        break;  // done with collision loop
      }
    }  // end collission loop

    // check if the BCC matches
    uint8_t bcc_val = uid_this_level[4];  // always at position 4, either with CT UID[0-2] or UID[0-3] in front.
    uint8_t bcc_calc = uid_this_level[0]^uid_this_level[1]^uid_this_level[2]^uid_this_level[3];
    if (bcc_val != bcc_calc) {
      MFRC630_PRINTF("Something went wrong, BCC does not match.\n");
      return 0;
    }

    // clear interrupts
    clear_irq0();
    clear_irq1();

    send_req[0] = cmd;
    send_req[1] = 0x70;
    // send_req[2,3,4,5] // contain the CT, UID[0-2] or UID[0-3]
    send_req[6] = bcc_calc;
    message_length = 7;

    // Ok, almost done now, we reenable the CRC's
    write_reg(MFRC630_REG_TXCRCPRESET, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_ON);
    write_reg(MFRC630_REG_RXCRCCON, MFRC630_RECOM_14443A_CRC | MFRC630_CRC_ON);

    // reset the Tx and Rx registers (disable alignment, transmit full bytes)
    write_reg(MFRC630_REG_TXDATANUM, (known_bits % 8) | MFRC630_TXDATANUM_DATAEN);
    uint8_t rxalign = 0;
    write_reg(MFRC630_REG_RXBITCTRL, (0 << 7) | (rxalign << 4));

    // actually send it!
    cmd_transceive(send_req, message_length);
    ("send_req %hhd long: ", message_length);
    print_block(send_req, message_length);
    MFRC630_PRINTF("\n");

    // Block until we are done...
    uint8_t irq1_value = 0;
    while (!(irq1_value & (1 << timer_for_timeout))) {
      irq1_value = get_irq1();
      if (irq1_value & MFRC630_IRQ1_GLOBAL_IRQ) {  // either ERR_IRQ or RX_IRQ
        break;  // stop polling irq1 and quit the timeout loop.
      }
    }
    cmd_idle();

    // Check the source of exiting the loop.
    uint8_t irq0_value = get_irq0();
    if (irq0_value & MFRC630_IRQ0_ERR_IRQ) {
      // Check what kind of error.
      uint8_t error = read_reg(MFRC630_REG_ERROR);
      if (error & MFRC630_ERROR_COLLDET) {
        // a collision was detected with NVB=0x70, should never happen.
        return 0;
      }
    }

    // Read the sak answer from the fifo.
    uint8_t sak_len = fifo_length();
    if (sak_len != 1) {
      return 0;
    }
    uint8_t sak_value;
    read_fifo(&sak_value, sak_len);

    MFRC630_PRINTF("SAK answer: ");
    print_block(&sak_value, 1);
    MFRC630_PRINTF("\n");

    if (sak_value & (1 << 2)) {
      // UID not yet complete, continue with next cascade.
      // This also means the 0'th byte of the UID in this level was CT, so we
      // have to shift all bytes when moving to uid from uid_this_level.
      uint8_t UIDn;
      for (UIDn=0; UIDn < 3; UIDn++) {
        // uid_this_level[UIDn] = uid_this_level[UIDn + 1];
        uid[(cascade_level-1)*3 + UIDn] = uid_this_level[UIDn + 1];
      }
    } else {
      // Done according so SAK!
      // Add the bytes at this level to the UID.
      uint8_t UIDn;
      for (UIDn=0; UIDn < 4; UIDn++) {
        uid[(cascade_level-1)*3 + UIDn] = uid_this_level[UIDn];
      }

      // Finally, return the length of the UID that's now at the uid pointer.
      return cascade_level*3 + 1;
    }

    MFRC630_PRINTF("Exit cascade %hhd long: ", cascade_level);
    print_block(uid, 10);
    MFRC630_PRINTF("\n");
  }  // cascade loop
  return 0;  // getting an UID failed.
}

// ISO-18693

void CLRC663::ISO15693_init(uint8_t protocol, const uint8_t* values){

	// Configure Timers
	write_reg(MFRC630_REG_T0CONTROL, 0x98);  		// Configure T0
	write_reg(MFRC630_REG_T1CONTROL, 0x92);			// Configure T1 and cascade it with T0
	write_reg(MFRC630_REG_T2CONTROL, 0x20);			// Configure T2 for LFO Autotrimm
	write_reg(MFRC630_REG_T2RELOADHI, 0x03);		// T2 reload value for LFO AutoTrimm
	write_reg(MFRC630_REG_T2RELOADLO, 0xFF);		// T2 reload value high
	write_reg(MFRC630_REG_T3CONTROL, 0x00);			// Configure T3 (for LPCD /Autotrimm)

	//Configure FiFo
	write_reg(MFRC630_REG_FIFOCONTROL, 0x90);		// Set Fifo-Size and Waterlevel
	write_reg(MFRC630_REG_WATERLEVEL, 0xFE);		// Set Waterlevel

	//Configure RXBITCTRL
	write_reg(MFRC630_REG_RXBITCTRL, 0x80);			// Set RXBITCTLR register


	//set the Protocol
	//AN1102_recommended_registers(values);				//Write buffered values
  write_regs(MFRC630_REG_DRVMOD, values, sizeof(values));

	cmd_idle();								//Cancel any commands
	flush_fifo();							//Flush Fifo
	clear_irq0();							//Clear IRQ0
	clear_irq1();							//Clear IRQ1

	//Set Timers
	write_reg(MFRC630_REG_T0RELOADHI, 0x18);			//T0 Reload Hi
	write_reg(MFRC630_REG_T0RELOADLO, 0x86);			//T0 Reload Lo
	write_reg(MFRC630_REG_T1RELOADHI, 0x00);			//T1 Reload Hi
	write_reg(MFRC630_REG_T1RELOADLO, 0x00);			//T1 Reload Lo

	// Write in FIFO "Load protocol" params(TxProtocol=Iso15693(0a), RxProtocol=Iso15693(0a),
	write_reg(MFRC630_REG_FIFODATA, protocol);
	write_reg(MFRC630_REG_FIFODATA, protocol);

	write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_IDLE_IRQEN);	// Enable IRQ0 interrupt source
	write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_IRQ_PINEN); 	// Enable IRQ1 interrupt source
	write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_LOADPROTOCOL);	// Execute Rc663 command: "Load protocol"

	write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_CLEAR);		//Disable IRQ
	write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_CLEAR);		//Disable IRQ

	flush_fifo();									//Flush Fifo

/*
	// Apply RegisterSet
	write_reg(MFRC630_REG_TXCRCPRESET,0x7B);
	write_reg(MFRC630_REG_RXCRCCON,0x7B);
	write_reg(MFRC630_REG_TXDATANUM,0x08);
	write_reg(MFRC630_REG_TXMODWIDTH,0x00);
	write_reg(MFRC630_REG_TXSYM10BURSTLEN,0x00);
	write_reg(MFRC630_REG_TXWAITCTRL,0x00);
	write_reg(MFRC630_REG_FRAMECON,0x0F);
	write_reg(MFRC630_REG_RXCTRL,0x02);
	write_reg(MFRC630_REG_RXTHRESHOLD,0x4E);
	write_reg(MFRC630_REG_RXANA,0x04);
	write_reg(MFRC630_REG_RXWAIT, 0x8C);				// Set the RxWait register
	write_reg(MFRC630_REG_TXWAITCTRL,0xC0);
	write_reg(MFRC630_REG_TXWAITLO,0x00);

	// Write Timer-0, Timer-1 reload values(high,low)
	write_reg(MFRC630_REG_T0RELOADHI,0x18);
	write_reg(MFRC630_REG_T0RELOADLO,0x86);
	write_reg(MFRC630_REG_T1RELOADHI,0x00);
	write_reg(MFRC630_REG_T1RELOADLO,0x00);
	write_reg(MFRC630_REG_TXAMP, 0x0A);
	write_reg(MFRC630_REG_DRVMOD,0x81);
	write_reg(MFRC630_REG_STATUS,0x00); // Disable MIFARE Crypto1
	//Set Driver
*/
 
}

/*
1: Cancels previous executions and the state machine returns into IDLE mode
2: Flushes the FIFO and defines FIFO characteristics
3: Fills the FIFO with 0x0A and 0x0A.
4: Executes LoadProtocol command with parameters 0x0A and 0x0A (FIFO). This
translates to load protocol ISO 15693
5: Flushes the FIFO and defines FIFO characteristics
6: Switches the RF filed ON.
7: Fills the FIFO with 0x01 (upper part EEPROM address), 0x94 (lower part EEPROM
address), 0x28 (Register Address) and 0x12 (number of bytes)
8: Executes LoadReg command with parameters from FIFO. This translates to load from
the EEPROM address 0x0194 into the the Registers starting at 0x28 the next 0x12 bytes.
(The EEPROM of the RC663 has a lot of different predefined configurations saved which
can be loaded via LoadReg)
9: Cancels previous executions and the state machine returns into IDLE mode
10: Flushes the FIFO and maintain FIFO characteristics.
11: Clears all bits in IRQ0
12: Fills the FIFO with 0x06 (Inventory Flag), 0x01 (inventory command), 0x00 (inventory
command)
13: Executes Transceive routine
14: A loop that repeats 16 rimes since an inventory command consists of 16 time slots
15: The functions CardResponded reads the IRQ0 register and checks if a Card has
responded
16: Reads the FIFO and saves it into the UID buffer.
17: Switches off the Start symbol
18: At the next Transceive routine no data is sent.
19: Flushes the FIFO and maintain FIFO characteristics.
20: Executes Transceive routine, without start symbol and without data => only EOF is
sent, which indicates that the next timeslot is reached
21: Switch OFF RF filed.
*/
uint16_t CLRC663::ISO15693_readTag(uint8_t* uid){
	cmd_idle();								//cancel any commands
	// Set timeout for Timer0/Timer1, set reload values
	write_reg(MFRC630_REG_T0RELOADHI, 0x24);
	write_reg(MFRC630_REG_T0RELOADLO, 0xEB);
	write_reg(MFRC630_REG_T1RELOADHI, 0x00);
	write_reg(MFRC630_REG_T1RELOADLO, 0x00);

	flush_fifo();							//clear the fifo
	// Write in FIFO "Load protocol" params(TxProtocol=Iso15693(0a), RxProtocol=Iso15693(0a),
	write_reg(MFRC630_REG_FIFODATA, 0xA);
	write_reg(MFRC630_REG_FIFODATA, 0xA);
	write_reg(MFRC630_REG_COMMAND, MFRC630_CMD_LOADPROTOCOL);	// Execute Rc663 command: "Load protocol"

//  write_reg(0x39, 0x03);  // Raise receiver gain to maximum
	write_reg(MFRC630_REG_DRVMOD, 0x8E);	//Field on

  const uint8_t buf[] = MFRC630_RECOM_15693_ID1_SSC26;
  write_regs(MFRC630_REG_DRVMOD, buf, sizeof(buf));

	cmd_idle();								//cancel any commands
	flush_fifo();							//clear the fifo
	clear_irq0();							//clear irq0
	clear_irq1();							//clear irq1


	// Prepare instruction to send to fifo
	uint8_t instruction[4] ={
			MFRC630_ISO15693_FLAGS,					//set the flags,
			MFRC630_ISO15693_INVENTORY,			//set "Inventory Command"
			MFRC630_ISO15693_BlANK,					//set blank
			MFRC630_ISO15693_BlANK					//set blank
	};

	// Send instruction to reader
	cmd_transceive(instruction, 4);

	// clear interrupts
	clear_irq0();							//clear irq0
	clear_irq1();							//clear irq1

	// Enable IRQ0,IRQ1 interrupt sources
	write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_IDLE_IRQEN  | MFRC630_IRQ0EN_TX_IRQEN);
	write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_IRQ_PINEN  | MFRC630_IRQ1EN_TIMER1_IRQEN );

	  // block until transmission ending
	  uint8_t irq0_value = 0;
	  uint8_t irq1_value = 0;
	  uint32_t timeout= millis() ;
	  while (!((irq0_value & 0x08)== 0x08)) {
	    irq0_value = get_irq0();
	    if(millis()>(timeout+50)){
	    	break;
	    }
	  }
	  //Wait for timer1 underflow (irq1(0x02) or RxIrQ irq0(0x04;
	  irq0_value =0;
	  timeout= millis();
	  while ( ((irq1_value & 0x02) !=0x02)  && ((irq0_value & 0x04) !=0x04)){
		irq1_value = get_irq1();
		irq0_value = get_irq0();
	    if(millis() > (timeout+50)){
		    	break;
		 }
	  }

	// Check for error
	if((irq1_value & 0x02)){
		return 0x00;								//return error!
	};

	// disable IRQ0,IRQ1
	write_reg(MFRC630_REG_IRQ0EN, MFRC630_IRQ0EN_CLEAR);
	write_reg(MFRC630_REG_IRQ1EN, MFRC630_IRQ1EN_CLEAR);

	//see if a uid was found:
	uint16_t fifo_len = fifo_length();
  Serial.println(fifo_len);
	if(fifo_len < MFRC630_ISO15693_UID_LENGTH){
		return 0x00;								//return error - invalid uid size!
	}

	//transfer UID to variable
  uint8_t len = fifo_len - 2;

  uint8_t dummy[10];
  read_fifo(dummy, fifo_len);

  for (int i = 0; i < len; i++) {   
    uid[i] = dummy[len + 1 - i] ;
  }
 	
	return len;								//return state - valid
}

// write recommended registers

void CLRC663::AN1102_recommended_registers_skip(uint8_t protocol, uint8_t skip) {
  switch (protocol) {
    case MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER:
      {
        const uint8_t buf[] = MFRC630_RECOM_14443A_ID1_106;
        write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
    case MFRC630_PROTO_ISO14443A_212_MILLER_BPSK:
      {
        const uint8_t buf[] = MFRC630_RECOM_14443A_ID1_212;
        write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
    case MFRC630_PROTO_ISO14443A_424_MILLER_BPSK:
      {
        const uint8_t buf[] = MFRC630_RECOM_14443A_ID1_424;
        write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
    case MFRC630_PROTO_ISO14443A_848_MILLER_BPSK:
      {
        const uint8_t buf[] = MFRC630_RECOM_14443A_ID1_848;
        write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
    case MFRC630_PROTO_ISO15693_1_OF_4_SSC:
      {
        const uint8_t buf[] = MFRC630_RECOM_15693_ID1_SSC26;
        ISO15693_init(MFRC630_PROTO_ISO15693_1_OF_4_SSC, buf);
      }
      break;
    case MFRC630_PROTO_ISO15693_1_OF_4_DSC:
      {
        const uint8_t buf[] = MFRC630_RECOM_15693_ID1_DSC;
        write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
    case MFRC630_PROTO_ISO15693_1_OF_256_SSC:
      {
        const uint8_t buf[] = MFRC630_RECOM_15693_ID1_SSC52;
        write_regs(MFRC630_REG_DRVMOD+skip, buf+skip, sizeof(buf)-skip);
      }
      break;
  }
}
void CLRC663::AN1102_recommended_registers(uint8_t protocol) {
  AN1102_recommended_registers_skip(protocol, 0);
}

void CLRC663::AN1102_recommended_registers_no_transmitter(uint8_t protocol) {
  AN1102_recommended_registers_skip(protocol, 5);
}





