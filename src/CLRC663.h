

#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include "mfrc630_def.h"

// debug print statement.
#ifdef MFRC630_DEBUG_PRINTF
#define MFRC630_PRINTF(...) MFRC630_DEBUG_PRINTF(__VA_ARGS__)
#else
#define MFRC630_PRINTF(...)
#endif

enum clrc633_transport {
  MFRC630_TRANSPORT_SPI = 1,
  MFRC630_TRANSPORT_I2C = 2
};


class CLRC663 {
    private:
        enum clrc633_transport _transport;
        int8_t _cs;
        int8_t _rst;
        int8_t _irq;
        // SPI specific
        SPIClass *_spi;
        // I2C specific
        uint8_t _i2c_addr;
        TwoWire *_wire;
        void write_regs(uint8_t reg, const uint8_t* values, uint8_t len);
        // utility functions
        void write_fifo(const uint8_t* data, uint16_t len);
        void read_fifo(uint8_t* rx, uint16_t len);
        // command functions
        void cmd_load_reg(uint16_t address, uint8_t regaddr, uint16_t length);
        void cmd_load_protocol(uint8_t rx, uint8_t tx);
        void cmd_transceive(const uint8_t* data, uint16_t len);
        void cmd_idle();
        void flush_fifo();
        uint16_t fifo_length();
        // timer functions
        void activate_timer(uint8_t timer, uint8_t active);
        void timer_set_control(uint8_t timer, uint8_t value);
        void timer_set_reload(uint8_t timer, uint16_t value);
        void timer_set_value(uint8_t timer, uint16_t value);
        uint16_t timer_get_value(uint8_t timer);
        // ICode SLIX
        bool ISO15693_getRandomNumber(uint8_t *randomData);
        bool ISO15693_setPassword(uint8_t identifier, uint8_t *password, uint8_t *random);
        bool ISO15693_disablePrivacyMode(uint8_t *password);
        // ISO-14443
        uint16_t iso14443a_REQA();
        uint16_t iso14443a_WUPA();
        uint16_t iso14443a_WUPA_REQA(uint8_t instruction);
        uint8_t iso14443a_select(uint8_t* uid, uint8_t* sak);
        // ISO-18693
        void ISO15693_init(uint8_t protocol, const uint8_t* values);
        uint16_t ISO15693_readTag(uint8_t *uid);
        void AN1102_recommended_registers_skip(uint8_t protocol, uint8_t skip);
    public:
        // SPI constructor
        CLRC663(SPIClass *spi, int8_t cs, int8_t rst = -1, int8_t irq = -1);
        // I2C constructor
        CLRC663(uint8_t i2c_address = 0x2A, int8_t rst = -1, int8_t irq = -1);

        // public functions
        void begin();
        void begin(int sda, int scl);
        void end();
        void reset();
        void softReset();
        void hardReset();
        uint8_t getVersion(); 
        // register interaction functions
        uint8_t read_reg(uint8_t reg);
        void write_reg(uint8_t reg, uint8_t value);
        // irq functions
        void clear_irq0();
        void clear_irq1();
        uint8_t get_irq0();
        uint8_t get_irq1();  
        // UID functions   
        void print_block(const uint8_t* data, uint16_t len);
        void AN1102_recommended_registers(uint8_t protocol);
        void AN1102_recommended_registers_no_transmitter(uint8_t protocol);
        uint8_t read_iso14443_uid(uint8_t *uid);
        uint8_t read_iso18693_uid(uint8_t *uid, uint8_t *password = NULL);
        // LPCD 
        void AN11145_start_IQ_measurement(uint8_t* i_val, uint8_t* q_val);
        void lpcd_start(uint8_t i_value, uint8_t q_value) ;
};





