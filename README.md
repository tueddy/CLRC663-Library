# CLRC663-Library

An Arduino library for the CLRC633 reader
 
Features:
   - can be connected via SPI or I2C (2 GPIOs less)
   - read ISO-14443 UID's
   - read ISO-18693 UID's
   - support for ICode-SLIX password protected tags (privacy mode)
   - LPCD - low power card detection
   - Auto calibration for LPCD-mode. Avoid false wakeups
  
Copyright (c) 2023 tueddy (Dirk Carstensen) 

Some header declarations and functions taken and extended from https://github.com/kenny5660/CLRC663.git

![CLRC663](https://user-images.githubusercontent.com/11274319/222130502-1bac1d0e-7034-4ce1-81d3-b94f3365112e.jpg)
