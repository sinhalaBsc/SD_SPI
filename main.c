 // SD Card Utilities
 // Uses SPI configuration defined by SPIModule, SystemWideConfiguration and MyConfig
 #include "MyConfig.h"
 #include "SystemWideConfig.h"
 #include "SPIModule.h"
 #include "SDCardUtilities.h"
 #include "utilities.h"
 
 unsigned long write_sd_sector (SD_CARD *sd_card, unsigned long address, unsigned char *buffer) {
     // write 512 byte buffer to sd_card sector at address, return next available sector, 0 if error
     SD_RESPONSE         sd_response;        // r1 response from sd_card 
     int                 i;                    // counter
     unsigned int         timeout;            // timeout value
     unsigned char         rc;                    // char returned from SPI
     unsigned long        next_address;        // next available sector
     
     if (!sd_card->hc)                                                    // if card is std capacity
         address = (unsigned long)address << 9;                            // convert address to byte offset (blocks always 512bytes)
     
     sd_response = tx_cmd(WRITE_SINGLE_BLOCK,address,-1,NULL,NODBUG);    // send CMD24, leave card asserted
     if (sd_response.byte != 0x00) {                                        // check that command was accepted
         sd_card->w_err = 1;                                                // write command not accepted
         UnAssertMySD();                                                    // unassert (because main code will not expect to)
         return 0;                                                        // return error
     }else{                                                                // command completed OK
         SPIc(0xFE);                                                        // send data start token
         for (i=0;i<512;i++)    SPIc(buffer[i]);                            // send buffer
         SPIc(0xFF);    SPIc(0xFF);                                            // send crc (don't care)
     }
     rc = SPIc(0xFF);                                                    // next byte will indicate if data was accepted
     if ((rc & 0x0F) != 0x05) {                                            // check for "data accepted" pattern in reponse
         sd_card->d_err = 1;                                                // data not accepted
         UnAssertMySD();                                                    // unassert card (because main code will not expect to)
         return 0;                                                        // return error
     }else{                                                                // wait for write to complete
         timeout = 0xFFFF;                                                // wait for huge timeout
         do {                                                        
             rc = SPIc(0xFF);                                            // send clocks
             timeout--;                                                    // countdown
         } while ((rc == 0x00) && (timeout != 0x00));                    // until reply is not 0 or times out
         sd_card->timeout = timeout;                                        // keep timeout value (DEBUG)
     }
     if (timeout == 0x00) {                                                // checkif timed out
         sd_card->no_w = 1;                                                // timed out waiting for write to complete        
         UnAssertMySD();                                                    // unassert card (because main code will not expect to)
         return 0;                                                        // return error
     }
     SPIc(0xFF);                                                            // send 8 clocks to wrap up command
     UnAssertMySD();                                                        // unassert card (because main code will expect this)    
     next_address = (unsigned long)(address+0x01);                        // set next available sector
     
     return next_address;                                                // return next address
 };
 
 SD_RESPONSE tx_cmd(unsigned char command_index, unsigned long command_argument, char extra_bytes, unsigned char *location, unsigned char debug) {
     union sd_command     command;                            // framed command
     SD_RESPONSE            response;                            // response from SD Card (R1, R2, R7)
     unsigned int        timeout;                            // timeout value
     unsigned char        (*ptoSPIcall)(unsigned char c);        // pointer to correct function for SPI send/rcv
     unsigned char        counter;                            // gp counter
     
     if (debug == 0x01)     ptoSPIcall = &SPIcdebug;            // if debugging, use SPIcdebug
     else                ptoSPIcall = &SPIc;                    // if not debugging, use SPIc
     
     AssertMySD();                                            // assert card        
                                                             // frame command properly
     command.fields.start_bit         = 0;                    // always 0
     command.fields.transmitter_bit    = 1;                    // always 1
     command.fields.index             = command_index;        // insert cmd index    
     command.fields.argument            = command_argument;        // insert cmd argument
     if (command_index == GO_IDLE_STATE)                      // calculate any CRCs (SEVEN bits)
         command.fields.crc             = 0x4A;
     else if (command_index == SEND_IF_COND) 
         command.fields.crc             = 0x43;
     else if (command_index == SEND_OP_COND) 
         command.fields.crc             = 0xF9;
     else if (command_index == READ_OCR) 
         command.fields.crc             = 0x25;
     else if (command_index == SEND_CSD) 
         command.fields.crc             = 0xAF;
     else if (command_index == CRC_ON_OFF) 
         command.fields.crc             = 0x25;
     else if (command_index == SET_BLOCKLEN) 
         command.fields.crc             = 0xFF;
     else
         command.fields.crc             = 0x7F;                    // CRC for all else (111 1111)
     command.fields.end_bit            = 1;                    // always 1
 
     ptoSPIcall(command.bytes.cmd_w_starttx);                // send 6 byte command
     ptoSPIcall(command.bytes.arg_msb);
     ptoSPIcall(command.bytes.arg_2);
     ptoSPIcall(command.bytes.arg_3);
     ptoSPIcall(command.bytes.arg_lsb);
     ptoSPIcall(command.bytes.crc_w_stop);
     timeout = 0xFF;
     do {        
         response.byte = ptoSPIcall(0xFF);
         timeout--;
     }while((response.byte == 0xFF) && (timeout != 0x00));
     if (timeout == 0)
         while (1);                                          // card did not complete command
         
     if (extra_bytes > 0) {                                    // if CMD expects more data
         for (counter = 0;counter < extra_bytes; counter++){    // get bytes
             *(location+counter)=ptoSPIcall(0xFF);            // store in "location"
         }    
         ptoSPIcall(0xFF);                                    // clean up with 8 clocks
         UnAssertMySD();                                        // deassert
         return response;                                    // return response byte
     } else if (extra_bytes == 0){                            // if CMD expects NO data
         ptoSPIcall(0xFF);                                    // clean up with 8 clocks
         UnAssertMySD();                                        // deassert
         return response;                                    // return response byte        
     } else {                                                // else (<0) CMD expects more data....
         ptoSPIcall(0xFF);                                    // clean up with 8 clocks
         return response;                                    // just return response byte
     }    
 };    
 
 SD_CARD init_sdcard (void) {                                    // used to initialize an SD Card
     SD_CARD                sd_card;                                // sd_card union (see .h)
     SD_RESPONSE            sd_response;                            // sd command response (see .h)
     unsigned char        rc;                                        // char returned from SPI 
     unsigned int        i,j;                                    // counters
     unsigned char        if_cond[4], ocr[4], csd[20], timeout;    // sd card registers and timeout
     unsigned long        c_size;                                 // C_SIZE card size from csd register
     unsigned char        block_len;                                 // BLOCK_LEN block length from csd register
     unsigned char        c_size_mult;                             // C_SIZE_MULT size multiploer from csd register
     
     sd_card.ok         = 0;    // card is initialized or status is OK
     sd_card.wp         = 0;    // card is write protected
     sd_card.cd        = 0;    // card is detected
     sd_card.hc        = 0;    // card is high capacity
     sd_card.v1        = 0;    // card is version 1
     sd_card.no_idle    = 0;    // card did not GO_STATE_IDLE
     sd_card.no_opc    = 0;     // send_op_cond timed out
     sd_card.no_ocr    = 0;    // send_ocr command timed out
     sd_card.no_pup    = 0;    // card not powered up after init request
     sd_card.no_csd    = 0;     // send_CSD command timed out
     sd_card.no_w    = 0;    // write did not complete (timed out)
     sd_card.csd_err    = 0;     // card did not issue 0xFE at start of CSD data
     sd_card.w_err    = 0;    // write error (command not accepted)
     sd_card.r_err    = 0;    // read error
     sd_card.d_err    = 0;    // data error (data not accepted)
     sd_card.pad        = 0;
     sd_card.timeout    = 0;    // timeout value when ops timeout
     sd_card.size    = 0;    // card size in 512kbyte blocks (ie actual capacity is 1000 * m_size / 2 bytes)
     /* --------------------------toggle power and wait for card to start---------------------------------------*/
     MySDPowerOFF;                                                    // Turn OFF the SD Card
     Delayms(500);                                                    // wait for Card to power down
     MySDPowerON;                                                    // Turn SD Card ON
     Delayms(500);                                                    // wait for Card to power up
     /* ---------------------------check for sd_card and wp status---------------------------------------------*/
     sd_card.cd = MySDCD;                                            // set chip detect status
     if (sd_card.cd != 0) return sd_card;                            // if no card, return
     sd_card.wp = MySDWP;                                            // set write protect status
     if (sd_card.wp == 1) return sd_card;                            // if write protected, return
     /* ---------------------------start sending SPI info to start card----------------------------------------*/    
       UnAssertMySD();                                                    // Make sure CS is high
     SPI_GOSLOW                                                        // set SPI to run at lowest setting
       Delayms(1);                                                        // wait for SPI module to sync
     for (i=0;i<10;i++) SPIc(0xFF);                                    // send 74+ clocks (80)
     AssertMySD();                                                    // assert the card
     Delayms(1);                                                        // wait 
     /*----------------------- initialize card using Microchip's Startup Sequence-------------------------------*/
     sd_response = tx_cmd(GO_IDLE_STATE,0x00000000,0,NULL,NODBUG);            // send command to go idle
     if((sd_response.byte == 0xFF) || ((sd_response.byte & 0xF7) != 0x01)) {    // check is card is idle or bus floating
         sd_card.no_idle = 1;                                                // card failed to go idle
         return sd_card;                                                        // return card status with error
     }    
     sd_response = tx_cmd(SEND_IF_COND,0x000001AA,4,if_cond,NODBUG);            // send command to check voltage
     if (sd_response.byte != 0x01) {                                            // v2 cards respond in idle state
         sd_card.v1 = 1;                                                        // v1 card
         return sd_card;                                                        // return card status
     }    
     //for (i=0;i<4;i++) if_cond[i]=SPIc(0xff);                                // read interface condition            
     if ((if_cond[3] == 0xaa) && (if_cond[2] ==0x01))                         // check if echo pattern and supply voltage is correct 
     {
         timeout = 200;                                                        // try 200 times to get operating condition
         do {
             sd_response = tx_cmd(SEND_OP_COND,0x40000000,0,NULL,NODBUG);    // ask for op cond with HCS = 1
             timeout--;
         } while ((sd_response.byte != 0x00) && (timeout != 0x00));            // until response is idle or timeout
         if (timeout == 0x00) {                                                
             sd_card.no_opc = 1;                                                // set time out warning
             return sd_card;                                                    // return with error
         }    
         sd_response = tx_cmd(READ_OCR, 0x00000000, 4, ocr,NODBUG);            // ask for ocr
         if (sd_response.byte != 0x00) {                                        // failed to read ocr cmd
             sd_card.no_ocr = 1;                                                // set ocr error
             return sd_card;                                                    // return with error
         }    
         //for (i=0;i<4;i++) ocr[i]=SPIc(0xff);                                // read ocr                            
         if ((ocr[0] & 0x80) != 0x80) {
             sd_card.no_pup = 1;                                                // card not in power-up state, ccs bit invalid
             return sd_card;
         }    
         if ((ocr[0] & 0x40) == 0x40) {                                        // check ccs bit
             sd_card.hc = 1;                                                    // card is high capacity
         }else{
             sd_card.hc = 0;                                                    // card is low capacity
         }        
     }else{
         sd_card.hc = 0;                                                        // card is low capacity (didn't respond to SEND_IF_COND properly)
         do {
             sd_response = tx_cmd(SEND_OP_COND,0x00000000,0,NULL,NODBUG);    // send op command with HCS = 0
             timeout--;
         } while ((sd_response.byte != 0x00) && (timeout != 0x00));
         if (timeout == 0x00){                                                // timeout trying to SEND_OP_CMD
             sd_card.no_opc = 1;                                                // set op_cond error
             return sd_card;                                                    // return with error
         }    
     }    
     UnAssertMySD();                                                            // unassert card
     SPI_GOMED                                                                // set SPI to medium speed
     AssertMySD();                                                            // assert card
 
     timeout = 100;                                                            // try 100 times to get CSD register
     do {
         UnAssertMySD();                                                        // in case SEND_CSD already sent
         sd_response = tx_cmd(SEND_CSD,0x00000000,-1,NULL,NODBUG);            // CMD9: ask for csd
         timeout--;
     } while ((sd_response.byte != 0x00) && (timeout != 0x00));
     if (timeout == 0x00) {
         sd_card.no_csd = 1;                                                    // timed out waiting for CSD
         UnAssertMySD();    
         return sd_card;
     }
     timeout = 30;                                                            // try 30 times to get data start token
     do {                                                
         rc = SPIc(0xFF);                                    
         timeout--;
     } while ((rc != 0xFE) && (timeout != 0x00));
     if (timeout == 0x00) {
         sd_card.csd_err = 1;                                                // no data start toekn from this card
         UnAssertMySD();
         return sd_card;
     }
     for (i=0;i<17;i++) csd[i] = SPIc(0xFF);                                    // read csd
     
     if(csd[0] & 0xC0) {                                                        //Check CSD_STRUCTURE field for v2+ struct device    
         //Must be a v2 device (or a reserved higher version, that doesn't currently exist)
         //Extract the C_SIZE field from the response.  It is a 22-bit number in bit position 69:48.  This is different from v1.  
         //It spans bytes 7, 8, and 9 of the response.
         c_size = (((unsigned long)csd[7] & 0x3F) << 16) | ((unsigned int)csd[8] << 8) | csd[9];
         sd_card.size = ((unsigned long)(c_size + 1) * (unsigned int)(1024u)) - 1;
     }else{
         //Must be a v1 device.
         //Extract the C_SIZE field from the response.  It is a 12-bit number in bit position 73:62.  
         //Although it is only a 12-bit number, it spans bytes 6, 7, and 8, since it isn't byte aligned.
         c_size = ((unsigned long)csd[6] << 16) | ((unsigned int)csd[7] << 8) | csd[8];    //Get the bytes in the correct positions
         c_size &= 0x0003FFC0;    //Clear all bits that aren't part of the C_SIZE
         c_size = c_size >> 6;    //Shift value down, so the 12-bit C_SIZE is properly right justified in the unsigned long.
         //Extract the C_SIZE_MULT field from the response.  It is a 3-bit number in bit position 49:47.
         c_size_mult = ((unsigned int)((csd[9] & 0x03) << 1)) | ((unsigned int)((csd[10] & 0x80) >> 7));
         //Extract the BLOCK_LEN field from the response. It is a 4-bit number in bit position 83:80.
         block_len = csd[5] & 0x0F;
         block_len = 1 << (block_len - 9); //-9 because we report the size in sectors of 512 bytes each
         sd_card.size = ((unsigned long)(c_size + 1) * (unsigned int)((unsigned int)1 << (c_size_mult + 2)) * block_len) - 1;
     }
     
     sd_response = tx_cmd(CRC_ON_OFF,0x00,0,NULL,NODBUG);          // Turn off CRC7 if we can, might be an invalid cmd on some cards (CMD59)
     sd_response = tx_cmd(SET_BLOCKLEN,512u,0,NULL,NODBUG);        // Now set the block length to media sector size. It should be already
     sd_card.ok = 1;                                                // card is ready
     UnAssertMySD();
     
     return         sd_card;
 }
     
     
 /*****************************************************************************
  * EOF
  *****************************************************************************/
