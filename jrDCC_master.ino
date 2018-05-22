#define DEBUG
#define I2C_TIMEOUT 500
#define CMD_PARAMS 5

#include <jr.h>
#include <jrDCC.h>
#include <jrCMD.h>
#include <jr_dynarray.h>
#include <MemoryFree.h>
#include <avr/wdt.h>



//#include <SPI.h>
#include <Wire.h>
//#include <NmraDcc.h>


//  -------------------- types -----------------
struct MasterDig {
  byte ard;
  DigitalPinU pin;
};

struct MasterLed {
  byte ard;
  LedPinU pin;
};


struct MasterAna {
  byte ard;
  AnalogPinU pin;
};

//  -------------------- global variables  -----------------
ArduinoU *slave ;
MasterAna *ana;
MasterDig *dig;
MasterLed *led;

byte systemState[get_byte(TOTAL_LEN)];
byte signalState[get_byte(SIGNALS_LEN*2)];


byte *stan;
byte *stanTmp;

int slaveCount=0;
int anaCount=0;
int digCount=0;
int ledCount=0;
int stanCount=0;

int MasterGarbage=0;
int mili=200;
int loop_print=0;


byte lastTested=70;             // for the loop 0-127
byte learningArduino=255; // which arduino info is learning now
byte learningPin=255;     // which pin is tested now

JRcmd jrcmd;
char msg_received_bufor [CMD_BUF+1];


//  -------------------- setup -----------------

void setup() {
  wdt_disable();
  Serial.begin(115200);           // start serial for output
  DEBUG_PRINT(F("Setup START"));
  Wire.begin();                // join i2c bus as the MASTER

  jrcmd.standard();
  jrcmd.add(F("SL")     ,&jr_cmd_slave_msg);
  jrcmd.add(F("SH")     ,&jr_cmd_sh);
  jrcmd.add(F("MILI")   ,&jr_cmd_mili);
  jrcmd.add(F("LOOP")   ,&jr_cmd_loop);
  jrcmd.add(F("APIN")   ,jr_cmd_getApin);
  jrcmd.add(F("ATRE")   ,jr_cmd_Atre);
  jrcmd.add(F("SETD")    ,jr_cmd_setD);
  jrcmd.add(F("DEL")    ,jr_cmd_del);
  // put your setup code here, to run once:
  //slave={};
}


//  -------------------- show led table-----------------
void showLedPin(MasterLed a) {
    JR_PRINTDECV("ard",a.ard);

    JR_PRINT(a.pin.a.name);JR_PRINTF(" ");
    
    JR_PRINTDECV("port",a.pin.a.port);
    JR_PRINTDECV("dcc",a.pin.a.dcc);
    JR_PRINTDECV("type",a.pin.a.signal_type);
    JR_PRINTDECV("s",sizeof(MasterLed));
    JR_LN;
  
}

void showLedPin() {
  //DEBUG_PRINT(anaCount);
  JR_POINTER(led);JR_PRINTV(" led",ledCount);JR_LN;
  
  for (byte i=0;i<ledCount;i++) {
    JR_VD(i);
    showLedPin(led[i]);
  }
}

//  -------------------- show analog table-----------------
void showAnalogPin(MasterAna a) {
    JR_PRINTDECV("ard",a.ard);

    JR_PRINT(a.pin.a.name);JR_PRINTF(" ");
    
    JR_PRINTDECV("port",a.pin.a.port);
    JR_PRINTDECV("threshold",a.pin.a.threshold);
    JR_PRINTDECV("type",a.pin.a.dcc_type);
    JR_PRINTDECV("s",sizeof(MasterAna));
    JR_LN;
  
}

void showAnalogPin() {
  //DEBUG_PRINT(anaCount);
  JR_POINTER(ana);JR_PRINTV(" ana",anaCount);JR_LN;
  
  for (byte i=0;i<anaCount;i++) {
    JR_VD(i);
    showAnalogPin(ana[i]);
  }
}

int sendBlockMsg(byte _type, integer _nr, boolean _block) {
	BlockMsgU msg;  
	msg.a.block=(_block)?1:0;
	msg.a.typ=_type;
	msg.a.nr=_nr;
	
	JR_PRINTDECV(F(" block?"),msg.a.block);
	JR_PRINTDECV(F(" typ"),msg.a.typ);
	JR_PRINTDECV(F(" nr"),msg.a.nr);
	
	Wire.beginTransmission (0);				// broadcast to all
	Wire.write(byte(block_msg));
	for (int i=0;i<sizeof(BlockMsg);i++) Wire.write(byte(msg.b[i]));
	byte err = Wire.endTransmission  ();  	// non-zero means error
	return err;
}

//  -------------------- show digital table-----------------
void showDigitalPin(MasterDig a) {
    JR_PRINTDECV("ard",a.ard);

    JR_PRINT(a.pin.a.name);JR_PRINTF(" ");
    
    JR_PRINTDECV("spi",a.pin.a.spi);
    JR_PRINTDECV("port",a.pin.a.port);
    JR_PRINTDECV("pull",a.pin.a.pull_up);
    JR_PRINTDECV("in",a.pin.a.in_port);
    JR_PRINTDECV("dcc",a.pin.a.dcc);
    JR_PRINTDECV("type",a.pin.a.dcc_type);
    JR_PRINTDECV("s",sizeof(MasterDig));
    JR_LN;
  
}

void showDigitalPin() {
  //DEBUG_PRINT(digCount);
  JR_POINTER(dig);JR_PRINTV(" dig",digCount);JR_LN;
  
  for (byte i=0;i<digCount;i++) {
    JR_VD(i);
    showDigitalPin(dig[i]);
  }
}



//  -------------------- show arduino table-----------------
void showArduino(ArduinoU a) {
    JR_PRINT(a.a.name);JR_PRINTF(" ");
    
    JR_PRINTDECV("i2c",a.a.i2c);
    JR_PRINTDECV("dig",a.a.digital);
    JR_PRINTDECV("ana",a.a.analog);
    JR_PRINTDECV("led",a.a.led);
    JR_PRINTDECV("byt",a.a.bytes);
    JR_PRINTDECV("board",a.a.board);
    JR_PRINTDECV("ts",a.a.ts);
    JR_PRINTDECV("s",sizeof(ArduinoU));
    JR_LN;
  
}

void showArduino() {
  //DEBUG_PRINT(slaveCount);
  JR_POINTER(slave);JR_PRINTV(" ard",slaveCount);JR_PRINTV(" stan",stanCount);JR_PRINTV(" free_mem",freeMemory());JR_LN;
  
  for (byte i=0;i<slaveCount;i++) {
    JR_VD(i);
    showArduino(slave[i]);
  }
}

//  -------------------- find arduino -----------------

byte findArduino(byte _i2c) {
  //return position (+1) of i2c device in slave table
  if (slave) {
    for (byte i=0;i<slaveCount;i++) {
      if (slave[i].a.i2c==_i2c) return i+1;
    }
  }
  return 0;
}


//  -------------------- delete arduino  -----------------

void delete_arduino(int a) {
  JR_PRINTF(" Delete Arduino ");
  JR_PRINTBIN(a);
        
        DynamicArrayHelper dynamicarrayhelper;      
        slave[a].a.i2c=127;  
        MasterGarbage=a;
        //short the Stan array
        {
          int t=0;
          for (int i=0;i<a;i++) t+=slave[i].a.bytes;
          for (int i=t+slave[a].a.bytes;i<stanCount;i++) stan[t-slave[a].a.bytes]=stan[t];
          dynamicarrayhelper.SetArrayLength( (void *&)stan,stanCount-slave[a].a.bytes,stanCount,sizeof(byte) );
        }
        
        dynamicarrayhelper.RemoveFromArray((void *&)slave,a, slaveCount,sizeof(ArduinoU));
        JR_PRINTF(" SLA-");
        // delete all pins from that board
        //renumber
        for (int i=0; i<digCount;i++) {
          if (dig[i].ard==a)  dig[i].ard=255;else
          if (dig[i].ard> a)  dig[i].ard--;
        };
        JR_PRINTF(" DIG-");
        for (int i=0; i<anaCount;i++) {
          if (ana[i].ard==a)  ana[i].ard=255; else
          if (ana[i].ard> a)  ana[i].ard--;
        };
        JR_PRINTF(" ANA-");
        for (int i=0; i<ledCount;i++) {
          if (led[i].ard==a)  led[i].ard=255; else
          if (led[i].ard> a)  led[i].ard--;
        };
        JR_PRINTF(" LED-");
    //JR_LN;
}

//  -------------------- command section -----------------

bool jr_cmd_setD  (ParserParam *p1) {  // send message to i2c slave
  ParserParam p=*p1;
  JR_PRINTF("jr_cmd_setD "); 
  JR_PRINT(CMD_PARAMS);
  {
    int sla  =p.i[1];
    int spi  =p.i[2];
    int port =p.i[3];
    int val  =p.i[4];
    
    
    if (sla<slaveCount) {
      JR_VF(sla);
      Wire.beginTransmission(byte(slave[sla].a.i2c));
      Wire.write(byte(setD));
      Wire.write(byte(spi));
      JR_VF(spi);
      Wire.write(byte(port));       
      JR_VF(port);
      Wire.write(byte(val));       
      JR_VF(val);
      Wire.endTransmission();
    }
  }
}

bool jr_cmd_Atre  (ParserParam *p1) {  // send message to i2c slave
  ParserParam p=*p1;
  JR_PRINTF("jr_cmd_Atre "); 
  {
    int sla  =p.i[1];
    int pina =p.i[2];
    int tre  =p.i[3];
    
    if (sla<slaveCount) {
    JR_VF(sla);
    Wire.beginTransmission(byte(slave[sla].a.i2c));
      Wire.write(byte(set_Atreshold));
      Wire.write(byte(pina));       
      wire_writeword(tre);
      Wire.endTransmission();
    }
   }
}
      

bool jr_cmd_getApin  (ParserParam *p1) {  // send message to i2c slave
  ParserParam p=*p1;
  JR_PRINTF("jr_cmd_getApin "); 
  {
    int sla  =p.i[1];
    int pina =p.i[2];
    if (sla<slaveCount) {
    JR_VF(sla);
    Wire.beginTransmission(byte(slave[sla].a.i2c));
      Wire.write(byte(get_Apin));
      Wire.write(byte(pina));       
      if (!Wire.endTransmission()) {
            JR_VF(pina);
            Wire.requestFrom(slave[sla].a.i2c, 4);
            if (wait_bytes(4)) {
              int wire_readword(res);
              int wire_readword(tre);
              JR_VF(res);JR_VF(tre);
              //treshold update to add
            }
      }
    }
  }
  JR_LN;  
}


bool jr_cmd_del  (ParserParam *p1) {  // send message to i2c slave
  ParserParam p=*p1;
  JR_PRINTF("jr_cmd_del "); 
  if (p.i[1] && findArduino(p.i[1])) {
    int del=findArduino(p.i[1])-1;
    JR_VF(del);
    delete_arduino(del);
  }
  JR_LN;
}

bool jr_cmd_mili  (ParserParam *p1) {  // send message to i2c slave
  ParserParam p=*p1;
  JR_PRINTF("jr_cmd_mili "); 
  if (p.i[1]) {
    mili=p.i[1];
    JR_VF(mili);
  }
  JR_LN;
}


bool jr_cmd_loop  (ParserParam *p1) {  // send message to i2c slave
  ParserParam p=*p1;
  JR_PRINTF("jr_cmd_loop "); 
  if (p.i[1]) {
    loop_print=p.i[1];
    if (loop_print && mili<2000) mili=2000;
    JR_VF(loop_print);
    JR_VF(mili);
  }
  JR_LN;
}


bool jr_cmd_sh  (ParserParam *p1) {  // send message to i2c slave
  JR_PRINTLNF("jr_cmd_sh"); 
  showArduino();    
  showDigitalPin();
  showAnalogPin();    
}

bool jr_cmd_slave_msg  (ParserParam *p1) {  // send message to i2c slave
  JR_PRINTF("jr_cmd_slave_msg "); 
  ParserParam p=*p1;
  char bufor1 [CMD_BUF+1];
  
  if (p.i[1] && strlen(p.raw)>6) {
    int fr=6+((p.i[1]<10)?-1:0)+((p.i[1]>99)?1:0);
    int g=strlen(p.raw+fr);
    strcpy(bufor1,p.raw+fr);
    JR_VF(fr);JR_VF(g);JR_PRINTF(":");
    JR_PRINT(bufor1);JR_PRINTF(":");
    JR_LN;
    
    
      Wire.beginTransmission(byte(p.i[1]));
      Wire.write(byte(msg_to_slave));
      Wire.write(byte(g));       
      Wire.write(bufor1,g);
//      JR_PRINTF(" ile:");JR_PRINTDEC(min(fr,CMD_BUF-2));
      Wire.endTransmission();
          
          
  }
  JR_LN;
};


//  -------------------- critical section -----------------

void critical_short_section() {
    //Dcc.process();
}


//  -------------------- wait bytes -----------------

boolean wait_bytes(byte ile,int _ts) {
  unsigned long  _init=millis();

  while (Wire.available()<ile) {
          critical_short_section();
      //    if ((_init+500)>millis()) {
      //      JR_PRINTF(" !! wait_bytes fails: ");JR_V(_init);JR_V(_ts);JR_PRINT(millis());JR_LN;
      //      return false;
      //    }
        }
  return true;
}

#ifdef I2C_TIMEOUT
boolean wait_bytes(byte ile) {
  return wait_bytes(ile,I2C_TIMEOUT);
}
#endif

//  -------------------- main loop memory garbage -----------------
void loop_garbage()
  {
    //------garbage start------
    if (loop_print) JR_PRINTF("Garbage Start:");
    if (loop_print) JR_PRINTDEC(MasterGarbage);
    //garbage pins which are belong to the unknown/lost arduino
    bool properNumber=false;
    if (MasterGarbage<slaveCount) {
      properNumber=true;
      if (slave[MasterGarbage].a.i2c==127) {
        JR_PRINTF(" SLA");
        DynamicArrayHelper dynamicarrayhelper;        
        if (dynamicarrayhelper.RemoveFromArray((void *&)slave,MasterGarbage, slaveCount, sizeof(ArduinoU))) {
          ;
        } else {
          JR_LN;JR_PRINTF(" !!! SLAVE inproperly removed !!! ");JR_V(MasterGarbage);JR_LN;
        }
      }
    }
    if (MasterGarbage<digCount) {
      properNumber=true;
      if (dig[MasterGarbage].ard==255) {
        JR_PRINTF(" DIG");
        DynamicArrayHelper dynamicarrayhelper;        
        dynamicarrayhelper.RemoveFromArray((void *&)dig,MasterGarbage, digCount, sizeof(MasterDig));
      }
    }
    if (MasterGarbage<anaCount) {
      properNumber=true;
      if (ana[MasterGarbage].ard==255) {
        JR_PRINTF(" ANA");
        DynamicArrayHelper dynamicarrayhelper;        
        dynamicarrayhelper.RemoveFromArray((void *&)ana,MasterGarbage, anaCount, sizeof(MasterAna));
      }
    }
    if (MasterGarbage<ledCount) {
      properNumber=true;
      if (led[MasterGarbage].ard==255) {
        JR_PRINTF(" ANA");
        DynamicArrayHelper dynamicarrayhelper;        
        dynamicarrayhelper.RemoveFromArray((void *&)led,MasterGarbage, ledCount, sizeof(MasterLed));
      }
    }
    
    if (properNumber) MasterGarbage++; else MasterGarbage=0;
    // --- loop --- check i2c devices -- stop
    if (loop_print) {JR_PRINTF(" :Garbage END");JR_LN;}
  }


//  -------------------- main loop i2c handling -----------------



//  -------------------- main loop -----------------
void loop() {
  // put your main code here, to run repeatedly:


  // check if there is any pending message from arduino's do Master to proceed

  if (msg_received_bufor[0]) {
    jrcmd.parse(msg_received_bufor,strlen(msg_received_bufor));
    msg_received_bufor[0]=0;
  }
  jrcmd.proceed(&Serial);

  
// garbage the memory in case of arduino's lost
  loop_garbage();




  // --- loop --- check i2c devices -- start
  // check if there is any new Arduino avaiable
  {
    //check the next i2c slave address
    //if (lastTested++>127) lastTested=0;
    if (lastTested++>75) { lastTested=70;   //only for test !!!
    //    while(1);  
    }
    if (loop_print)JR_PRINTF("Check I2C:");
    if (loop_print)JR_PRINTDEC(lastTested);
    
    Wire.beginTransmission(lastTested);
    if (byte error=Wire.endTransmission()) {
    //device lastTested NOT found or ERROR
      if (loop_print) JR_PRINTF(" Not Found");
      //if we have just lerning it --> clear learn
      if (lastTested==learningArduino) {
        learningArduino=255;
        learningPin=255;    
        if (loop_print) JR_PRINTF(" Learning Clear");
      }

      // is the arduino known?
      if (byte a=findArduino(lastTested)) {
        //yes --> so delete info and reduce the slave table  
        //  lost contact
        if (loop_print) JR_PRINTF(" Contact Lost");
        
        delete_arduino(a-1);
        //possibly send not active to Rocrail 
      }     
      
    } else {  
      //device lastTested FOUND
      if (loop_print) JR_PRINTF(" FOUND");
      // is the arduino known?
      if (byte a=findArduino(lastTested)) {
        //yes --> only ts update
        slave[a-1].a.ts=millis();
        if (loop_print) JR_PRINTF(" Update ts");
        ; 
      } else {
        //new device has found
        if (learningArduino==255) {
          JR_PRINTF(" +");
          byte reqSize=1+sizeof(ArduinoU);
         ; //only one device can be in the learning mode
          Wire.beginTransmission(lastTested);
          Wire.write(byte(hand_shake));

          JR_PRINTF(" handshake");
          if (!Wire.endTransmission()) {
            Wire.requestFrom(lastTested, reqSize);
            if (wait_bytes(reqSize) && Wire.read()==PROTOCOL_VER) {
               //PROTOCOL_VER has to match !!!
              JR_PRINTF(" Protocol:OK");             
              ArduinoU newArd;
              DynamicArrayHelper dynamicarrayhelper;

              JR_PRINTF(" Wait for INFO");

              for (byte i=0;i<sizeof(ArduinoU);i++) {
                byte b1=Wire.read();
                newArd.b[i]=b1;
              }

              showArduino(newArd);
              //byte elements =sizeof(slave)/sizeof(ArduinoU);
              
              if (dynamicarrayhelper.AddToArray( (void *&)slave,&newArd,slaveCount,(byte) sizeof(ArduinoU))) {
                //add the Stan array
                dynamicarrayhelper.SetArrayLength( (void *&)stan,stanCount+newArd.a.bytes,stanCount,sizeof(byte) );
                learningArduino=lastTested;
                learningPin=0;
                //start learning            
                JR_PRINTF(" Start learning");
              }
                           
            }
            
          }
        }

        
      }
      
    }
    if (loop_print) {JR_PRINTF(" :I2C");JR_LN;}
  }
    // --- loop --- check i2c devices -- end


    // -- learning Arduino  start
    //      - learn digital ports
    //      - learn analog  ports
    //      - start getting states
    
 {  //
    if (learningArduino<127) {
      DynamicArrayHelper dynamicarrayhelper;
      if (loop_print) JR_PRINTF("LEARN:");
      
      if (byte ard=findArduino(learningArduino)) {
        ard--;
        if (learningPin==0) {
          // reset counter on the learning slave
          Wire.beginTransmission(byte(slave[ard].a.i2c));
          Wire.write(byte(reset_pin_info));
          Wire.endTransmission();
          JR_PRINTF(" RESET");
        }
        if (learningPin<slave[ard].a.digital) {
          // next digital to read
          JR_PRINTF(" DIG");
          Wire.beginTransmission(byte(slave[ard].a.i2c));
          Wire.write(get_pin_info);       
          if (!Wire.endTransmission()) {
            MasterDig newItem;
            newItem.ard=ard;
            JR_PRINTBINV(F(" get"),sizeof(DigitalPinU));
            Wire.requestFrom(learningArduino, sizeof(DigitalPinU));
            learningPin++;
            if (wait_bytes(sizeof(DigitalPinU))) {
              for (byte i=0;i<sizeof(DigitalPinU);i++) {
                byte b1=Wire.read();
                newItem.pin.b[i]=b1;
              }
              showDigitalPin(newItem);
              dynamicarrayhelper.AddToArray( (void *&)dig,&newItem,digCount,(byte) sizeof(MasterDig));
              JR_PRINTF(" ADDED");
            }
          }
        } else {
          if (learningPin<slave[ard].a.digital+slave[ard].a.analog) {
            // next analog to read
            JR_PRINTF(" ANA");
            Wire.beginTransmission(byte(slave[ard].a.i2c));
            Wire.write(get_pin_info);       
            if (!Wire.endTransmission()) {
              MasterAna newItem;
              newItem.ard=ard;
              JR_PRINTBINV(F(" get"),sizeof(AnalogPinU));
              Wire.requestFrom(learningArduino, sizeof(AnalogPinU));
              learningPin++;
              if (wait_bytes(sizeof(AnalogPinU))) {
                for (byte i=0;i<sizeof(AnalogPinU);i++) {
                 byte b1=Wire.read();
                 newItem.pin.b[i]=b1;
                }
                showAnalogPin(newItem);
                dynamicarrayhelper.AddToArray( (void *&)ana,&newItem,anaCount,(byte) sizeof(MasterAna));
                JR_PRINTF(" ADDED");
              }
            }
          } else {
            if (learningPin<slave[ard].a.digital+slave[ard].a.analog+slave[ard].a.led) {
              // next analog to read
              JR_PRINTF(" LED");
              Wire.beginTransmission(byte(slave[ard].a.i2c));
              Wire.write(get_pin_info);       
              if (!Wire.endTransmission()) {
                MasterLed newItem;
                newItem.ard=ard;
                JR_PRINTBINV(F(" get"),sizeof(LedPinU));
                Wire.requestFrom(learningArduino, sizeof(LedPinU));
                learningPin++;
                if (wait_bytes(sizeof(LedPinU))) {
                  for (byte i=0;i<sizeof(LedPinU);i++) {
                   byte b1=Wire.read();
                   newItem.pin.b[i]=b1;
                  }
                  showLedPin(newItem);   
                  dynamicarrayhelper.AddToArray( (void *&)led,&newItem,ledCount,(byte) sizeof(MasterLed));
                  JR_PRINTF(" ADDED");
                }
              }  
            } else {
            // end od learning --> so start getting states
            learningArduino=255;
            mili=6000;loop_print=1;
            JR_PRINTF(" END LEARNING");
            }
          }
        }    
      } else {
        if (loop_print) JR_PRINTDECV("i2c not found",learningArduino);
        learningArduino=255;
      }
    if (loop_print) {JR_PRINTF(" :LEARN");JR_LN;}
   }
 }
    // -- learning Arduino  end


 //if (0)
 { //get Slave states
   DynamicArrayHelper dynamicarrayhelper;      
   int tmpCount=0;
   int tmpA=0;
   
   dynamicarrayhelper.SetArrayLength( (void *&)stanTmp,stanCount,tmpCount,sizeof(byte) );
   
    for (int i=0;i<slaveCount;i++) {      
      // in learning do not get the states
      if (slave[i].a.i2c==learningArduino) continue;        
            Wire.beginTransmission(byte(slave[i].a.i2c));
            Wire.write(get_states);       
            if (!Wire.endTransmission()) {
              if (loop_print) {JR_PRINTDECV(F("slave"),i);}
              if (loop_print) {JR_PRINTDECV(F("i2c"),byte(slave[i].a.i2c));}
              byte newItem;
              if (loop_print) {JR_PRINTDECV(F("bytes"),byte(slave[i].a.bytes));}
              Wire.requestFrom(byte(slave[i].a.i2c), byte(slave[i].a.bytes));
              int tmpS=tmpA;
              for(int j=0;j<slave[i].a.bytes;j++) {
                byte b=Wire.read();
                if (loop_print) {JR_PRINTBINV(j,b);}
                stanTmp[tmpA++]=b;
              }
              byte u=slave[i].a.analog+slave[i].a.digital;
              
              u=GetBit(stanTmp,u+(tmpS*8));
              JR_VF(u);
              if (!msg_received_bufor[0] && u) {
                  Wire.beginTransmission(byte(slave[i].a.i2c));
                  Wire.write(msg_to_master);       
                  if (!Wire.endTransmission()) {
                    JR_PRINTF (" MSG:");JR_PRINT(slave[i].a.i2c);JR_PRINTF(":");
                    Wire.requestFrom(byte(slave[i].a.i2c), byte(CMD_BUF));
                    for (byte u=0;u<CMD_BUF;u++) msg_received_bufor[u]=Wire.read();
                    msg_received_bufor[CMD_BUF]=0;
                    JR_PRINT(msg_received_bufor);JR_PRINT (strlen(msg_received_bufor));JR_LN;
                  } 
              }
              
            }
        //while (1);
//        tmpS+=slave[i].a.bytes;
    }

  //check if any state change
  tmpCount=0;
  for (int i=0;i<slaveCount;i++) {      
      // in learning do not get the states
      if (slave[i].a.i2c==learningArduino) continue;        

      for (int j=0;j<slave[i].a.digital;j++) {
        byte b=GetBit(stanTmp,tmpCount*8+j);
        if (b!=GetBit(stan,tmpCount*8+j)) {
          Serial.println();
          Serial.print(F("FB "));
          if (b) { 
            SetBit(stan,tmpCount*8+j); 
            Serial.print(F("1 "));
          } else { 
            ClearBit(stan,tmpCount*8+j); 
            Serial.print(F("0 "));
          }
          Serial.println(slave[i].a.name);
        }
      }

      for (int j=0;j<slave[i].a.analog;j++) {
        byte d=slave[i].a.digital;
        byte b=GetBit(stanTmp,tmpCount*8+j+d);
        if (b!=GetBit(stan,tmpCount*8+j+d)) {
          Serial.println();
          Serial.print(F("FB "));
          if (b) { 
            SetBit(stan,tmpCount*8+j+d); 
            Serial.print(F("1 "));
          } else { 
            ClearBit(stan,tmpCount*8+j+d); 
            Serial.print(F("0 "));
          }
          Serial.println(slave[i].a.name);
        }
      }
      
      tmpCount+=slave[i].a.bytes;
  }  
    
  dynamicarrayhelper.SetArrayLength( (void *&)stanTmp,0,tmpCount,sizeof(byte) );
 }


 
 if (loop_print) JR_LN;
 delay (mili);
}
