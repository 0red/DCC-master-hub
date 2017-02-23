#define DEBUG
#define I2C_TIMEOUT 500

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

struct MasterAna {
  byte ard;
  AnalogPinU pin;
};

//  -------------------- global variables  -----------------
ArduinoU *slave ;
MasterAna *ana;
MasterDig *dig;
byte *stan;
byte *stanTmp;

int slaveCount=0;
int anaCount=0;
int digCount=0;
int stanCount=0;

int MasterGarbage=0;
int mili=2000;
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
  
  // put your setup code here, to run once:
  //slave={};
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
          if (dig[i].ard==a) dig[i].ard=255;else
          if (dig[i].ard> a)  dig[i].ard--;
        };
        JR_PRINTF(" DIG-");
        for (int i=0; i<anaCount;i++) {
          if (ana[i].ard==a) ana[i].ard=255; else
          if (ana[i].ard> a)  ana[i].ard--;
        };
        JR_PRINTF(" ANA-");
    //JR_LN;
}

//  -------------------- command section -----------------

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
    if (properNumber) MasterGarbage++; else MasterGarbage=0;
    // --- loop --- check i2c devices -- stop
    if (loop_print) {JR_PRINTF(" :Garbage END");JR_LN;}
  }


//  -------------------- main loop -----------------
void loop() {
  // put your main code here, to run repeatedly:

  jrcmd.proceed(&Serial);

  loop_garbage();


  // --- loop --- check i2c devices -- start
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
 {  //
    if (learningArduino<127) {
      DynamicArrayHelper dynamicarrayhelper;
      if (loop_print) JR_PRINTF("LEARN:");
      
      if (byte ard=findArduino(learningArduino)) {
        ard--;
        if (learningPin==0) {
          Wire.beginTransmission(byte(slave[ard].a.i2c));
          Wire.write(byte(reset_pin_info));
          Wire.endTransmission();
          JR_PRINTF(" RESET");
        }
        if (learningPin<slave[ard].a.digital) {
          // digital to read
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
            // digital to read
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
            // end od learning
            learningArduino=255;
            JR_PRINTF(" END LEARNING");
          }
        }    
      } else {
        if (loop_print) JR_PRINTDECV("i2c not found",learningArduino);
        learningArduino=255;
      }
    if (loop_print) {JR_PRINTF(" :LEARN");JR_LN;}
   }
 }


 //if (0)
 { //get states
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
              for(int j=0;j<slave[i].a.bytes;j++) {
                byte b=Wire.read();
                if (loop_print) {JR_PRINTBINV(j,b);}
                stanTmp[tmpA++]=b;
              }
            }
        //while (1);
    }
  dynamicarrayhelper.SetArrayLength( (void *&)stanTmp,0,tmpCount,sizeof(byte) );
 }


 
 if (loop_print) JR_LN;
 delay (mili);
}