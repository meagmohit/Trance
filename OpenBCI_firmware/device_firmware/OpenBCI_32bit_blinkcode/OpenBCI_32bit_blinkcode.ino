/*
 *
 *  >>>> THIS CODE USED TO STREAM OpenBCI V3_32 DATA TO DONGLE <<<<
 *  >>>> SUPPORTS 16 CHANNELS WITH DAISY MODULE EXPANSION CARD <<<<
 *
 * This code is written to target a PIC32MX250F128B with UDB32-MX2-DIP bootloader
 * To Program, user must manually reset the PIC32 on the OpenBCI 32bit Board
 * press RST, then press PROG, then release RST, then release PROG
 * Adjust as needed if you are testing on different hardware.
 *
 * Find the custom libraries here https://github.com/OpenBCI/OpenBCI_32bit_Libraries
 * download and place the libraries inside your: User/Documents/Arduino/libraries folder.
 * 
 * You need the latest version of the chipKIT-core hardware files. 
 * Go to the chipKIT website http://chipkit.net/ 
 * or directly to their core Wiki http://chipkit.net/wiki/index.php?title=ChipKIT_core
 * and follow the directions to install chipKIT board files into the latest Arduino IDE.
 * Then, when you open up Arduino IDE, you can select 'OpenBCI 32' from the boards menu!
 *
 * Any SDcard code is based on RawWrite example in SDFat library
 * ASCII commands are received on the serial port to configure and control
 * Serial protocol uses '+' immediately before and after the command character
 * We call this the 'burger' protocol. the '+' re the buns. Example:
 * To begin streaming data, send '+b+'
 * The OpenBCI Dongle firmware takes care of the burger protocol
 *
 * This software is provided as-is with no promise of workability
 * Use at your own risk, wysiwyg.

 * Made by Joel Murphy, Luke Travis, Conor Russomanno Summer, 2014.
 */

#include <OBCI32_SD.h>
#include <DSPI.h>
#include <math.h>
#include <EEPROM.h>
#include "OpenBCI_32_Daisy.h"


//------------------------------------------------------------------------------
//  << SD CARD BUSINESS >>
boolean SDfileOpen = false;
char fileSize = '0';  // SD file size indicator
int blockCounter = 0;
//------------------------------------------------------------------------------
//  << OpenBCI BUSINESS >>
boolean is_running = false;    // this flag is set in serialEvent on reciept of ascii prompt
OpenBCI_32_Daisy OBCI;         //Uses SPI bus and pins to say data is ready.
// these are used to change individual channel settings from PC
char currentChannelToSet;      // keep track of what channel we're loading settings for
boolean getChannelSettings = false; // used to receive channel settings command
int channelSettingsCounter;    // used to retrieve channel settings from serial port
int leadOffSettingsCounter;
boolean getLeadOffSettings = false;
// these are all subject to the radio requirements: 31byte max packet length (maxPacketLength - 1 for packet checkSum)
#define OUTPUT_NOTHING (0)        // quiet
#define OUTPUT_8_CHAN (1)         // not using Daisy module
#define OUTPUT_16_CHAN (2)        // using Daisy module
int outputType = OUTPUT_8_CHAN;   // default to 8 channels

//------------------------------------------------------------------------------
//  << LIS3DH Accelerometer Business >>
boolean addAccelToSD = false;   // this flag get's set and cleared in the code below
//------------------------------------------------------------------------------
//  << PUT FILTER STUFF HERE >>
boolean useFilters = false;     // add DSP if you like, and set this true to turn them on
//------------------------------------------------------------------------------
//  << BASIC BOARD STUFF >>
int LED = 11;     // alias for the blue LED
int PGCpin = 12;  // PGC pin goes high when PIC is in bootloader mode
//------------------------------------------------------------------------------
//  << AUXILIARY DATA STUFF >>
boolean addAuxToSD = false;     // use this to write auxiliary data to SD if you like
//------------------------------------------------------------------------------

// << EYE BLINK DETECTION >> 


const int s2rate = 50;     // M2 changed from 50 to 250
const int srate = 50;
const int fs = 250;
const int delta = 100;
const int filter_order = 4;
const int total_time = 40;   // total time transferred //C//40
const int batch_time = 8;   // amount of time for command
const int data_len = batch_time*srate;
const int data_len_max = s2rate*total_time;
const int m_ind = 20;
int time_index;
//int time_shift = 125;

// For Filtering the Data

float buffer_val[filter_order];
int buffer_index;
float buffer_avg;
// For storing the last 10s of data
float signal_data[data_len+1];
float signal_data_max[data_len_max];
float signal_data_max2[data_len_max];
float current_val;
float raw_value;
int data_index;
int data_index_max;
// For Peak Detection
boolean lookformax;
float mn;
float mx;
int mnpos;
int mxpos;
int mintab_pos[m_ind];
int maxtab_pos[m_ind];
float mintab_val[m_ind];
float maxtab_val[m_ind];
int mx_ind;
int mn_ind;
//Processing
int time_process = data_len - 1; // Initialize with this to check
unsigned long mytime_millis;
boolean blink_detected;
boolean loop_finish;
int total_detected_blinks;
int pos_detected_blinks[5];

// For power experiments
boolean power_mode; // if power_mode is true, board accepts directly the blink positions and jumps to dopower() 
float power_params[3]; //[0] is activity ratio, [1] is first command position, [2] is 2nd command position
float aratio; // Activity ratio in %

void setup(void) {

  OSCCONbits.PLLODIV = 0b100;
  OSCCONbits.PLLMULT = 0b111;
  Serial0.begin(115200);  // using hardware uart number 0
  pinMode(LED, OUTPUT); digitalWrite(LED,HIGH);    // blue LED
  pinMode(PGCpin,OUTPUT); digitalWrite(PGCpin,LOW);// used to tell RFduino if we are in bootloader mode NOT IMPLEMENTED
  delay(1000);            // take a break

  startFromScratch();     // initialize OpenBCI, read device IDs
  // you can set EITHER useAccel or useAux to true
  // if you want both, you MUST set and clear one of the variables every sample
  OBCI.useAccel = false;  // option to add/remove accelerometer data to stream          //Mohit : Set 'false' in place of 'true'
  OBCI.useAux = false;     // option to add/remove auxiliary data to stream


  //enable_low_power_mode();
  // Initialize buffer and signal_data
  
  aratio = 0;
  power_mode = true;   // true or false - hardcode
  for (int i=0; i<filter_order; i++){
    buffer_val[i] = 0;
  }
  for (int i=0; i<data_len; i++){
    signal_data[i]=0;
  }
  for (int i=0; i<data_len_max; i++){
    signal_data_max[i]=0;
    signal_data_max2[i]=0;
  }
  for (int i=0; i<m_ind; i++){
    mintab_pos[i]=-10000;
    maxtab_pos[i]=-10000;
    mintab_val[i]=-10000;
    maxtab_val[i]=-10000;
  }
  for (int i=0; i<3; i++){
   power_params[i]=0; 
  }
  mx_ind=0;
  mn_ind=0;
  mnpos = 0;
  mxpos = 0;
  mn = +INFINITY;
  mx = -INFINITY;
  lookformax = true;
  current_val = 0;
  raw_value = 0;
  data_index = 0;
  data_index_max=0;
  time_index = 0;
  buffer_index = 0;
  buffer_avg = 0;
  mytime_millis = 0;
  blink_detected = false;
  total_detected_blinks=0;

  //Serial0.println(F_CPU, DEC);
  enable_low_power_mode();
}

void loop() {
  getData();
}

int charCounter=0;
byte b[4];

void getData() {
  while (Serial0.available()) {
    byte inChar = Serial0.read();
    b[charCounter++] = inChar;
    if (charCounter == 4) {
      raw_value = b[0] | b[1] << 8 | b[2] << 16 | b[3] << 24;
      // Serial0.println(value, DEC) ;
      charCounter = 0;

      if(!power_mode){
        if(data_index_max < data_len_max){  // 0 to 1999 will go in signal_data_max
          signal_data_max[data_index_max] = raw_value;
        }
        else if((data_index_max>=data_len_max)&(data_index_max<2*data_len_max)){
          signal_data_max2[data_index_max-data_len_max] = raw_value;
        }
        data_index_max++;
        data_index_max = (data_index_max) % (2*data_len_max); 
        if(data_index_max==2*data_len_max-1){
          aratio = signal_data_max[0];
          signal_data_max[0] = signal_data_max[1];
          //Serial0.print("Activity Ratio: ");
          //Serial0.println(aratio);
          callLoop(); 
        }
      }
      else{
        power_params[data_index_max] = raw_value;
        data_index_max++;
        if(data_index_max==3){
          aratio = power_params[0];
          dopower();
        }
      }

      
    }
  }
}

void callLoop(void){
  total_detected_blinks=0;
  for(int i=0; i<total_time-batch_time; i++){
    clear_stuff();
    time_index=0;
    blink_detected=false;
    loop_finish = false;
    for(int j=0; j<data_len; j++){
      raw_value = signal_data_max[i*s2rate+j];//(signal_data_max[i*s2rate+j*5+4] + signal_data_max[i*s2rate+j*5+3] + signal_data_max[i*s2rate+j*5+2] + signal_data_max[i*s2rate+j*5+1] + signal_data_max[i*s2rate+j*5])/5;
      BlinkDetector();
      if (blink_detected){
        pos_detected_blinks[total_detected_blinks]=i;
        total_detected_blinks++;
        Serial0.print("nth blink: ");
        Serial0.print(i);
        Serial0.print("..");
        Serial0.println((millis()-mytime_millis));      
        i=i+batch_time-1;
        break;
      }
    }
    if(!blink_detected){
      clear_stuff();
     time_index=0;
     blink_detected=false;
     loop_finish = false;
     for(int j=0; j<data_len; j++){
       raw_value = signal_data_max2[i*s2rate+j];//(signal_data_max[i*s2rate+j*5+4] + signal_data_max[i*s2rate+j*5+3] + signal_data_max[i*s2rate+j*5+2] + signal_data_max[i*s2rate+j*5+1] + signal_data_max[i*s2rate+j*5])/5;
        BlinkDetector();
        if (blink_detected){
          pos_detected_blinks[total_detected_blinks]=i;
          total_detected_blinks++;
          Serial0.print("nth blink: ");
          Serial0.print(i);
          Serial0.print("..");
          Serial0.println((millis()-mytime_millis));      
          i=i+batch_time-1;
          break;
        }
      }      
    }

  }
  //Serial0.println("Finished");
  for (int i=0; i<total_detected_blinks; i++){
    //Serial0.println(pos_detected_blinks[i]);
  }
  //Serial0.println("Power Start");
  dopower();
}
// Doing Processing

boolean presenceofI2(int index){
  int blink_1 = (int) power_params[1];
  int blink_2 = (int) power_params[2];
  if(index==blink_1)
    return true;
  if(index==blink_2)
    return true;
  /*Serial0.println();
  Serial0.println(blink_1);
  Serial0.println(blink_2);
  Serial0.println(index);*/
  
  return false;
}

void dopower(){

  
  
  float hp_secs = aratio/5; // total seconds for high-power mode for a command
  unsigned long hp_delay = (unsigned long) 20000L*5*hp_secs/3;
  //int hp_delay = 20000*hp_secs/3;
  Serial0.println(aratio);
  Serial0.println(hp_delay);
  Serial0.println(power_params[1]);
  Serial0.println(power_params[2]);
  bool command_flag = false;
  //Stage-1
  disable_low_power_mode();
  delay(40000); // 6s
  enable_low_power_mode();
  delay(2000);  // Takes 2s rest after start
  //Stage-2
  for(int i=0; i<(total_time*5); i++){
    int curr_sec = i/5;
    if(!power_mode){
      command_flag = presenceofI(curr_sec) && ((i%5) == 0); // if i in pos_detected_blinks
    }
    else{
      command_flag = presenceofI2(curr_sec) && ((i%5) == 0); // if i in pos_detected_blinks
    }
    
    if(command_flag){
      //Serial0.println("HP");
      pinMode(LED, OUTPUT); digitalWrite(LED,LOW);
      disable_low_power_mode();
      //for (int loop_ind=0; loop_ind<5; loop_ind++){
        delay(hp_delay); 
      //}
      enable_low_power_mode();
      pinMode(LED, OUTPUT); digitalWrite(LED,HIGH);
      i = i + ((int) aratio);
    }
    else{
      Serial0.print('M');
      delay(999);   // Delay for 0.2s
    }
  }
  //Stage-3
  delay(2000);  // Takes 2s rest before end
  disable_low_power_mode();
  delay(40000); // 6s
  enable_low_power_mode();

  /*
  //Old-Code
  disable_low_power_mode();
  delay(40000); // 6s
  enable_low_power_mode();
  delay(2000);
  for(int i=0; i<total_time; i++){
    // if i in pos_detected_blinks
    if(presenceofI(i)){
      int secs = i%5;
      delay(secs*1000);
      disable_low_power_mode();
      delay(20000);
      enable_low_power_mode();
      //Serial0.print('M');
      //delay(1000);
      i=secs+i;
    }

    // if i not in
    else{
      if (i%5 == 0){
        Serial0.print('M');
      }
      delay(1000);
    }
  }
  disable_low_power_mode();
  delay(40000); // 6s
  enable_low_power_mode();
  */
}

boolean presenceofI(int index){
  for (int i=0;i<total_detected_blinks;i++){
    if (pos_detected_blinks[i]==index){
      return true;
    }
  }
  return false;
}

void func_blinkNOTdetected(){
  //already in low power mode - ask radio to shut down for 5 seconds
  //Serial0.print('M');
  delay(1000); // delay 1s
}

void func_blinkdetected(){
  //wait for 5 seconds and turn to high power mode, wait 3 s(wrt to 40MHz) on highpower mode, and then go to low power mode + radio off
  delay(5000);
  disable_low_power_mode();
  delay(20000);  // 20s i.e. 3s*40/6
  enable_low_power_mode(); 
  //Serial0.print('M');
}

void clear_stuff(){
  for (int i=0; i<filter_order; i++){
    buffer_val[i] = 0;
  }
  for (int i=0; i<data_len; i++){
    signal_data[i]=0;
  }
  for (int i=0; i<m_ind; i++){
    mintab_pos[i]=-10000;
    maxtab_pos[i]=-10000;
    mintab_val[i]=-10000;
    maxtab_val[i]=-10000;
  }
  mx_ind=0;
  mn_ind=0;
  mxpos = 0;
  mnpos = 0;
  mn = +INFINITY;
  mx = -INFINITY;
  lookformax = true;
  current_val = 0;
  raw_value = 0;
  data_index = 0;
  data_index_max=0;
  time_index = 0;
  buffer_index = 0;
  buffer_avg = 0;
  mytime_millis = 0;
  blink_detected = false;
}

void BlinkDetector(void){
  //Step 1: Filter
  filter_data();
  //Step 2: Store Data
  store_data();
  //Step 3: Find the Minima/Maxima [PEAKDET code]
  peakdet(current_val);

  
  //Serial0.println(time_index,DEC);
  if (time_index >= data_len-1){
    /*
    Serial0.println("What??");
    Serial0.println(mn_ind,DEC);
    //Serial0.println(mx_ind,DEC);
    for (int i=0; i<10; i++){
      // Serial0.println(signal_data[i]); // check signal data : it is correct
      Serial0.println(mintab_pos[i],DEC);
      Serial0.println(mintab_val[i]);
     
    }
    */
    if (true) {//(time_process<=time_index){
      mytime_millis = millis();
      initiate_processing();
      //Serial0.println("Processing Time");
      //Serial0.println((millis()-mytime_millis));     
      loop_finish = true; 
      return;
    }
  }
  time_index++;
}

void initiate_processing(void){
  // Step1: Finding candidates of first 3 blinks
  //if(!myAns){
  //  return;
  //}
  // Convert min to new array for straight up indexing
  //Serial0.println(time_index);
  int myMintab_pos[m_ind];
  int myMaxtab_pos[m_ind];
  float myMintab_val[m_ind];
  float myMaxtab_val[m_ind];

  int temp_index = (mn_ind + (m_ind-1)) % m_ind;
  int totalMin = 0;
  while((mintab_pos[temp_index]>(time_index-data_len))&(mintab_pos[temp_index]<(time_index+srate))){  // M1 - 50 is replaced with srate
    temp_index = (temp_index + (m_ind-1)) % m_ind;
    totalMin++;
  }
  for (int i=0; i<totalMin; i++){
    myMintab_val[i] = mintab_val[((temp_index+1+i)%m_ind)];
    myMintab_pos[i] = mintab_pos[((temp_index+1+i)%m_ind)];
  }

  //Serial0.println("Hello");

  int temp_index_2 = (mx_ind + (m_ind-1)) % m_ind;
  int totalMax = 0;
  while((maxtab_pos[temp_index_2]>(time_index-data_len))&(maxtab_pos[temp_index_2]<(time_index+srate))){  // M1 - 50 is replaced with srate
    temp_index_2 = (temp_index_2 + (m_ind-1)) % m_ind;
    totalMax++;
  }
  for (int i=0; i<totalMax; i++){
    myMaxtab_val[i] = maxtab_val[((temp_index_2+1+i)%m_ind)];
    myMaxtab_pos[i] = maxtab_pos[((temp_index_2+1+i)%m_ind)];
  }
  /*
  Serial0.println("Hello");
  Serial0.println(totalMin,DEC);
  Serial0.println(totalMax,DEC);
  for (int i=0; i<totalMin; i++){
    Serial0.println(myMintab_val[i]);
    Serial0.println(myMintab_pos[i],DEC);
  }
  for (int i=0; i<totalMax; i++){
    Serial0.println(myMaxtab_val[i]);
    Serial0.println(myMaxtab_pos[i],DEC);
  }
  */
  
  if(totalMin<4){
    //Serial0.println("Total Min was <4");
    return;
  }
  
  int found1=0;
  int cand_1[3];
  for (int i=0; i<3; i++){
    cand_1[i] = 0;
  }
  int i;
  for (i=0; i<totalMin-2; i++){
    if ((myMintab_pos[i+1]-myMintab_pos[i]) > 2*srate){ // change 200 to 2*fs for original   // replaced 200 with 4*srate
       continue;
    }
    if (((myMintab_pos[i+1]-myMintab_pos[i]) < 2*srate)&((myMintab_pos[i+2]-myMintab_pos[i+1]) < 2*srate)){ // replaced 200 with 4*srate
      found1=3;
      cand_1[0] = i;
      cand_1[1]=i+1;
      cand_1[2]=i+2;
      break;
    }
    if (((myMintab_pos[i+1]-myMintab_pos[i]) < 2*srate)&((myMintab_pos[i+2]-myMintab_pos[i+1]) > 2*srate)){ // replaced 200 with 4*srate
      found1=2;
      cand_1[0] = i;
      cand_1[1] = i+1;
      break;
    }
  } 
  //Serial0.println(found1,DEC);
  
  if (found1==0){
    //Serial0.println("Found1==0 found..");
    return;
  }
  //Serial0.println(found1,DEC);

  int min_i = i;
  int max_i = i+1;
  float thresh = 2*myMaxtab_val[max_i] + 2*myMaxtab_val[max_i+1] - myMintab_val[min_i] - 2*myMaxtab_val[min_i+1] - myMintab_val[min_i+2];
  if (found1==2){
    thresh = thresh - myMaxtab_val[max_i+1] - myMintab_val[min_i+2];
    thresh=thresh/6.0;
  }
  else{
    thresh=thresh/8.0;
  }
  //thresh = thresh/2.0;
  //Serial0.println(thresh,DEC);
  
  int found2=0;
  int cand_2[3];
  for (int i=0; i<3; i++){
    cand_2[i] = 0;
  }
  int j;
  for (j=cand_1[found1-1]+1; j<totalMin-2; j++){
    if ((myMintab_pos[j+1]-myMintab_pos[j]) > 2*srate){ // change 200 to 2*fs for original   // replaced 200 with 4*srate
       continue;
    }
    if ((myMintab_pos[j+1]-myMintab_pos[j] < 2*srate)&(myMintab_pos[j+2]-myMintab_pos[j+1] < 2*srate)&(myMaxtab_val[j+1] - myMintab_val[j] > thresh)&(myMaxtab_val[j+2] - myMintab_val[j+1] > thresh)){  // // replaced 200 with 4*srate
      found2=3;
      cand_2[0] = j;
      cand_2[1]=j+1;
      cand_2[2]=j+2;
      break;
    }
    if (((myMintab_pos[j+1]-myMintab_pos[j]) < 2*srate)&((myMintab_pos[j+2]-myMintab_pos[j+1]) > 2*srate)&(myMaxtab_val[j+1] - myMintab_val[j] > thresh)){ // replaced 200 with 4*srate
      found1=2;
      cand_2[0] = j;
      cand_2[1] = j+1;
      break;
    }
  }
  //printf("\n Found2: %d \n",found2);
  //Serial0.println(found2,DEC);
  if (found2==0){
    //printf("Found2==0 found.. \n");
    return;
  } 

  /*
  for(int k=0; k<3; k++){
    Serial0.println(cand_1[k],DEC);
    Serial0.println(cand_2[k],DEC);
  }
  */
  // Checked: Working till here

  int blink_dur = srate/2 -1;  //   corresponds to 0.5s??  // M2 changed to 124 from 24
  int total_cand = 0;
  int final_cand[6];
  for(int k=0;k<6;k++){
    final_cand[k] = 0;
  }
  
  for (int k=0; k<found1; k++){
    int tind = cand_1[k];
    if ((myMintab_pos[cand_1[k]]>(time_index-data_len+blink_dur/2))&(myMintab_pos[cand_1[k]]<(time_index-blink_dur/2))){   // check if error all these variables
      final_cand[total_cand] = cand_1[k];
      total_cand++;
    }
  }
  for (int k=0; k<found2; k++){
    int tind = cand_2[k];
    if ((myMintab_pos[cand_2[k]]>(time_index-data_len+blink_dur/2))&(myMintab_pos[cand_2[k]]<(time_index-blink_dur/2))){   // check if error all these variables
      final_cand[total_cand] = cand_2[k];
      total_cand++;
    }
  } 
  //Serial0.println("Total Candidates:");
  //Serial0.println(total_cand,DEC);
  //Checked: Working Till here
  
  float corr_matrix[36];
  float powr_matrix[36];
  float copw_matrix[36];

  for (int k=0; k<36; k++){
    corr_matrix[k]=1;
    powr_matrix[k]=1;
    copw_matrix[k]=1;
  }

  //Serial0.println("Printing matrices");
  /*
  int k1=0;
  int k2=1;
  float test = 0;
  float *trym = &test;
  int temp_start = myMintab_pos[final_cand[k1]]-blink_dur/2;
  int temp_end = myMintab_pos[final_cand[k1]]+blink_dur/2;
  int match_start = myMintab_pos[final_cand[k2]] - blink_dur/2;
  int match_end = myMintab_pos[final_cand[k2]] + blink_dur/2;
  Serial0.println(mean(temp_start,temp_end));
  Serial0.println(stdv(temp_start,temp_end));
  Serial0.println(corr(temp_start,match_start,temp_end,match_end,trym));
  Serial0.println(mean(match_start,match_end));
  Serial0.println(stdv(match_start,match_end));  
  Serial0.println(*trym);
  */
  for (int k1=0;k1<total_cand;k1++){
    for (int k2=k1+1;k2<total_cand;k2++){
      int temp_start = myMintab_pos[final_cand[k1]]-blink_dur/2;
      int temp_end = myMintab_pos[final_cand[k1]]+blink_dur/2;
      int match_start = myMintab_pos[final_cand[k2]] - blink_dur/2;
      int match_end = myMintab_pos[final_cand[k2]] + blink_dur/2;
      float *pw = &powr_matrix[k1*total_cand+k2];
      corr_matrix[k1*total_cand+k2] = corr(temp_start,match_start,temp_end,match_end,pw);
      corr_matrix[k2*total_cand+k1] = corr_matrix[k1*total_cand+k2];
      powr_matrix[k2*total_cand+k1] = powr_matrix[k1*total_cand+k2];
      copw_matrix[k1*total_cand+k2] = corr_matrix[k1*total_cand+k2]/sqrt(powr_matrix[k1*total_cand+k2]);
      copw_matrix[k2*total_cand+k1] = copw_matrix[k1*total_cand+k2];
    }
  }
  /*
  Serial0.println("Printing copw matrix");
  for (int k1=0;k1<total_cand;k1++){
    for (int k2=0;k2<total_cand;k2++){
      Serial0.println(copw_matrix[k1*total_cand+k2]);
    }
  }*/
  //Serial0.println("Printign copw sum");
  float copw_sum[total_cand];
  for (int k1=0;k1<total_cand;k1++){
    copw_sum[k1]=0;
    for (int k2=0;k2<total_cand;k2++){
      //Serial0.println(copw_matrix[k1*total_cand+k2]);
      copw_sum[k1] = copw_sum[k1] + copw_matrix[k1*total_cand+k2];
    }
    //Serial0.println(copw_sum[k1]);
  }

  // Checked Working Till Here
  
  float myMax_val[3];
  int myMax_ind[3];

  for(int k=0;k<3;k++){
    myMax_val[k] = -INFINITY;
    myMax_ind[k] = -1;
  }
  for(int k=0;k<total_cand;k++){
    if (copw_sum[k]>myMax_val[0]){
      myMax_val[0]=copw_sum[k];
      myMax_ind[0]=k;
    }
  }
  for(int k=0;k<total_cand;k++){
    if ((copw_sum[k]>myMax_val[1])&(copw_sum[k]<myMax_val[0])){
      myMax_val[1]=copw_sum[k];
      myMax_ind[1]=k;
    }
  }
  for(int k=0;k<total_cand;k++){
    if ((copw_sum[k]>myMax_val[2])&(copw_sum[k]<myMax_val[1])){
      myMax_val[2]=copw_sum[k];
      myMax_ind[2]=k;
    }
  }/*
  Serial0.println("Printing Max Indices");
  Serial0.println(myMax_ind[0],DEC);
  Serial0.println(myMax_ind[1],DEC);
  Serial0.println(myMax_ind[2],DEC);
  */
  //Serial0.println("Print copw_matrix values");
  boolean final_blinks[total_cand];
  for (int k=0; k<total_cand; k++){
    final_blinks[k]=false;
    int new_index1 = myMax_ind[0]*total_cand+k;
    int new_index2 = myMax_ind[1]*total_cand+k;
    int new_index3 = myMax_ind[2]*total_cand+k;
    //Serial0.println(copw_matrix[new_index1]);
    //Serial0.println(copw_matrix[new_index2]);
    //Serial0.println(copw_matrix[new_index3]);
    
    if ((copw_matrix[new_index1]>0.5)|(copw_matrix[new_index2]>0.5)|(copw_matrix[new_index3]>0.5)){
      final_blinks[k] = true;
    }
    //Serial0.println(final_blinks[k]);
  }

  int total_blinks=0;
  for (int k=0; k<6; k++){
    if (final_blinks[k]==true){
      total_blinks++;
    }
  }
  //Serial0.println("total_blinks");
  //Serial0.println(total_blinks,DEC);

  if (total_blinks<4){
    //Serial0.println("BLINK NOT DETECTED");
    return;
  }
  
  double checkMax_val=-INFINITY;
  double checkMax_ind=-1;  
  int blinkpos[total_blinks];
  for (int k=0;k<total_blinks;k++){
    blinkpos[k] = myMintab_pos[final_cand[k]];
  }
  for (int k=0;k<total_blinks-1;k++){
    if ((blinkpos[k+1]-blinkpos[k])>checkMax_val){
      checkMax_val = (blinkpos[k+1]-blinkpos[k]);
      checkMax_ind = k;
    }
  }
  //Serial0.println("checkmax_index");
  //Serial0.println(checkMax_ind);
  if((checkMax_ind==0)|(checkMax_ind==total_blinks-2)){
     //Serial0.println("BLINK NOT DETECTED: Detected blinks does not have space in between");
     return;
  }

  //Serial0.println("BLINK DETECTED");
  blink_detected = true;
  //Serial0.println("Processing Time");
  //Serial0.println((millis()-mytime_millis));  
  //Serial0.println("BLINK POSITIONS");
  //for (int k=0;k<total_blinks-1;k++){
  //  Serial0.println(blinkpos[k],DEC);
  //}
  //Serial0.println("--- END ---");

  // Check : WORKS!!! :)
  
}

float mean(int start_pos, int end_pos){
  int N = end_pos - start_pos+1;
  float sum=0;
  for (int ind=start_pos; ind<=end_pos; ind++){
    sum = sum+signal_data[ind];
  }
  sum = sum/(float)N;
  return sum;
}

float stdv(int start_pos, int end_pos){
  int N = end_pos - start_pos+1;
  float avg = mean(start_pos,end_pos);
  float sum=0;
  for (int ind=start_pos; ind<=end_pos; ind++){
    sum = sum + pow((signal_data[ind] - avg), 2);
  }
  sum = sum/(float)(N-1);
  return sqrt(sum);
}

float corr(int start_1, int start_2, int end_1, int end_2, float *pw){
  int N = end_1-start_1+1;
  if(N!=(end_2-start_2+1)){
    Serial0.println("ERROR: Length not same");
    return 0;
  }
  float avg_1 = mean(start_1,end_1);
  float avg_2 = mean(start_2,end_2);
  float std_1 = stdv(start_1,end_1);
  float std_2 = stdv(start_2,end_2);
  float coeff = 0;
  for (int ind=0;ind<N;ind++){
    coeff = coeff + (signal_data[start_1+ind]-avg_1)*(signal_data[start_2+ind]-avg_2);
  }
  coeff = coeff/(float)((N-1)*std_1*std_2);
  if (std_1>std_2){
    *pw = std_1/std_2;
  }
  else{
    *pw = std_2/std_1;
  }
  return coeff;
}

/*
boolean check_minSize(void){
  int mintab_index_minus3 = (mn_ind + 7)%10;
  if (mintab_pos[mintab_index_minus3]<(time_index-data_len)){
    time_process = time_process+time_shift;
    return false;
  }

  //Get Candidate Signals if there
  int found1=0;
  
  
}
*/
void filter_data(void){
  buffer_avg = buffer_avg + raw_value/filter_order - buffer_val[buffer_index]/filter_order;
  buffer_val[buffer_index] = raw_value;
  buffer_index++;
  buffer_index = buffer_index % filter_order;  
}

void store_data(void){
  current_val = buffer_avg;
  signal_data[data_index] = current_val;
  data_index++;
  data_index = data_index % data_len; 
}

void peakdet(float value){
  if (value > mx){
    mx = value;
    mxpos=time_index;
  }
  if (value < mn){
    mn = value;
    mnpos=time_index;
  }
  if (lookformax) {
    if (value < mx-delta){
      maxtab_val[mx_ind] = mx;
      maxtab_pos[mx_ind] = mxpos;
      mn = value;
      mnpos = time_index;
      mx_ind = (mx_ind+1)%m_ind;
      lookformax = false;
    }
  }
  else{
    if (value > mn+delta){
      mintab_val[mn_ind] = mn;
      mintab_pos[mn_ind] = mnpos;
      mx = value;
      mxpos = time_index;
      mn_ind = (mn_ind+1)%m_ind;
      lookformax = true;
    }        
  }
}

/*
//Define Variables
int NBlink = 12;
float signal_1[2000];
int blink_duration_f = 129;
int myInd = 0;

void gatherData(void) {
  signal_1[myInd]= buffer_sum/filter_order; // contains filtered data
  if (myInd==data_len-1){
   processData(); 
   myInd=0;
  }
  myInd++;
}

void processData(void) {
  // Step 1: Peak Detection
}
*/

void blinkLED(void) {
  pinMode(LED, OUTPUT); digitalWrite(LED,HIGH);
  delay(1000);
  pinMode(LED, OUTPUT); digitalWrite(LED,LOW);
  delay(1000);
  pinMode(LED, OUTPUT); digitalWrite(LED,HIGH);
  blinkLED();
}

// Now Original OpenBCI Code



// some variables to help find 'burger protocol' commands
int plusCounter = 0;
char testChar;
unsigned long commandTimer;

void eventSerial(){
  while(Serial0.available()){
    char inChar = (char)Serial0.read();

    if(plusCounter == 1){  // if we have received the first 'bun'
      testChar = inChar;   // this might be the 'patty', stop laughing
      plusCounter++;       // get ready to look for another 'bun'
      commandTimer = millis();  // don't wait too long!
    }

    if(inChar == '+'){  // if we see a 'bun' on the serial
      plusCounter++;    // make a note of it
      if(plusCounter == 3){  // looks like we got a command character
        if(millis() - commandTimer < 5){  // if it's not too late,
          if(getChannelSettings){ // if we just got an 'x' expect channel setting parameters
            loadChannelSettings(testChar);  // go get em!
          }else if(getLeadOffSettings){  // if we just got a 'z' expect lead-off setting parameters
            loadLeadOffSettings(testChar); // go get em!
          }else{
            getCommand(testChar);    // decode the command
          }
        }
        plusCounter = 0;  // get ready for the next one
      }
    }
  }
}


void getCommand(char token){
    switch (token){
//TURN CHANNELS ON/OFF COMMANDS
      case '1':
        changeChannelState_maintainRunningState(1,DEACTIVATE); break;
      case '2':
        changeChannelState_maintainRunningState(2,DEACTIVATE); break;
      case '3':
        changeChannelState_maintainRunningState(3,DEACTIVATE); break;
      case '4':
        changeChannelState_maintainRunningState(4,DEACTIVATE); break;
      case '5':
        changeChannelState_maintainRunningState(5,DEACTIVATE); break;
      case '6':
        changeChannelState_maintainRunningState(6,DEACTIVATE); break;
      case '7':
        changeChannelState_maintainRunningState(7,DEACTIVATE); break;
      case '8':
        changeChannelState_maintainRunningState(8,DEACTIVATE); break;
      case '!':
        changeChannelState_maintainRunningState(1,ACTIVATE); break;
      case '@':
        changeChannelState_maintainRunningState(2,ACTIVATE); break;
      case '#':
        changeChannelState_maintainRunningState(3,ACTIVATE); break;
      case '$':
        changeChannelState_maintainRunningState(4,ACTIVATE); break;
      case '%':
        changeChannelState_maintainRunningState(5,ACTIVATE); break;
      case '^':
        changeChannelState_maintainRunningState(6,ACTIVATE); break;
      case '&':
        changeChannelState_maintainRunningState(7,ACTIVATE); break;
      case '*':
        changeChannelState_maintainRunningState(8,ACTIVATE); break;
      case 'q':
        changeChannelState_maintainRunningState(9,DEACTIVATE); break;
      case 'w':
        changeChannelState_maintainRunningState(10,DEACTIVATE); break;
      case 'e':
        changeChannelState_maintainRunningState(11,DEACTIVATE); break;
      case 'r':
        changeChannelState_maintainRunningState(12,DEACTIVATE); break;
      case 't':
        changeChannelState_maintainRunningState(13,DEACTIVATE); break;
      case 'y':
        changeChannelState_maintainRunningState(14,DEACTIVATE); break;
      case 'u':
        changeChannelState_maintainRunningState(15,DEACTIVATE); break;
      case 'i':
        changeChannelState_maintainRunningState(16,DEACTIVATE); break;
      case 'Q':
        changeChannelState_maintainRunningState(9,ACTIVATE); break;
      case 'W':
        changeChannelState_maintainRunningState(10,ACTIVATE); break;
      case 'E':
        changeChannelState_maintainRunningState(11,ACTIVATE); break;
      case 'R':
        changeChannelState_maintainRunningState(12,ACTIVATE); break;
      case 'T':
        changeChannelState_maintainRunningState(13,ACTIVATE); break;
      case 'Y':
        changeChannelState_maintainRunningState(14,ACTIVATE); break;
      case 'U':
        changeChannelState_maintainRunningState(15,ACTIVATE); break;
      case 'I':
        changeChannelState_maintainRunningState(16,ACTIVATE); break;

// TEST SIGNAL CONTROL COMMANDS
      case '0':
        activateAllChannelsToTestCondition(ADSINPUT_SHORTED,ADSTESTSIG_NOCHANGE,ADSTESTSIG_NOCHANGE); break;
      case '-':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_SLOW); break;
      case '=':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); break;
      case 'p':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_DCSIG); break;
      case '[':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_SLOW); break;
      case ']':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_FAST); break;

// SD CARD COMMANDS
    //    5min     15min    30min    1hr      2hr      4hr      12hr     24hr    512blocks
      case 'A': case'S': case'F': case'G': case'H': case'J': case'K': case'L': case 'a':
        fileSize = token; SDfileOpen = setupSDcard(fileSize); //
        break;
      case 'j': // close the file, if it's open
        if(SDfileOpen){ SDfileOpen = closeSDfile(); }
        break;

// CHANNEL SETTING COMMANDS
      case 'x':  // expect 6 parameters
        if(!is_running) {Serial0.println("ready to accept new channel settings");}
        channelSettingsCounter = 0;
        getChannelSettings = true; break;
      case 'X':  // latch channel settings
        if(!is_running) {Serial0.println("updating channel settings");}
        writeChannelSettings_maintainRunningState(currentChannelToSet); break;
      case 'd':  // reset all channel settings to default
        if(!is_running) {Serial0.println("updating channel settings to default");}
        setChannelsToDefaultSetting(); break;
      case 'D':  // report the default settings
        sendDefaultChannelSettings(); break;

// LEAD OFF IMPEDANCE DETECTION COMMANDS
      case 'z':  // expect 2 parameters
        if(!is_running) {Serial0.println("ready to accept new impedance detect settings");}
        leadOffSettingsCounter = 0;  // reset counter
        getLeadOffSettings = true;
        break;
      case 'Z':  // latch impedance parameters
        if(!is_running) {Serial0.println("updating impedance detect settings");}
        changeChannelLeadOffDetect_maintainRunningState(currentChannelToSet);
        break;

// DAISY MODULE COMMANDS
      case 'c':  // use 8 channel mode
        if(OBCI.daisyPresent){ OBCI.removeDaisy(); }
        outputType = OUTPUT_8_CHAN;
        break;
      case 'C':  // use 16 channel mode
        if(OBCI.daisyPresent == false){OBCI.attachDaisy();}
        if(OBCI.daisyPresent){
          Serial0.print("16"); outputType = OUTPUT_16_CHAN;
        }else{
          Serial0.print("8"); outputType = OUTPUT_8_CHAN;
        }
        sendEOT();
        break;

// STREAM DATA AND FILTER COMMANDS
      case 'b':  // stream data
        if(SDfileOpen) stampSD(ACTIVATE);                     // time stamp the start time
        if(OBCI.useAccel){OBCI.enable_accel(RATE_25HZ);}      // fire up the accelerometer if you want it
        startRunning(outputType);                             // turn on the fire hose
        break;
     case 's':  // stop streaming data
        if(SDfileOpen) stampSD(DEACTIVATE);       // time stamp the stop time
        if(OBCI.useAccel){OBCI.disable_accel();}  // shut down the accelerometer if you're using it
        stopRunning();
        break;
      case 'f':
         useFilters = true;
         break;
      case 'g':
         useFilters = false;
         break;

//  INITIALIZE AND VERIFY
      case 'v':
         startFromScratch();  // initialize ADS and read device IDs
         break;
//  QUERY THE ADS AND ACCEL REGITSTERS
     case '?':
        printRegisters();     // print the ADS and accelerometer register values
        break;
     default:
       break;
     }
  }// end of getCommand

void sendEOT(){
  Serial0.print("$$$");  // shake hands with the controlling program
}

void loadChannelSettings(char c){

  if(channelSettingsCounter == 0){  // if it's the first byte in this channel's array, this byte is the channel number to set
    currentChannelToSet = getChannelNumber(c); // we just got the channel to load settings into (shift number down for array usage)
    channelSettingsCounter++;
    if(!is_running) {
      Serial0.print("load setting ");
      Serial0.print("for channel ");
      Serial0.println(currentChannelToSet+1,DEC);
    }
    return;
  }
//  setting bytes are in order: POWER_DOWN, GAIN_SET, INPUT_TYPE_SET, BIAS_SET, SRB2_SET, SRB1_SET
  if(!is_running) {
    Serial0.print(channelSettingsCounter-1);
    Serial0.print(" with "); Serial0.println(c);
  }
  c -= '0';
  if(channelSettingsCounter-1 == GAIN_SET){ c <<= 4; }
  OBCI.channelSettings[currentChannelToSet][channelSettingsCounter-1] = c;
  channelSettingsCounter++;
  if(channelSettingsCounter == 7){  // 1 currentChannelToSet, plus 6 channelSetting parameters
    if(!is_running) Serial0.print("done receiving settings for channel ");Serial0.println(currentChannelToSet+1,DEC);
    getChannelSettings = false;
  }
}

void writeChannelSettings_maintainRunningState(char chan){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  stopRunning();                   //must stop running to change channel settings

  OBCI.writeChannelSettings(chan+1);    // change the channel settings on ADS

  if (is_running_when_called == true) {
    startRunning(cur_outputType);  //restart, if it was running before
  }
}

void setChannelsToDefaultSetting(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  stopRunning();  //must stop running to change channel settings

  OBCI.setChannelsToDefault();   // default channel settings

  if (is_running_when_called == true) {
    startRunning(cur_outputType);  //restart, if it was running before
  }
}

void loadLeadOffSettings(char c){
   if(leadOffSettingsCounter == 0){  // if it's the first byte in this channel's array, this byte is the channel number to set
    currentChannelToSet = getChannelNumber(c); // we just got the channel to load settings into (shift number down for array usage)
    if(!is_running) Serial0.print("changing LeadOff settings for channel "); Serial0.println(currentChannelToSet+1,DEC);
    leadOffSettingsCounter++;
    return;
  }
//  setting bytes are in order: PCHAN, NCHAN
  if(!is_running) {
    Serial0.print("load setting "); Serial0.print(leadOffSettingsCounter-1);
    Serial0.print(" with "); Serial0.println(c);
  }
  c -= '0';
  OBCI.leadOffSettings[currentChannelToSet][leadOffSettingsCounter-1] = c;
  leadOffSettingsCounter++;
  if(leadOffSettingsCounter == 3){  // 1 currentChannelToSet, plus 2 leadOff setting parameters
    if(!is_running) Serial0.print("done receiving leadOff settings for channel ");Serial0.println(currentChannelToSet+1,DEC);
    getLeadOffSettings = false; // release the serial COM
  }
}

char getChannelNumber(char n){
  if(n > '0' && n < '9'){
    n -= '1';
  }
  switch(n){
    case 'Q':
      n = 0x08; break;
    case 'W':
      n = 0x09; break;
    case 'E':
      n = 0x0A; break;
    case 'R':
      n = 0x0B; break;
    case 'T':
      n = 0x0C; break;
    case 'Y':
      n = 0x0D; break;
    case 'U':
      n = 0x0E; break;
    case 'I':
      n = 0x0F; break;
    default: break;
  }
  return n;
}

void changeChannelState_maintainRunningState(byte chan, int start)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;

  //must stop running to change channel settings
  stopRunning();
  if (start == 1) {
    OBCI.activateChannel(chan);
  } else if (start == 0){
    OBCI.deactivateChannel(chan);
  }
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

void activateAllChannelsToTestCondition(byte testInputCode, byte amplitudeCode, byte freqCode)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  //must stop running to change channel settings
  stopRunning(); delay(10);

  //set the test signal to the desired state
  OBCI.configureInternalTestSignal(amplitudeCode,freqCode);
  //change input type settings for all channels
  OBCI.changeInputType(testInputCode);

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

int changeChannelLeadOffDetect_maintainRunningState(char chan)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;

  //must stop running to change channel settings
  stopRunning();

  OBCI.changeChannelLeadOffDetect(chan);

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

void sendDefaultChannelSettings(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;

  OBCI.reportDefaultChannelSettings();
  sendEOT();
  delay(10);

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

boolean stopRunning(void) {
  if(is_running){
    OBCI.stopStreaming();  // stop the data acquisition, turn off accelerometer
    is_running = false;
    }
    return is_running;
  }

boolean startRunning(int OUT_TYPE) {
  if(!is_running){
    outputType = OUT_TYPE;
    OBCI.startStreaming();  // start the data acquisition, turn on accelerometer
    is_running = true;
  }
    return is_running;
}

void printRegisters(){

  if(!is_running){
    // print the ADS and LIS3DH registers
    OBCI.printAllRegisters();
    sendEOT();
    delay(20);
  }

}

void startFromScratch(){
  if(!is_running){
    OBCI.initialize();     // initializes accelerometer and on-board ADS and on-daisy ADS if present
    delay(500);
    Serial0.println("OpenBCI V3 16 channel");
    OBCI.configureLeadOffDetection(LOFF_MAG_6NA, LOFF_FREQ_31p2HZ);
    Serial0.print("On Board ADS1299 Device ID: 0x"); Serial0.println(OBCI.ADS_getDeviceID(ON_BOARD),HEX);
    if(OBCI.daisyPresent){  // library will set this in initialize() if daisy present and functional
      Serial0.print("On Daisy ADS1299 Device ID: 0x"); Serial0.println(OBCI.ADS_getDeviceID(ON_DAISY),HEX);
    }
    Serial0.print("LIS3DH Device ID: 0x"); Serial0.println(OBCI.LIS3DH_getDeviceID(),HEX);
    sendEOT();
  }
}

void enable_low_power_mode(){
  OSCCONbits.PLLODIV = 0b100;
  OSCCONbits.PLLMULT = 0b111;
  changeChannelState_maintainRunningState(3,DEACTIVATE);
  changeChannelState_maintainRunningState(4,DEACTIVATE);
  changeChannelState_maintainRunningState(5,DEACTIVATE);
  changeChannelState_maintainRunningState(6,DEACTIVATE);
  changeChannelState_maintainRunningState(7,DEACTIVATE);
  changeChannelState_maintainRunningState(8,DEACTIVATE);
}

void disable_low_power_mode(){
  changeChannelState_maintainRunningState(3,ACTIVATE);
  changeChannelState_maintainRunningState(4,ACTIVATE);
  changeChannelState_maintainRunningState(5,ACTIVATE);
  changeChannelState_maintainRunningState(6,ACTIVATE);
  changeChannelState_maintainRunningState(7,ACTIVATE);
  changeChannelState_maintainRunningState(8,ACTIVATE);
  OSCCONbits.PLLODIV = 0b001;
  OSCCONbits.PLLMULT = 0b101;
}



// end







