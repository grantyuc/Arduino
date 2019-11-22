#include "pitches.h"


// notes in the melody:
int melody[] = {
0,NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6};
int melody2[] = {
0,NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6, NOTE_D6, NOTE_E6, NOTE_F6, NOTE_G6, NOTE_A6, NOTE_B6, NOTE_C7};
int noteIndex=0;
int duration = 500;  // 500 miliseconds
int lengthBee = 64, lengthStar = 48, lengthButterflyLove = 224, lengthLeaveTruth = 640;
int bee[]={5,3,3,0,4,2,2,0,1,2,3,4,5,5,5,0,5,3,3,0,4,2,2,0,1,3,5,5,3,0,0,0,
           2,2,2,2,2,3,4,0,3,3,3,3,3,4,5,0,5,3,3,0,4,2,2,0,1,3,5,5,1,0,0,0};
int star[]={1,1,5,5,6,6,5,0,4,4,3,3,2,2,1,0,5,5,4,4,3,3,2,0,5,5,4,4,3,3,2,0,1,1,5,5,6,6,5,0,4,4,3,3,2,2,1,0};
int butterflyLove[]={3,3,3,2,3,0,0,0,2,3,2,2,-6,0,-6,-7,1,0,2,1,-7,0,-6,-5,-6,0,0,0,0,0,0,0,
                     3,3,3,2,3,0,0,6,5,6,5,5,2,0,2,3,4,0,5,4,3,0,2,1,3,0,0,0,0,0,0,3,
                     6,0,7,6,5,0,0,3,5,0,0,0,0,0,3,5,2,0,6,5,3,0,2,2,3,0,0,0,0,0,0,0,
                     2,0,6,6,0,0,0,0,1,0,6,6,0,0,6,7,8,0,7,6,7,0,6,7,3,0,0,0,0,0,0,3,
                     6,0,7,6,5,0,0,3,5,0,0,0,0,0,4,5,6,0,7,6,7,0,6,7,3,0,0,0,0,0,0,0,
                     2,0,6,6,0,0,0,0,1,0,6,6,0,0,6,7,8,0,7,6,7,0,5,0,6,0,0,0,0,0,0,0,
                     10,9,9,8,8,7,7,6,8,7,7,6,6,5,5,4,6,5,5,4,4,3,3,2,3,0,0,0,0,0,0,0};
int leaveTruth[]={5, -6, 1, 5, 0, -6, 1, 5, 0, -6, 1, 5, 0, -6, 0, 1, 0, 1, -6, 1, 0, 1, -6, 1, 0, 2, -6, 3, 0, 4, 0, 0, 12, 6, 8, 12, 0, 6, 8, 12, 0, 6, 8, 12, 0, 6, 10, 11, 0, 12, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 99, 99, 99, 99, 99, 5, 5, 5, 5, 2, 2, 2, 99, 5, 5, 5, 5, 2, 3, 4, 0, 3, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 99, 99, 99, 99, 99, 5, 5, 5, 5, 2, 2, 2, 99, 5, 5, 5, 5, 2, 3, 4, 0, 5, 0, 0, 99, 2, 3, 4, 0, 0, 0, 0, -6, -7, 15, 0, 0, 0, 0, 99, 4, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 4, 3, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 99, 5, 5, 5, 5, 2, 2, 2, 99, 5, 5, 5, 5, 2, 3, 4, 0, 3, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 99, 99, 99, 99, 99, 12, 12, 12, 12, 9, 9, 9, 99, 12, 12, 12, 12, 9, 10, 11, 0, 10, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 99, 99, 99, 99, 99, 12, 12, 12, 12, 9, 9, 9, 99, 12, 12, 12, 12, 9, 10, 12, 0, 13, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 99, 99, 12, 12, 99, 99, 8, 8, 99, 99, 12, 12, 99, 99, 9, 9, 99, 99, 8, 9, 10, 0, 10, 10, 0, 9, 0, 8, 10, 0, 12, 12, 99, 99, 12, 12, 99, 99, 8, 8, 99, 99, 12, 12, 99, 99, 9, 9, 99, 10, 11, 10, 0, 0, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 99, 99, 12, 12, 99, 99, 8, 8, 99, 99, 12, 12, 99, 99, 9, 9, 99, 99, 8, 9, 10, 0, 10, 10, 10, 15, 0, 0, 14, 0, 0, 12, 99, 99, 12, 12, 99, 99, 8, 8, 99, 99, 12, 12, 99, 99, 9, 9, 99, 8, 9, 8, 0, 0, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 10, 9, 8, 7, 8, 0, 0, 0, 0, 8, 0, 5, 9, 0, 0, 0, 10, 9, 8, 7, 8, 0, 0, 0, 0, 8, 0, 8, 7, 0, 0, 0, 10, 9, 8, 7, 8, 0, 0, 0, 0, 8, 0, 8, 9, 0, 0, 13, 0, 12, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 10, 9, 8, 7, 8, 0, 0, 0, 0, 8, 0, 5, 9, 0, 0, 0, 10, 9, 8, 7, 8, 0, 0, 0, 0, 8, 0, 10, 10, 0, 0, 0, 10, 9, 8, 7, 8, 0, 0, 0, 0, 8, 0, 5, 9, 0, 0, 13, 0, 12, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 99, 99, 12, 12, 99, 99, 8, 8, 99, 99, 12, 12, 99, 99, 9, 9, 99, 99, 8, 9, 10, 0, 10, 10, 0, 9, 0, 8, 10, 0, 12, 12, 99, 99, 12, 12, 99, 99, 8, 8, 99, 99, 12, 12, 99, 99, 9, 9, 99, 10, 11, 10, 0, 0, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 99, 99, 12, 12, 99, 99, 8, 8, 99, 99, 12, 12, 99, 99, 9, 9, 99, 99, 8, 9, 10, 0, 10, 10, 10, 15, 0, 0, 14, 0, 0, 12, 99, 99, 12, 12, 99, 99, 8, 8, 99, 99, 12, 12, 99, 99, 9, 9, 99, 8, 9, 8, 0, 0, 0, 0, 0, 0, 0, 0, 99, 99, 99, 99, 99, 99, 12, 12};

void setup() {
   pinMode(13,OUTPUT);
}

void loop() { 
/*  for (int thisNote = 0; thisNote < lengthBee; thisNote++) {
            int count=1;
            for(int i=1;thisNote+1<lengthBee;i++){
            if(bee[thisNote+1]==0){
              thisNote++;
              count++;}
            else break;}
             duration = 450;
             duration*=count;
  	    tone(13, melody[bee[thisNote+1-count]], duration);
	     
	    // 間隔一段時間後再播放下一個音階
	    delay(duration+count*50);
	  }
	   
	  // 1.5秒後重新播放
	  delay(1500);

   for (int thisNote = 0; thisNote < lengthStar; thisNote++) {
            int count=1;
            for(int i=1;thisNote+1<lengthStar;i++){
            if(star[thisNote+1]==0){
              thisNote++;
              count++;}
            else break;}
             duration = 350;
             duration*=count;
  	    tone(13, melody[star[thisNote+1-count]], duration);
	     
	    // 間隔一段時間後再播放下一個音階
	    delay(duration+count*50);
	  }
          delay(1500);
*/          
   for (int thisNote = 0; thisNote < lengthButterflyLove; thisNote++) {
            int count=1;
            for(int i=1;thisNote+1<lengthButterflyLove;i++){
            if(butterflyLove[thisNote+1]==0){
              thisNote++;
              count++;}
            else break;}
            duration = 320;
            duration*=count;
            if(butterflyLove[thisNote+1-count]==0) noteIndex=0;
            if(butterflyLove[thisNote+1-count]>0) noteIndex=butterflyLove[thisNote+1-count]+7;
            else if(butterflyLove[thisNote+1-count]<0) noteIndex=-butterflyLove[thisNote+1-count];
  	    tone(13, melody2[noteIndex], duration);
	     
	    // 間隔一段時間後再播放下一個音階
	    delay(duration+count*50);
	  }
          delay(1500);
          
/*   for (int thisNote = 0; thisNote < lengthLeaveTruth; thisNote++) {
            int count=1;
            for(int i=1;thisNote+1<lengthLeaveTruth;i++){
            if(leaveTruth[thisNote+1]==0){
              thisNote++;
              count++;}
            else break;}
            duration = 150;
            duration*=count;
            if(leaveTruth[thisNote+1-count]==0) noteIndex=0;
            if(leaveTruth[thisNote+1-count]>0) noteIndex=leaveTruth[thisNote+1-count]+7;
            else if(leaveTruth[thisNote+1-count]<0) noteIndex=-leaveTruth[thisNote+1-count];
  	    tone(13, melody2[noteIndex], duration);
	     
	    // 間隔一段時間後再播放下一個音階
	    delay(duration+count*50);
	  }
          delay(1500);*/
	}

