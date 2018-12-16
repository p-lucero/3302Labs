/*******************************************
  IR Remote
********************************************/
#include <Sparki.h> // include the sparki library
#include "final_project.h"

// remote diagram
// /------^-----\
// |            |
// | 69  70  71 |
// | 68  64  67 |
// |  7  21   9 |
// | 22  25  13 |
// | 12  24  94 |
// |  8  28  90 |
// | 66  82  74 |
// \____________/

int numpad[10] = {25, 12, 24, 94, 8, 28, 90, 66, 82, 74}; // TODO this could go into progmem but would require rewriting find_num

// void flash(byte r, byte g, byte b)
// {
//   for (byte i = 0; i<1000; i++)
//   {
//     if (i % 2 == 0)
//       {sparki.RGB(r, g, b);}
//     else
//       {sparki.RGB(r, g, b);} // FIXME? this is the same color...
//   }
// }

int find_num(int code)
{
  for (byte i = 0; i<10; i++)
  {
    if (numpad[i] == code)
      {return i;}
  }
  return (-1);
}

void useElevator(){
  // block until Sparki receives a command that tells him he's in the elevator
  moveStop();
  sparki.RGB(RGB_GREEN);
  int code = 0;
  while (code != 9){
    delay(50);
    code = sparki.readIR();
  }
  sparki.RGB(RGB_OFF);
}

byte differentiateObject(int flame_detected){
  // block until Sparki receives a command that tells him what object he's looking at
  // returns 0 for false alarm, 255 for person, and the cell type for if it's an actual object
  if (flame_detected == 0){
    sparki.beep();
    return FIRE;
  }
  moveStop();
  int code = 0;
  int code_num = 0;
  byte retval = 128;
  sparki.RGB(RGB_RED);
  while (true){
    delay(50);
    code = sparki.readIR();
    code_num = find_num(code);
    if (code_num != -1){
      retval = code_num; // user has inputted a number, infer to be cell type (covers false alarm)
      break;
    }
    else if (code == 13){
      retval = 255;
      break;
    }
  }
  delay(500);
  sparki.RGB(RGB_OFF);
  return retval;
}

byte* getNextTarget(){
  // block until Sparki receives commands detailing his next target or if he's done
  // returns NULL if done, or an array of three bytes representing the next coordinates to search
  moveStop();
  sparki.RGB(RGB_BLUE);
  int code = 0;
  int code_num = 0;
  byte* retval = NULL;
  while(true){
    delay(50);
    code = sparki.readIR();
    code_num = find_num(code);
    if (code == 22)
      break; // return NULL, there is no next target
    else if (code == 13){ // find out the next target
      retval = new byte[3];
      byte counter = 0;
      while(counter < 3){
        delay(50);
        code = code_num = -1;
        code = sparki.readIR();
        code_num = find_num(code);
        if (code_num != -1){
          retval[counter] = code_num;
          counter++;
          sparki.beep();
          delay(1000); // don't hold the button for more than a second, or bad things happen.
        }
      }
      break;
    }
  }
  sparki.RGB(RGB_OFF);
  return retval;
}