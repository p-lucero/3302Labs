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

int numpad[10] PROGMEM = {25, 12, 24, 94, 8, 28, 90, 66, 82, 74};

void flash(byte r, byte g, byte b)
{
  for (byte i = 0; i<1000; i++)
  {
    if (i % 2 == 0)
      {sparki.RGB(r, g, b);}
    else
      {sparki.RGB(r, g, b);} // FIXME? this is the same color...
  }
}

int find_num(byte code)
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

}

byte differentiateObject(){
  // block until Sparki receives a command that tells him what object he's looking at
  // returns 0 for false alarm, 255 for person, and the cell type for if it's an actual object

}

byte* getNextTarget(){
  // block until Sparki receives commands detailing his next target or if he's done
  // returns NULL if done, or an array of three bytes representing the next coordinates to search

}

// for reference
// byte* readsensor(bool blocking) 
// {  
//   sparki.RGB(RGB_BLUE);
//   sparki.beep();
//   byte* room_num = new byte[4]; // to avoid getting destroyed with the stack frame
//   byte count = 0;
  
//   short code = sparki.readIR();
//   if(code != -1){
//     // sparki.print("Received code: ");
//     flash(0, 0, 100);
//     // sparki.println(code);
//   }

//   switch(code){
    
//   // Arrow buttons
//   case 70: return int('u'); //sparki select up; break;
//   case 21: return int('d'); //sparki select down; break;
//   case 67: return int('r'); //sparki select right; break;
//   case 68: return int('l'); //sparki select left; break;
  
//   // Turn buttons
//   case 71: return int('f'); //sparki choose right side (fire)
//   case 69: return int('p'); //sparki choose left side (person)
  
//   // Stop Button
//   case 64:  sparki.moveStop(); return int('s'); //sparki stop button

//   // Gripper Buttons
//   case 9:  sparki.gripperOpen(); return int('o');
//   case 7:  sparki.gripperClose(); return int('c');
  
//   //keypad for room numbers
//   default:
//   	num_code = find_num(code);
//     if (num_code == 0) 
//     {
//     	while (!flag && count >= 4)
//     	{
    		
//     		count++;
//     	}
//     }
//     else 
//     {
//     	flash(100, 0, 0);
//     }
//   }

//   sparki.RGB(RGB_OFF);
//   return room_num;
// }