/*******************************************
  IR Remote
********************************************/
#include <Sparki.h> // include the sparki library
#include "final_project.h"

void setup() 
{ 

}

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

void flash(int r, int g, int b)
{
  for (int i = 0; i<1000; i++)
  {
  	if (i % 2 == 0)
  		{sparki.RGB(r, g, b);}
  	else
  		{sparki.RGB(r, g, b);}
  }
}

int find_num(int code)
{
  for (int i = 0; i<10; i++)
  {
  	if (numpad[i] == code)
  		{return i}
  }
  return (-1);
}

int * readsensor() 
{  
    sparki.RGB(RGB_BLUE);
  sparki.beep();
  int numpad[10] = [25, 12, 24, 94, 8, 28, 90, 66, 82, 74];
  int room_num[4] = [0, 0, 0, 0];
  bool flag = false;
  int count = 0;
  
  int code = sparki.readIR();
  if(code != -1){
    sparki.print("Received code: ");
    flash(0, 0, 100);
    sparki.println(code);
  }

  switch(code){
    
  // Arrow buttons
  case 70: return int('u'); //sparki select up; break;
  case 21: return int('d'); //sparki select down; break;
  case 67: return int('r'); //sparki select right; break;
  case 68: return int('l'); //sparki select left; break;
  
  // Turn buttons
  case 71: return int('f'); //sparki choose right side (fire)
  case 69: return int('p'); //sparki choose left side (person)
  
  // Stop Button
  case 64:  sparki.moveStop(); return int('s'); //sparki stop button

  // Gripper Buttons
  case 9:  sparki.gripperOpen(); return int('o');
  case 7:  sparki.gripperClose(); return int('c');
  
  //keypad for room numbers
  default:
  	num_code = find_num(code);
    if (num_code == 0) 
    {
    	while (!flag && count >= 4)
    	{
    		
    		count++;
    	}
    }
    else 
    {
    	flash(100, 0, 0);
    }
  }

  sparki.updateLCD();
}

