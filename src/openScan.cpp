/*
 * openscan firmware
 * Copyright (C) Christian Völlinger  2019
 *
 * Based on orginal openscan firmware
 * Copyright (C) 2019 Thomas Megel
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <Arduino.h>

#include "config.h"
#include "eeprom-config.h"

#include <LiquidCrystal_I2C.h>
#include <multiCameraIrControl.h>
#include <AccelStepper.h>

#include <menu.h>//menu macros and objects
//#include <menuIO/lcdOut.h>// lcd menu output, doesn't work with our I2 DisplayLib
#include "i2LcdOut.h" // custom implementation of lcdOut.h

using namespace Menu;

/**
 * transmission of the gears, used for rotor movment
 */
#define STEPPER_GEAR 12
#define ROTOR_GEAR 64
float GEAR_RATIO = ROTOR_GEAR / STEPPER_GEAR;

int totalphotos = 0;
int currentRotorPosition = 0;
int currentTablePosition = 0;

enum PgrmState {IDLE, START, MOVE, RUNNING, TAKEPICTURE, FINISHED, ABORT};
PgrmState _state = PgrmState::IDLE;
PgrmStruct active;

AccelStepper StepperTable(AccelStepper::DRIVER, STEPPERTABLE_STEP_PIN, STEPPERTABLE_DIR_PIN);
AccelStepper StepperRotor(AccelStepper::DRIVER, STEPPERROTOR_STEP_PIN, STEPPERROTOR_DIR_PIN);

LiquidCrystal_I2C lcd(0x27, 20, 4);

Camera *pCamera = NULL;

void initializeSteppers()
{
  StepperTable.setMaxSpeed(user_config.tableMaxSpeed); //set max speed the motor will turn (steps/second)   //KREIS
  StepperTable.setAcceleration(user_config.tableMaxAcc); //set acceleration (steps/second^2)
  StepperRotor.setMaxSpeed(user_config.rotorMaxSpeed); //set max speed the motor will turn (steps/second)   // TURNTABLE
  StepperRotor.setAcceleration(user_config.rotorMaxAcc); //set acceleration (steps/second^2)

  StepperTable.setCurrentPosition(0);
  StepperRotor.setCurrentPosition(0);
}


class DirectShutter : public Camera
{

  public:
    DirectShutter(int pin)
    {
      _pin = pin;
    }

    void shutterNow()
    {
      digitalWrite(PHOTO1_PIN, HIGH);
      delay(100);
      digitalWrite(PHOTO1_PIN, LOW);
    }
    private:
      int _pin;
};

void setCameraShutter()
{
  if (pCamera) 
  {
    delete pCamera;
    pCamera = NULL;
  }
  switch (user_config.triggerType)
  {
  case TriggerType::Direct :
    pCamera = new DirectShutter(PHOTO1_PIN);
    break;
  
  case TriggerType::Canon :
    pCamera = new Canon(PHOTO1_PIN);
    break;

  case TriggerType::Minolta :
    pCamera = new Minolta(PHOTO1_PIN);
    break;

  case TriggerType::Nikon :
    pCamera = new Nikon(PHOTO1_PIN);
    break;
  
  case TriggerType::Olympus :
    pCamera = new Olympus(PHOTO1_PIN);
    break;

  case TriggerType::Pentax :
    pCamera = new Pentax(PHOTO1_PIN);
    break;
  
  case TriggerType::Sony :
    pCamera = new Sony(PHOTO1_PIN);
    break;
  
  default:
    break;
  }
}

void takePicture()
{
  pCamera->shutterNow();
  delay(user_config.timePerPhoto);
  totalphotos++;
}

void testShutter()
{
  setCameraShutter();
  pCamera->shutterNow();
}

void updateDisplay()
{
  lcd.clear();
  lcd.print("3D Scanner");
  lcd.setCursor(0,1);
  lcd.print("Photo "); lcd.print(totalphotos); lcd.print(" / "); lcd.print(active.photosPerRotation * active.positions);
  lcd.setCursor(0,3);
  lcd.print("Hold to cancel");
}

void statemachine()
{
  switch (_state)
  {
  case PgrmState::IDLE:
  {
    #ifdef ZUMSCAN
      digitalWrite(STEPPERTABLE_ENABLE_PIN, HIGH);
      digitalWrite(STEPPERROTOR_ENABLE_PIN, HIGH);
    #endif
    break;
  }

  case PgrmState::START:
  {
    #ifdef ZUMSCAN
      digitalWrite(STEPPERTABLE_ENABLE_PIN, LOW);
      digitalWrite(STEPPERROTOR_ENABLE_PIN, LOW);
    #endif
    
    currentRotorPosition = 0;
    currentTablePosition = 0;
    totalphotos = 0;
    active = user_config.program[user_config.lastProg];
    setCameraShutter();

    // move to start angle
    int startPos = user_config.rotorStepsPerRotation / 360.0f * -active.minVert;
    StepperRotor.moveTo(startPos*GEAR_RATIO);
    _state = PgrmState::RUNNING;
    break;
  }

  case PgrmState::MOVE:
  {
    /* 
    calculate next position for rotor and table
    */
    int stepsPerPhoto = user_config.tableStepsPerRotation / active.photosPerRotation;
    float stepsPerVertMove =  ((1.0f*user_config.rotorStepsPerRotation / 360.0f) * (active.minVert * 2)) / active.positions * GEAR_RATIO;

    if (currentRotorPosition < active.positions)
    {
      if (currentTablePosition < active.photosPerRotation-1)
      {
        // move table
        StepperTable.move(stepsPerPhoto);
        currentTablePosition++;
      }
      else
      {
        // move rotor
        currentTablePosition = 0;
        StepperTable.move(stepsPerPhoto);
        StepperRotor.move(stepsPerVertMove);
        currentRotorPosition++;
      }
      _state = PgrmState::RUNNING;
    }
    else
    {
      _state = PgrmState::FINISHED;
    }
    break;
  }

  case PgrmState::RUNNING:
  {
    /* wait until steppers reached destination positions */
    if (StepperRotor.distanceToGo() == 0 && StepperTable.distanceToGo() == 0)
    {
      _state = PgrmState::TAKEPICTURE;
    }

    break;
  }

  case PgrmState::TAKEPICTURE:
  {
    /* code */
    takePicture();
    updateDisplay();
    _state = PgrmState::MOVE;
    break;
  }

  case PgrmState::ABORT:
  case PgrmState::FINISHED:
  {
    /* code */
    StepperRotor.moveTo(0);
    StepperRotor.runToPosition();
    _state = PgrmState::IDLE;
    break;
  }

  default:
    break;
  }
}

void setActivePrgm()
{
  active = user_config.program[user_config.lastProg];
}

void saveEditedPrgm()
{
  user_config.program[user_config.lastProg].minVert = active.minVert;
  user_config.program[user_config.lastProg].photosPerRotation = active.photosPerRotation;
  user_config.program[user_config.lastProg].positions = active.positions;
  saveConfig();
}

void resetConfiguration() 
{
  resetToDefaultConfig();
};

void runProgramCallback() 
{
  _state = PgrmState::START;
};


MENU(subMenuEditProgram,"Edit Program", doNothing,noEvent,noStyle
  ,FIELD(active.photosPerRotation, "Photos/Rot ", "", 1, 100, 10, 1, doNothing, enterEvent | exitEvent| updateEvent, wrapStyle)
  ,FIELD(active.minVert, "Min/Max Vert.", "", 0, 90, 10, 1, doNothing, enterEvent | exitEvent| updateEvent, wrapStyle)
  ,FIELD(active.positions, "Positions ", "", 1, 180, 10, 1, doNothing, enterEvent | exitEvent| updateEvent, wrapStyle)
  ,OP("Save", saveEditedPrgm, enterEvent)
  ,EXIT("<Back")
);

// Setup Menu
MENU(subMenuProgram,"Program", doNothing,noEvent,noStyle
  ,FIELD(user_config.lastProg,  "Program No. ", "", 0, PROGRAM_COUNT-1, 1, 0, setActivePrgm, enterEvent | exitEvent| updateEvent, wrapStyle)
  ,SUBMENU(subMenuEditProgram)
  ,OP("Run program",runProgramCallback,enterEvent)
  ,EXIT("<Back")
);

SELECT(user_config.triggerType,selMenu,"Trigger: ", doNothing,noEvent,wrapStyle
  ,VALUE("Direct", TriggerType::Direct, doNothing,noEvent)
  ,VALUE("Canon",TriggerType::Canon, doNothing,noEvent)
  ,VALUE("Minolta",TriggerType::Minolta,doNothing,noEvent)
  ,VALUE("Nikon",TriggerType::Nikon,doNothing,noEvent)
  ,VALUE("Olympus",TriggerType::Olympus,doNothing,noEvent)
  ,VALUE("Sony",TriggerType::Sony,doNothing,noEvent)
)
  
MENU(subMenuSettings,"Settings", doNothing,noEvent,noStyle
  ,FIELD(user_config.timePerPhoto,  "Time/Photo  ", "ms",0,5000,100,10, doNothing, noEvent, wrapStyle)
  ,SUBMENU(selMenu)
  ,FIELD(user_config.rotorStepsPerRotation,  "Rotor  ", "#/1",0,8000,100,1, doNothing, noEvent, wrapStyle)
  ,FIELD(user_config.tableStepsPerRotation,  "Table ", "#/1",0,8000,100,1, doNothing, noEvent, wrapStyle)
  ,FIELD(user_config.rotorMaxSpeed,    "Rotor Spd ", "", 0, 5000, 100, 10, initializeSteppers, enterEvent | exitEvent| updateEvent, wrapStyle)
  ,FIELD(user_config.rotorMaxAcc,     "Rotor Acc ", "", 0, 5000, 100, 10, initializeSteppers, enterEvent | exitEvent| updateEvent, wrapStyle)
  ,FIELD(user_config.tableMaxSpeed,   "Table Spd ", "", 0, 5000, 100, 10, initializeSteppers, enterEvent | exitEvent| updateEvent, wrapStyle)
  ,FIELD(user_config.tableMaxAcc,    "Table Acc ", "", 0, 5000, 100, 10, initializeSteppers, enterEvent | exitEvent| updateEvent, wrapStyle)
  ,OP("Test Shutter",testShutter, enterEvent | exitEvent)
  ,OP("Save", saveConfig, enterEvent)
  ,OP("Reset", resetConfiguration, enterEvent | exitEvent)
  ,EXIT("<Back")
)

//Main Menu
MENU(mainMenu,"3D Scanner",Menu::doNothing,noEvent,noStyle
  ,OP("Run last program",runProgramCallback, enterEvent)
  ,SUBMENU(subMenuProgram)
  ,SUBMENU(subMenuSettings)
);

#ifdef ZUMSCAN
ClickEncoder clickEncoder(BUTTON_UP_PIN, BUTTON_DOWN_PIN, BUTTON_OK_PIN, 4);
ClickEncoderStream in(clickEncoder,1);
void timerIsr() {clickEncoder.service();}
#endif

#ifdef OPENSCAN
keyMap joystickBtn_map[]={
 {-BUTTON_OK_PIN, defaultNavCodes[enterCmd].ch} ,
 {-BUTTON_UP_PIN, defaultNavCodes[upCmd].ch} ,
 {-BUTTON_DOWN_PIN, defaultNavCodes[downCmd].ch}  ,
};
keyIn<3> in(joystickBtn_map);
#endif


#define MAX_DEPTH 3

MENU_OUTPUTS(out,MAX_DEPTH
  ,LCD_OUT(lcd,{0,0,20,4})
  ,NONE
);
NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);//the navigation root object

/**
 * Initializes the program
 */
void setup() {
  Serial.begin(9600);
  Serial.println("3D Scanner");

  loadConfig();
  active = user_config.program[user_config.lastProg];
  setCameraShutter();
  initializeSteppers();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.home();

#ifdef ZUMSCAN
  pinMode(STEPPERTABLE_ENABLE_PIN, OUTPUT);
  pinMode(STEPPERROTOR_ENABLE_PIN, OUTPUT);
    // start ISR for Encoder
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
#endif
#ifdef OPENSCAN
  in.begin();
#endif

  pinMode(PHOTO1_PIN, OUTPUT);
  pinMode(BUTTON_OK_PIN, INPUT_PULLUP);

  _state = PgrmState::IDLE;
}

/**
 * Main program loop  
 */
void loop() 
{
  if (_state == IDLE) 
  {
    nav.poll();
  }
  #ifdef ZUMSCAN
  else if (clickEncoder.getButton() == ClickEncoder::Held)
  #endif
  #ifdef OPENSCAN
  else if (in.lastkey == navCmds::escCmd)
  #endif
  {
    _state = PgrmState::ABORT;
  }
  

  statemachine();
  StepperRotor.run();
  StepperTable.run();

}
