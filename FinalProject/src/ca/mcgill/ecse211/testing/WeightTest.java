package ca.mcgill.ecse211.testing;

import ca.mcgill.ecse211.demo.FinalDemo;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class WeightTest {
  
  /**
   * The robot's left motor
   */
  public static final RegulatedMotor CLAW_MOTOR = 
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  /**
   * The robot's touch sensor to detect heavy cans
   */
  public static final SampleProvider TOUCH_SENSOR;
  static {
    @SuppressWarnings("resource")
    SensorModes touchSensorMode = new EV3TouchSensor(LocalEV3.get().getPort("S4"));
    TOUCH_SENSOR = touchSensorMode.getMode("Touch");
  }
  
  public static void main(String[] args) {
    clawTest();
  }
  

  /**
   * Tests the weight function
   * Note: init must be called in main class first!
   */
  public static void weightTestFinal() {
    while (true) {
      FinalDemo.CLAW.open();
      Button.waitForAnyPress();
      FinalDemo.CLAW.close();
      if (FinalDemo.CLAW.isHeavy()) {
        Sound.twoBeeps();
      } else {
        Sound.beepSequenceUp();
      }
      if (Button.waitForAnyPress() == Button.ID_ESCAPE) break;
    }
  }
  
  public static void clawTest() {
    CLAW_MOTOR.close();
    UnregulatedMotor u = new UnregulatedMotor(LocalEV3.get().getPort("D"));
    int i = 1;
    while (true) {
      LCD.drawString("Pow: " + i, 0, 0);
      u.setPower(i);
      int b = Button.waitForAnyPress();
      if (b == Button.ID_ESCAPE) {
        break;
      } else if (b == Button.ID_DOWN) {
        i --;
      } else if (b == Button.ID_UP) {
        i ++ ;
      } else {
        i = 0;
      }
    }
  }
}
