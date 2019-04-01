package ca.mcgill.ecse211.testing;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.MirrorMotor;
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
  
  //This was a failure!
  public static void buttonTest() {
    (new Thread() {
      public void run() {
        float[] sample = new float[TOUCH_SENSOR.sampleSize()];
        while (true) {
          TOUCH_SENSOR.fetchSample(sample, 0);
          if (sample[0] == 1){
            Sound.beep();
          }
          try {
            sleep(15);
          } catch(InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }).start();
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
