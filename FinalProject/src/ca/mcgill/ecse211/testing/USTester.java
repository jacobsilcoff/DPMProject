package ca.mcgill.ecse211.testing;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class USTester {

  public static final SampleProvider DIST;
  static {
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
    DIST = usSensor.getMode("Distance");
  }

  public static void main(String[] args) throws InterruptedException {
    Sound.twoBeeps();
    (new Thread() {
      public void run() {
        while (true) {
          float dist = readUS();
          if (dist < 40) {
            Sound.beepSequenceUp();
            try {
              sleep(1000);
            } catch (InterruptedException e) {
              // TODO Auto-generated catch block
              e.printStackTrace();
            }
          }
          LCD.drawString("Dist: " + dist + "       ", 0, 0);
          try {
            sleep(50);
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          }
        }
      }
    }).start();
  
  while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
    Thread.sleep(100);
  }
  System.exit(0);
}

/**
 * Polls the ultrasonic sensor and returns the result
 * 
 * @return The US reading in cm
 */
private static float readUS() {
  float[] usData = new float[DIST.sampleSize()];
  DIST.fetchSample(usData, 0);
  if (usData[0] == 255) {
    return -1;
  }
  return usData[0] * 100f;
}
}
