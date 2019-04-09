package ca.mcgill.ecse211.testing;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

public class GyroTester {
  /**
   * The port for the gyroscope
   */
  public static final EV3GyroSensor GYRO =  new EV3GyroSensor(LocalEV3.get().getPort("S4"));
  /**
   * The robots gyroscope sample provider
   */
  public static final SampleProvider GYRO_DATA = GYRO.getAngleMode();

  public static void main(String[] args) throws InterruptedException {
    class TestThread extends Thread {
      public void run() {
        GYRO.reset();
        while (true) {
          float val = readGyro();
          LCD.clear();
          LCD.drawString("Θ: " + val, 0,0);
          try {
            sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
      
      public void otherMethod() throws InterruptedException{
        Sound.beep();
        sleep(10000);
      }
    }
    
    TestThread t = new TestThread();
    t.start();
    
    while (true) {
      int b = Button.waitForAnyPress();
      if (b == Button.ID_ESCAPE) {
        break;
      } else {
        GYRO.reset();
      }
    }
    System.exit(0);
  }
  
  
  /**
   * Gets the value of the gyroscope
   * @return
   */
  private static float readGyro() {
    float[] sample = new float[GYRO_DATA.sampleSize()];
    GYRO_DATA.fetchSample(sample, 0);
    return sample[0];
  }
  
}
