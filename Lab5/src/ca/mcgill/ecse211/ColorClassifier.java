package ca.mcgill.ecse211;

import lejos.hardware.lcd.LCD;

public class ColorClassifier extends Thread{
  
  public static final int SLEEP_TIME = 20;
  public static final int DETECTION_THRESH = 10;
  private CanColor color;
  
  
  public ColorClassifier() {
    color = CanColor.UNKOWN;
  }
  
  public void run() {
    while (true) {
      
      //gets a color reading
      float[] sample = new float[Lab5.LIGHT_SENSOR.sampleSize()];
      Lab5.LIGHT_SENSOR.fetchSample(sample, 0);
      
      printReading(sample);
      color = classifyColor(sample);
      LCD.drawString(color.toString() + "           ", 0, 2);
      
      try {
        sleep(SLEEP_TIME);
      } catch (InterruptedException ie) {
        ie.printStackTrace();
      }
    }
  }
  
  public static void printReading(float[] c) {
    
    LCD.drawString("rgb(" + toByte(c[0]) + ", " + toByte(c[1]) + ", " + toByte(c[2]) + ")      ", 0, 1);
  }
  
  public static byte toByte(float colorValue) {
    return (byte)(colorValue * 256);
  }
  
  public static CanColor classifyColor(float[] c) {
    float r = c[0], g = c[1], b = c[2];
    if (Math.min(Math.min(r, g), b) < DETECTION_THRESH) {
      return CanColor.UNKOWN;
    }
    return CanColor.RED;
  }
  
}
