package ca.mcgill.ecse211;

import lejos.hardware.lcd.LCD;

public class ColorClassifier extends Thread{
  
  public static final int SLEEP_TIME = 20;
  public static final int DETECTION_THRESH = 15;
  private CanColor color;
  static int samples = 0;
  static long red = 0;
  static long grn = 0;
  static long blu = 0;
  
  public ColorClassifier() {
    color = CanColor.UNKOWN;
  }
  
  public void run() {
    while (true) {
      
      //gets a color reading
      float[] sample = new float[Lab5.LIGHT_SENSOR.sampleSize()];
      Lab5.LIGHT_SENSOR.fetchSample(sample, 0);
      byte[] c = toByte(sample);
      
      printReading(c);
      color = classifyColor(c);
      LCD.drawString(color.toString() + "           ", 0, 2);
      if (samples > 500) {
        LCD.drawString("avg(" + (red/samples) + "," + (grn/samples) + "," + (blu/samples) + ")           ", 0, 3);
      }
      
      try {
        sleep(SLEEP_TIME);
      } catch (InterruptedException ie) {
        ie.printStackTrace();
      }
    }
  }
  
  public static void printReading(byte[] c) {
    
    LCD.drawString("rgb(" + c[0] + ", " + c[1] + ", " + c[2] + ")      ", 0, 1);
  }
  
  public static byte[] toByte(float[] color) {
    byte[] b = new byte[color.length];
    for (int i = 0; i < color.length; i++) 
      b[i] = toByte(color[i]);
    return b;
  }
  
  public static byte toByte(float colorValue) {
    return (byte)(colorValue * 256);
  }
  
  public static CanColor classifyColor(byte[] c) {
    
    byte r = c[0], g = c[1], b = c[2];
    red += r;
    grn += g;
    blu += b;
    samples ++;
    if (Math.max(Math.max(r, g), b) < DETECTION_THRESH) {
      red = grn = blu = samples = 0;
      return CanColor.UNKOWN;
    }
    return CanColor.getClosestColor(c);
  }
  
}
