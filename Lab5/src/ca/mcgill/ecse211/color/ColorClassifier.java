package ca.mcgill.ecse211.color;

import ca.mcgill.ecse211.lab5.Lab5;
import lejos.hardware.lcd.LCD;

public class ColorClassifier extends Thread{
  
  public static final int SLEEP_TIME = 20;
  public static final int DETECTION_THRESH = 15;
  private CanColor colorLabel;
  private AveragedColor avgCol;
  
  //Variables used for calculating averages
  static int samples = 0;
  static long red = 0;
  static long grn = 0;
  static long blu = 0;
  
  public ColorClassifier() {
    colorLabel = CanColor.UNKOWN;
    avgCol = new AveragedColor();
  }
  
  public void run() {
    while (true) {
      
      //gets a color reading
      float[] sample = new float[Lab5.COLOR_SENSOR.sampleSize()];
      Lab5.COLOR_SENSOR.fetchSample(sample, 0);
      
      byte[] c = toByte(sample);
      printReading(c);
      
      avgCol.add(sample);
      
      colorLabel = classifyColor(toByte(avgCol.getAvg()));
      LCD.drawString(colorLabel.toString() + "           ", 0, 2);
      String avg = avgCol.toString();
      LCD.drawString("avg: " + avg.substring(0,avg.indexOf(',')) + "           ", 0, 3);
      LCD.drawString("   " + avg.substring(avg.indexOf(',')) + "           ", 0, 4);
      
      
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
  
  public static byte[] toByte(double[] color) {
    byte[] b = new byte[color.length];
    for (int i = 0; i < color.length; i++) 
      b[i] = toByte(color[i]);
    return b;
  }
  
  public static byte toByte(float colorValue) {
    return (byte)(colorValue * 256);
  }
  
  public static byte toByte(double colorValue) {
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
  
  /**
   * Represents a color that is averaged
   * over a number of samples.
   * Used to implement a rolling average filter.
   * @author jacob
   *
   */
  private class AveragedColor {
    /**
     * This value stores the default number of samples that are stored in the buffer
     */
    private static final int DEFAULT_N = 25;
    private int n;
    private int numEls;
    private double[][] samples;
    private int sampleIndex;
    private double[] avg;
    
    /**
     * Creates a buffer that will store a default number of samples, as specified by DEFAULT_N
     */
    public AveragedColor() {
      n = DEFAULT_N;
      clear();
    }
    
    /**
     * Creates a buffer that will store a default number of samples, as specified by DEFAULT_N
     */
    public AveragedColor(int n) {
      this.n = n;
      clear();
    }
    
    /**
     * Adds a measurement to the buffer and updates the average
     * 
     * @param x The data sample to add to the buffer
     */
    public void add(double[] x) {
      if (numEls < n) {
        numEls++;
        for (int i = 0; i < 3; i++) {
          avg[i] = (avg[i] * (numEls - 1.0) + x[i]) / numEls;
        }
      } else {
        for (int i = 0; i < 3; i++) {
          avg[i] = avg[i] + 1.0 / n * (x[i] - samples[sampleIndex][i]);
        }
      }
      samples[sampleIndex] = x;
      sampleIndex = (sampleIndex + 1) % n;
    }
    
    public void add(float[] x) {
      double[] temp = new double[3];
      for (int i = 0; i < 3; i++) {
        temp[i] = x[i];
      }
      add(temp);
    }

    /**
     * Returns the average of the buffer
     * 
     * @return The average of the buffer
     */
    public double[] getAvg() {
      return avg;
    }
    
    /**
     * Clears the buffer
     */
    public void clear() {
      samples = new double[n][3];
      avg = new double[3];
      sampleIndex = 0;
      numEls = 0;
    }
    
    public String toString() {
      return "rgb(" + 
          ((int)(avg[0]*10))/10.0 + ", " + 
          ((int)(avg[1]*10))/10.0 + ", " + 
          ((int)(avg[2]*10))/10.0 + ")";
    }
    
  }
  
}
