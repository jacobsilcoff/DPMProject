package ca.mcgill.ecse211.canhandling;

import ca.mcgill.ecse211.demo.BetaDemo;
import lejos.hardware.lcd.LCD;

/**
 * This is a routine that can be used to identify the color of a soda can
 * located in front of the robot, assuming that the can is centered under
 * the axis of rotation of the light sensor arm
 * 
 * @author group 6
 */
public class ColorClassifier {

  public static final int MAX_TACHO = 210;
  public static final int SLEEP_TIME = 20;
  public static final int SCAN_SPD = 180;
  public static final int MOVE_SPD = 150;
  private CanColor colorLabel;


  /**
   * Creates a ColorClassifier
   */
  public ColorClassifier() {
    colorLabel = CanColor.UNKNOWN;
  }
  
  /**
   * Returns true if a can is seen, else false
   * @return true if a can is seen, else false
   */
  public boolean canDetected() {
    double[] totalReadings = new double[3];
    int numReadings = 0;
    for (int i = 0; i < 10; i++) {
      float[] sample = new float[BetaDemo.COLOR_SENSOR.sampleSize()];
      BetaDemo.COLOR_SENSOR.fetchSample(sample, 0);
      if (!isWhite(sample)) {
        for (int j = 0; j < sample.length; j++) {
          totalReadings[j] += sample[j];
        }
        numReadings++;
      }
      sleep();
    }
    float[] avgReading = new float[totalReadings.length];
    for (int i = 0; i < avgReading.length; i++) {
      avgReading[i] = (float) (totalReadings[i] / numReadings);
    }
    for (int i = 0; i < 3; i++) {
      if (Float.isNaN(avgReading[i])) {
        //LCD.clear();
        //LCD.drawString("NO CAN", 0, 0);
        return false;
      }
    }
    return true; 
  }

  /**
   * Uses the light sensor to calculate the 
   * color of the can.
   * @return The color of the can. CanColor.UNKONWN if no can is detected
   */
  public CanColor classify() {
    BetaDemo.CAN_MOTOR.resetTachoCount();
    BetaDemo.CAN_MOTOR.setSpeed(SCAN_SPD);
    BetaDemo.CAN_MOTOR.backward();

    double[] totalReadings = new double[3];
    int numReadings = 0;

    while (Math.abs(BetaDemo.CAN_MOTOR.getTachoCount()) < 360*3) {
      float[] sample = new float[BetaDemo.COLOR_SENSOR.sampleSize()];
      BetaDemo.COLOR_SENSOR.fetchSample(sample, 0);
      if (!isWhite(sample)) {
        for (int i = 0; i < sample.length; i++) {
          totalReadings[i] += sample[i];
        }
        numReadings++;
      }
      sleep();
    }
    BetaDemo.CAN_MOTOR.setSpeed(0);
    BetaDemo.CAN_MOTOR.stop();

    float[] avgReading = new float[totalReadings.length];
    for (int i = 0; i < avgReading.length; i++) {
      avgReading[i] = (float) (totalReadings[i] / numReadings);
    }
    for (int i = 0; i < 3; i++) {
      if (Float.isNaN(avgReading[i])) {
        LCD.clear();
        LCD.drawString("NO CAN", 0, 0);
        return CanColor.UNKNOWN;
      }
    }
    for (int i = 0; i < 3; i++) {
      LCD.drawString(avgReading[i] * 1000 + "", 0, 5 + i);
    }
    colorLabel = CanColor.getClosestColor(new int[] {(int) (avgReading[0] * 1000),
        (int) (avgReading[1] * 1000), (int) (avgReading[2] * 1000)});
    LCD.drawString(colorLabel.toString(), 0, 4);

    BetaDemo.CAN_MOTOR.flt();
    
    return colorLabel;
  }
  
  /**
   * Prints out information of the can
   * used for testing. This method can be modified
   * to find the mean, std dev, and other stats
   * about a can
   */
  public void getData() {
    BetaDemo.CAN_MOTOR.resetTachoCount();
    BetaDemo.CAN_MOTOR.setSpeed(SCAN_SPD);
    BetaDemo.CAN_MOTOR.backward();

    double[] totalReadings = new double[3];
    int numReadings = 0;

    while (Math.abs(BetaDemo.CAN_MOTOR.getTachoCount()) < 360*3) {
      float[] sample = new float[BetaDemo.COLOR_SENSOR.sampleSize()];
      BetaDemo.COLOR_SENSOR.fetchSample(sample, 0);
      if (!isWhite(sample)) {
        for (int i = 0; i < sample.length; i++) {
          totalReadings[i] += sample[i];
        }
        numReadings++;
      }
      sleep();
    }

    BetaDemo.CAN_MOTOR.setSpeed(0);
    BetaDemo.CAN_MOTOR.stop();
    
    float[] avgReading = new float[totalReadings.length];
    for (int i = 0; i < avgReading.length; i++) {
      avgReading[i] = (float) (totalReadings[i] / numReadings);
    }
    for (int i = 0; i < 3; i++) {
      if (Float.isNaN(avgReading[i])) {
        LCD.clear();
        LCD.drawString("NO CAN", 0, 1);
        return;
      } else {
        LCD.drawString((new String[]{"r: ", "g: ", "b: "})[i] + 
            (int)(avgReading[i] * 1000), 0, 1+i);
      }
    }
    colorLabel = CanColor.getClosestColor(new int[] {(int)(avgReading[0] * 1000),
        (int) (avgReading[1] * 1000), (int) (avgReading[2] * 1000)});
    LCD.drawString(colorLabel.toString(), 0, 0);
  }


  /**
   * Returns the identified color of the can
   * 
   * @return the color of the can being identified
   */
  public CanColor getColor() {
    return colorLabel;
  }

  /**
   * Tells whether or not a certain color is white or not
   * 
   * @param color an arry of 3 floats, r, g, and b each 0-1
   * @return true for white, false for non-white (ie, a color)
   */
  private static boolean isWhite(float[] color) {
    float min = color[0];
    float max = color[0];
    for (float f : color) {
      min = min > f ? f : min;
      max = max < f ? f : max;
    }
    return (max - min < .015);
  }

  /**
   * Sleeps the thread for one tick
   */
  private void sleep() {
    try {
      Thread.sleep(SLEEP_TIME);
    } catch (InterruptedException ie) {
      ie.printStackTrace();
    }
  }

  private static void printReading(byte[] c) {
    LCD.drawString("rgb(" + c[0] + ", " + c[1] + ", " + c[2] + ")      ", 0, 6);
  }

  private static byte[] toByte(float[] color) {
    byte[] b = new byte[color.length];
    for (int i = 0; i < color.length; i++)
      b[i] = toByte(color[i]);
    return b;
  }

  private static byte toByte(float colorValue) {
    return (byte) (colorValue * 1000);
  }

}
