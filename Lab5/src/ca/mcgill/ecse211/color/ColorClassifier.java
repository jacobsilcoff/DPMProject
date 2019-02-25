package ca.mcgill.ecse211.color;

import ca.mcgill.ecse211.lab5.Lab5;
import lejos.hardware.lcd.LCD;

/**
 * This is a routine that can be used to identify the color of a soda can located in front of the
 * robot, assuming that the can is centered under the axis of rotation of the light sensor arm
 * 
 * @author jacob
 */
public class ColorClassifier {

  public static final int MAX_TACHO = 210;
  public static final int SLEEP_TIME = 20;
  public static final int SCAN_SPD = 50;
  public static final int MOVE_SPD = 150;
  private static boolean calibrated = false;
  private CanColor colorLabel;


  /**
   * Creates a ColorClassifier
   */
  public ColorClassifier() {
    colorLabel = CanColor.UNKOWN;
  }

  /**
   * Starts by checking if the arm has been calibrated to zero. if it has,
   */
  public CanColor classify() {
    if (!calibrated) {
      calibrate();
    }
    Lab5.SENSOR_MOTOR.setSpeed(SCAN_SPD);
    Lab5.SENSOR_MOTOR.rotateTo(MAX_TACHO, true);

    double[] totalReadings = new double[3];
    int numReadings = 0;

    while (Lab5.SENSOR_MOTOR.isMoving()) {
      float[] sample = new float[Lab5.COLOR_SENSOR.sampleSize()];
      Lab5.COLOR_SENSOR.fetchSample(sample, 0);
      if (!isWhite(sample)) {
        for (int i = 0; i < sample.length; i++) {
          totalReadings[i] += sample[i];
        }
        printReading(toByte(sample));
        numReadings++;
      }
      sleep();
    }

    LCD.drawString("pass 1 done", 0, 0);

    Lab5.SENSOR_MOTOR.rotateTo(0, true);

    while (Lab5.SENSOR_MOTOR.isMoving()) {
      float[] sample = new float[Lab5.COLOR_SENSOR.sampleSize()];
      Lab5.COLOR_SENSOR.fetchSample(sample, 0);
      if (!isWhite(sample)) {
        for (int i = 0; i < sample.length; i++) {
          totalReadings[i] += sample[i];
        }
        printReading(toByte(sample));
        numReadings++;
      }
      sleep();
    }

    float[] avgReading = new float[totalReadings.length];
    for (int i = 0; i < avgReading.length; i++) {
      avgReading[i] = (float) (totalReadings[i] / numReadings);
    }
    for (int i = 0; i < 3; i++) {
      LCD.drawString(avgReading[i] * 1000 + "", 0, 5 + i);
    }
    colorLabel = CanColor.getClosestColor(new byte[] {(byte) (avgReading[0] * 1000),
        (byte) (avgReading[1] * 1000), (byte) (avgReading[2] * 1000)});
    LCD.drawString(colorLabel.toString(), 0, 4);

    return colorLabel;
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
   * Ensures that the tachometer is zeroed properly by moving the arm as far CCW as possible 
   */
  public void calibrate() {
    // moves towards the left
    Lab5.SENSOR_MOTOR.setSpeed(SCAN_SPD);
    Lab5.SENSOR_MOTOR.backward();
    try {
      Thread.sleep(4500);
    } catch (InterruptedException ie) {
      ie.printStackTrace();
    }
    Lab5.SENSOR_MOTOR.stop();
    Lab5.SENSOR_MOTOR.resetTachoCount();
    calibrated = true;
  }

  /** 
   * Allows the arm to be calibrated as a thread.
   * @param wait True to block, false to not block
   */
  public void calibrate(boolean wait) {
    if (wait) {
      calibrate();
    } else {
      (new Thread() {
        public void run() {
          calibrate();
        }
      }).start();
    }

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

    LCD.drawString("rgb(" + c[0] + ", " + c[1] + ", " + c[2] + ")      ", 0, 1);
  }

  private static byte[] toByte(float[] color) {
    byte[] b = new byte[color.length];
    for (int i = 0; i < color.length; i++)
      b[i] = toByte(color[i]);
    return b;
  }

  private static byte toByte(float colorValue) {
    return (byte) (colorValue * 256);
  }

}
