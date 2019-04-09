package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import ca.mcgill.ecse211.demo.AveragedBuffer;
import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

/**
 * This class represents a routine used by the robot
 * to find the x and y axis of the grid system it is on.
 * 
 * It uses a light sensor to detect these lines, assuming an 
 * accurate theta value stored in the odometer. 
 * @author jacob
 *
 */
public class LightLocalizer {

  /**
   * The speed of the motors during localization
   */
  private static final int MOTOR_SPEED = 190;

  /**
   * The time between polling the sensor, in ms
   */
  public static final int POLL_DELAY = 7;

  /**
   * The time (ms) waited before checking that the navigation is done
   */
  public static final int SLEEP_TIME = 50;
  /**
   * This represents the minimum difference from the mean for a light sensor reading to be
   * considered significant
   */
  private static final float LIGHT_THRESHOLD = 0.05f;
  
  /**
   * Making this smaller leads to CW rotation
   */
  private static final double CORRECTION = -14;//smaller for cw

  private Odometer odo;
  private AveragedBuffer<Float> samples;
  private double x;
  private double y;
  private boolean midTravel;


  /**
   * Creates a light localizer instance with a navigation thread
   * @param x The x coordinate of the cross around which localization occurs
   * @param y The y coordinate of the cross around which localization occurs
   * @throws OdometerExceptions if there are problems creating the odometer
   */
  public LightLocalizer(double x, double y) throws OdometerExceptions {
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }

    samples = new AveragedBuffer<Float>(100);
    this.x = x;
    this.y = y;
    midTravel = true;
  }
  /**
   * Creates a light localizer instance with a navigation thread
   * @param x The x coordinate of the cross around which localization occurs
   * @param y The y coordinate of the cross around which localization occurs
   * @param midTravel Whether or not this is the first time localizing the robot.
   * This is used to determine whether or not the odometer's x and y are reliable
   * @throws OdometerExceptions if there are problems creating the odometer
   */
  public LightLocalizer(double x, double y, boolean midTravel) throws OdometerExceptions {
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }

    samples = new AveragedBuffer<Float>(100);
    this.x = x;
    this.y = y;
    this.midTravel = midTravel;
  }

  /**
   * Starts the localization.
   */
  public void run() {
    if (!midTravel) {
      FinalDemo.NAV.turnTo(35);
      moveToLine(true);
      moveBackwards(8);
    }
    FinalDemo.NAV.turnTo(35);
    //Find the 4 intersections
    rotateToLine(false);
    double tYN = odo.getXYT()[2];
    rotateToLine(false);
    double tXP = odo.getXYT()[2];
    rotateToLine(false);
    double tYP = odo.getXYT()[2];
    rotateToLine(false);
    double tXN = odo.getXYT()[2];

    //calculates & updates values
    double d = Math.sqrt(Math.pow(FinalDemo.LINE_OFFSET_X,2) + Math.pow(FinalDemo.LINE_OFFSET_Y, 2));
    double tS = Math.toDegrees(Math.atan(FinalDemo.LINE_OFFSET_X / FinalDemo.LINE_OFFSET_Y));
    double tY = (tYN > tYP) ? (tYN - tYP) : (tYN + 360 - tYP);
    double tX = (tXP > tXN) ? (tXP - tXN) : (tXP + 360 - tXN);
    odo.setX(x - d * Math.cos(Math.toRadians(tY/2)));
    odo.setY(y - d * Math.cos(Math.toRadians(tX/2)));
    //FROM THE Y POINTS:
    double odo180 = (tXP - tX/2.0 + tS + 360) % 360;
    double odo270 =  (tYN - tY/2.0 + tS + 360) % 360;
    //FROM THE X POINTS:
    double err180 = (180 - odo180 + 360) % 360;
    double err270 = (270 - odo270 + 360) % 360;
    double avgError = (err180 + err270) / 2;
    odo.setTheta(odo.getXYT()[2] + avgError + CORRECTION);
  }

  /**
   * Moves the robot forward or backward until a line is detected
   * 
   * @param forwards True to move forward, false for backward
   */
  public void moveToLine(boolean forwards) {
    int dir = forwards ? 1 : -1;
    FinalDemo.NAV.setSpeeds(dir * MOTOR_SPEED, dir * MOTOR_SPEED);
    waitUntilLine();
    if (FinalDemo.DEBUG_ON) {
      Sound.beep(); //found a line
    }
    FinalDemo.NAV.setSpeeds(0, 0);
  }

  /**
   * Rotates the robot clockwise(cw) or counterclockwise (ccw)
   * until a line is detected
   * 
   * @param cw True to move cw, false for ccw
   */
  public void rotateToLine(boolean cw) {
    int dir = cw ? 1 : -1;
    FinalDemo.NAV.setSpeeds(dir * MOTOR_SPEED * 0.5f, -dir * MOTOR_SPEED * 0.5f);
    waitUntilLine();
    if (FinalDemo.DEBUG_ON) {
      Sound.beep(); //found a line
    }
    FinalDemo.NAV.setSpeeds(0, 0);
  }

  /**
   * Blocks until a line is detetected by the robot
   */
  private void waitUntilLine() {
    float[] sample = new float[FinalDemo.LINE_SENSOR.sampleSize()];
    do {
      FinalDemo.LINE_SENSOR.fetchSample(sample, 0);
      samples.add(sample[0]);
      FinalDemo.LCD.clear();
      FinalDemo.LCD.drawString(sample[0] + ", " + samples.getAvg() + "      ",0,4);
      sleep();
    } while (sample[0] > samples.getAvg() - LIGHT_THRESHOLD);
    samples.clear();
    if (FinalDemo.DEBUG_ON) {
      Sound.beep();
    }
  }

  /**
   * Sleeps for the default amount of time
   */
  private void sleep() {
    try {
      Thread.sleep(POLL_DELAY);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * Moves the robot backwards (straight) a certain distance, using the odometer.
   * 
   * @param dist
   */
  public static void moveBackwards(double dist) {
    FinalDemo.NAV.setSpeeds(MOTOR_SPEED,MOTOR_SPEED);
    double[] start = FinalDemo.NAV.getOdo().getXYT();

    FinalDemo.LEFT_MOTOR.backward();
    FinalDemo.RIGHT_MOTOR.backward();

    while (Navigation.dist(FinalDemo.NAV.getOdo().getXYT(), start) < Math.abs(dist)) {
      try {
        Thread.sleep(30);
      } catch (InterruptedException e) {
      }
    }
    FinalDemo.NAV.setSpeeds(0, 0);
  }

}
