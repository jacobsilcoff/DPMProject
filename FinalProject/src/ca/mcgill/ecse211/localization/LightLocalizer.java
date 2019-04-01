package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import ca.mcgill.ecse211.demo.AveragedBuffer;
import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;

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
  private static final double CORRECTION = -17.5;

  private Odometer odo;
  private AveragedBuffer<Float> samples;
  private int x;
  private int y;
  private boolean firstTime;


  /**
   * Creates a light localizer instance with a navigation thread
   * @param x The x coordinate of the robot's block, from bottom left hand corner
   * @param y The y coordinate of the robot's block, from the bottom left hand corner
   * @throws OdometerExceptions if there are problems creating the odometer
   */
  public LightLocalizer(int x, int y) throws OdometerExceptions {
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }

    samples = new AveragedBuffer<Float>(100);
    this.x = x;
    this.y = y;
    firstTime = true;
  }
  /**
   * Creates a light localizer instance with a navigation thread
   * @param x The x coordinate of the robot's block, from bottom left hand corner
   * @param y The y coordinate of the robot's block, from the bottom left hand corner
   * @param firstTime Whether or not this is the first time localizing the robot.
   * This is used to determine whether or not the odometer's x and y are reliable
   * @throws OdometerExceptions if there are problems creating the odometer
   */
  public LightLocalizer(int x, int y, boolean firstTime) throws OdometerExceptions {
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }

    samples = new AveragedBuffer<Float>(100);
    this.x = x;
    this.y = y;
    this.firstTime = firstTime;
  }

  /**
   * Starts the localization.
   */
  public void run() {
    FinalDemo.NAV.turnTo(45);
    moveToLine(true);
    moveBackwards(5);
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
    double tY = Math.abs(tYN - tYP);
    tY = tY > 180 ? 360 - tY : tY; 
    double tX = Math.abs(tXN - tXP);
    tX = tX > 180 ? 360 - tX : tX;
    odo.setX(FinalDemo.GRID_WIDTH * (x+1) 
        - d * Math.cos(Math.toRadians(tY/2)));
    odo.setY(FinalDemo.GRID_WIDTH * (y+1)
        - d * Math.cos(Math.toRadians(tX/2)));
    double odo270 = (tY/2.0 + tS + tYP + 360) % 360; //what the odometer reads when the robot is at 270
    double odo180 = (tX/2.0 + tS + tXN + 360) % 360; //what the odometer reads when the robot is at 180
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
    Sound.beep(); //found a line
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
    Sound.beep(); //found a line
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
    Sound.beep();
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
  private void moveBackwards(double dist) {
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
