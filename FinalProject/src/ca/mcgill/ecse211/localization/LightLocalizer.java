package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import ca.mcgill.ecse211.demo.AveragedBuffer;
import ca.mcgill.ecse211.demo.Demo;
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
  private static final int MOTOR_SPEED = 130;

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

  private Odometer odo;
  private Navigation nav;
  private AveragedBuffer<Float> samples;
  private int x;
  private int y;
  private boolean firstTime;


  /**
   * Creates a light localizer instance with a navigation thread
   * @param oc The odometry correction that is turned on/off as the localizer operates
   * @param x The x coordinate of the robot's block, from bottom left hand corner
   * @param y The y coordinate of the robot's block, from the bottom left hand corner
   * @throws OdometerExceptions if there are problems creating the odometer
   */
  public LightLocalizer(OdometryCorrection oc, int x, int y) throws OdometerExceptions {
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    this.nav = new Navigation(null); // start a new nav thread
    nav.start();
    samples = new AveragedBuffer<Float>(100);
    this.x = x;
    this.y = y;
    firstTime = true;
  }
  /**
   * Creates a light localizer instance with a navigation thread
   * @param oc The odometry correction that is turned on/off as the localizer operates
   * @param x The x coordinate of the robot's block, from bottom left hand corner
   * @param y The y coordinate of the robot's block, from the bottom left hand corner
   * @param firstTime Whether or not this is the first time localizing the robot.
   * This is used to determine whether or not the odometer's x and y are reliable
   * @throws OdometerExceptions if there are problems creating the odometer
   */
  public LightLocalizer(OdometryCorrection oc, int x, int y, boolean firstTime) throws OdometerExceptions {
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    this.nav = new Navigation(null); // start a new nav thread
    nav.start();
    samples = new AveragedBuffer<Float>(100);
    this.x = x;
    this.y = y;
    this.firstTime = firstTime;
  }

  /**
   * Starts the localization.
   */
  public void run() {
    if (firstTime) {
      /*
       * Roughly sets up x and y in the case
       * where the odometer's position values
       * are unreliable
       */
      nav.turnTo(0);
      moveToLine(true);
      odo.setY(OdometryCorrection.LINE_SPACING * (y+1) + Demo.LINE_OFFSET_Y);
      moveBackwards(Demo.LINE_OFFSET_Y + 10);
      nav.turnTo(90);
      moveToLine(true);
      odo.setX(OdometryCorrection.LINE_SPACING * (x+1) + Demo.LINE_OFFSET_Y);
    }

    /*
     * Moves to safe rotation position,
     * about which all 4 lines will be intersected
     */
    nav.travelTo(OdometryCorrection.LINE_SPACING * (x+1) - 6, 
        OdometryCorrection.LINE_SPACING * (y+1) - 6);
    while (nav.isNavigating()) sleep();
    /*
     * Turns to 60 deg to ensure the light sensor is in the
     * starting block, to more easily know the order in which
     * lines will be crossed
     */
    nav.turnTo(60); 

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
    double d = Math.sqrt(Math.pow(Demo.LINE_OFFSET_X,2) + Math.pow(Demo.LINE_OFFSET_Y, 2));
    double tY = Math.abs(tYN - tYP);
    tY = tY > 180 ? 360 - tY : tY; 
    double tX = Math.abs(tXN - tXP);
    tX = tX > 180 ? 360 - tX : tX;
    odo.setX(OdometryCorrection.LINE_SPACING * (x+1) 
        - d * Math.cos(Math.toRadians(tY/2)));
    odo.setY(OdometryCorrection.LINE_SPACING * (y+1)
        - d * Math.cos(Math.toRadians(tX/2)));
    double sensorTheta = Math.toDegrees(Math.acos(Demo.LINE_OFFSET_X / Demo.LINE_OFFSET_Y));
    double odo270 = (tY/2 + tYP - sensorTheta + 360) % 360; //what the odometer reads when the robot is at 270
    double odo180 = (tX/2 + tXP - sensorTheta + 360) % 360; //what the odometer reads when the robot is at 180
    double avgError = ((odo180 - 180) + (odo270 - 270)) / 2;
    odo.setTheta(odo.getXYT()[2] - avgError + 38);
    nav.end();
  }

  /**
   * Moves the robot forward or backward until a line is detected
   * 
   * @param forwards True to move forward, false for backward
   */
  public void moveToLine(boolean forwards) {
    int dir = forwards ? 1 : -1;
    nav.setSpeeds(dir * MOTOR_SPEED, dir * MOTOR_SPEED);
    waitUntilLine();
    Sound.beep(); //found a line
    nav.setSpeeds(0, 0);
  }

  /**
   * Rotates the robot clockwise(cw) or counterclockwise (ccw)
   * until a line is detected
   * 
   * @param cw True to move cw, false for ccw
   */
  public void rotateToLine(boolean cw) {
    int dir = cw ? 1 : -1;
    nav.setSpeeds(dir * MOTOR_SPEED * 0.5f, -dir * MOTOR_SPEED * 0.5f);
    waitUntilLine();
    Sound.beep(); //found a line
    nav.setSpeeds(0, 0);
  }

  /**
   * Blocks until a line is detetected by the robot
   */
  private void waitUntilLine() {
    float[] sample = new float[Demo.LINE_SENSOR.sampleSize()];
    do {
      Demo.LINE_SENSOR.fetchSample(sample, 0);
      samples.add(sample[0]);
      Demo.LCD.clear();
      Demo.LCD.drawString(sample[0] + ", " + samples.getAvg() + "      ",0,4);
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
    nav.setSpeeds(MOTOR_SPEED,MOTOR_SPEED);
    double[] start = nav.getOdo().getXYT();

    Demo.LEFT_MOTOR.backward();
    Demo.RIGHT_MOTOR.backward();

    while (Navigation.dist(nav.getOdo().getXYT(), start) < Math.abs(dist)) {
      try {
        Thread.sleep(30);
      } catch (InterruptedException e) {
      }
    }
    nav.setSpeeds(0, 0);
  }

}
