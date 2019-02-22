package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import ca.mcgill.ecse211.lab5.AveragedBuffer;
import ca.mcgill.ecse211.lab5.Lab5;
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
public class LightLocalizer extends Thread {

  /**
   * The speed of the motors during localization
   */
  private static final int MOTOR_SPEED = 100;
  /**
   * This is the offset distance (cm) of our US sensor and the wheelbase axis
   */
  private static final double SENSOR_OFFSET_X = 1;
  /**
   * This is the offset distance (cm) of our US sensor and the wheelbase axis
   */
  private static final double SENSOR_OFFSET_Y = 1;
  /**
   * The distance the robot moves downwards before detecting the y axis, in grid units
   */
  private static final double DOWN_DIST = 0.5;

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
   * The correct value of t when the light sensor is perpendicular to the y axis
   */
  private static final double CORRECT_T_AVG =
      (Math.toDegrees(Math.atan(Lab5.LINE_OFFSET_X/Lab5.LINE_OFFSET_Y)) + 270) % 360;

  private Odometer odo;
  private Navigation nav;
  private AveragedBuffer<Float> samples;
  private int x;
  private int y;


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
  }

  /**
   * Starts the thread.
   * When executed, the robot will turn to 270deg, and 
   * move backwards until the y axis is detected.
   * Then, it will move back to the center of the tile, and
   * turn 90deg down to face 180deg, moving backwards until the
   * x axis is detected. The robot will then navigate to the origin.
   */
  public void run() {
    // find y-axis
    nav.turnTo(90);
    moveToLine(true); //move forwards to a line (should be x=0 gridline)
    odo.setX(SENSOR_OFFSET_Y + (x+1) * OdometryCorrection.LINE_SPACING);
    //this line is used to make future steps more predictable
    odo.setY((y + 0.5) * OdometryCorrection.LINE_SPACING);
    
   
    
    //line up robot mid-point at with y-axis
    nav.travelTo(OdometryCorrection.LINE_SPACING * (x+1) - (2 * Lab5.LINE_OFFSET_Y / 3), odo.getXYT()[y]);
    while (nav.isNavigating()) {
      sleep();
    }
    //TODO: Add verification that the robot is hitting the same line!
    //TODO: Verify that this works!
    rotateToLine(true);
    double t1 = odo.getXYT()[2];
    rotateToLine(false);
    double t2 = odo.getXYT()[2];
    correctAngle(t1,t2);
    nav.travelTo(OdometryCorrection.LINE_SPACING * (x+0.5), 
        OdometryCorrection.LINE_SPACING * (y+0.5));
    while (nav.isNavigating()) {
      sleep();
    }
    
    // find x-axis
    nav.turnTo(0);
    moveToLine(true); //move forwards to a line (should be y=0 gridline)
    odo.setY(SENSOR_OFFSET_Y + (y+1) * OdometryCorrection.LINE_SPACING);
  }

  /**
   * Given two angles that represent where the robot's light sensor
   * interesected a straight line during a rotation, sets the
   * odometer theta to be correct
   * @param t1
   * @param t2
   */
  private void correctAngle(double t1, double t2) {
    double odoAvg = ((t1 + t2)/2 % 360);
    double err = CORRECT_T_AVG - odoAvg;
    odo.setTheta(odo.getXYT()[2] + err);
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
    nav.setSpeeds(dir * MOTOR_SPEED, -dir * MOTOR_SPEED);
    waitUntilLine();
    Sound.beep(); //found a line
    nav.setSpeeds(0, 0);
  }
  
  /**
   * Blocks until a line is detetected by the robot
   */
  private void waitUntilLine() {
    float[] sample = new float[Lab5.LINE_SENSOR.sampleSize()];
    do {
      Lab5.LINE_SENSOR.fetchSample(sample, 0);
      samples.add(sample[0]);
      Lab5.LCD.clear();
      Lab5.LCD.drawString(sample[0] + ", " + samples.getAvg() + "      ",0,4);
      sleep();
    } while (sample[0] > samples.getAvg() - LIGHT_THRESHOLD);
    Sound.beep();
  }

  /**
   * Sleeps for the default amount of time
   */
  private void sleep() {
    try {
      sleep(POLL_DELAY);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * Moves the robot forward (straight) a certain distance, using the odometer.
   * 
   * @param dist
   */
  private void moveForward(double dist) {
    nav.setSpeeds(MOTOR_SPEED,MOTOR_SPEED);
    double[] start = nav.getOdo().getXYT();
    Lab5.LEFT_MOTOR.forward();
    Lab5.RIGHT_MOTOR.forward();
    while (Navigation.dist(nav.getOdo().getXYT(), start) < dist) {
      try {
        sleep(30);
      } catch (InterruptedException e) {
      }
    }
    nav.setSpeeds(0, 0);
  }

}
