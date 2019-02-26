package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.color.CanColor;
import ca.mcgill.ecse211.color.ColorClassifier;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;

/**
 * This is a thread that takes a robot, which we assume is on LL and calibrated, and searches the
 * designated search region for cans. We will search in strips of predefined width
 * 
 * @author jacob
 *
 */
public class CanFinder extends Thread {

  /**
   * The amount the robot moves over, in cm, for each pass it completes
   */
  public static final double PASS_WIDTH = OdometryCorrection.LINE_SPACING;
  /**
   * The amount of time the system sleeps between ticks
   */
  public static final int SLEEP_TIME = 15;
  /**
   * The distance at which point a can is detected
   */
  public static final int DETECTION_DIST = 20; // should be 25
  /**
   * The distance at which point a can is clearly not detected
   */
  public static final int NO_CAN = 50;
  /**
   * Speed used to scan a can
   */
  public static final int SCAN_SPD = 40;

  public static final float GRID_WIDTH = OdometryCorrection.LINE_SPACING;

  private Navigation nav;
  private Odometer odo;
  private CanColor target;
  private int nextX;
  private int nextY;



  public CanFinder(Navigation nav, CanColor target) {
    this.target = target;
    this.nav = nav;
    // pass num is the number of passes back & forth made by the robot
    nextX = 0;
    nextY = 1;
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
  }

  public void run() {
    while (nextX <= Lab5.URx - Lab5.URy) {
      int dir = nextX % 2 == 0 ? 1 : -1;
      if ((nextY == 0 && nextX % 2 == 0) || (nextY == Lab5.URy - Lab5.LLy && nextX % 2 == 1)) {
        // approach horizontally
        nav.travelTo((nextX + Lab5.LLx) * GRID_WIDTH - Lab5.CAN_DIST,
            (nextY + Lab5.LLy) * GRID_WIDTH);
        awaitNav();
        if (Lab5.CLASSIFIER.classify() != CanColor.UNKNOWN) {
          if (Lab5.CLASSIFIER.classify() == target) {
            Sound.twoBeeps();
          } else {
            Sound.beep();
          }

          moveBack(15);
          nav.travelTo((nextX + Lab5.LLx) * GRID_WIDTH, (Lab5.LLy + nextY + dir * 0.5));
          awaitNav();
        }
        nextY += dir;

      } else {
        // approaching vertically
        nav.travelTo((nextX + Lab5.LLx) * GRID_WIDTH,
            (nextY + Lab5.LLy) * GRID_WIDTH - dir * Lab5.CAN_DIST);
        awaitNav();
        CanColor color = Lab5.CLASSIFIER.classify();
        if (color != CanColor.UNKNOWN) {
          if (color == target) {
            Sound.twoBeeps();
          } else {
            Sound.beep();
          }
          if ((nextY == 0 && nextX % 2 == 1) || (nextY == Lab5.URy - Lab5.LLy && nextX % 2 == 0)) {
            // end of a pass
            moveBack(15);
            nav.travelTo((nextX + Lab5.LLx + 0.5) * GRID_WIDTH, (Lab5.LLy + nextY) * GRID_WIDTH);
            awaitNav();
            nextX++;
          } else {
            // Avoid collision
            moveBack(10);
            nav.travelTo((nextX + Lab5.LLx - 0.5) * GRID_WIDTH, odo.getXYT()[1]);
            awaitNav();
            nav.travelTo(odo.getXYT()[0], (nextY + Lab5.LLy + 0.5) * GRID_WIDTH);
            awaitNav();
            nav.travelTo((nextX + Lab5.LLx) * GRID_WIDTH, odo.getXYT()[1]);
            awaitNav();
          }
        }
        nextY += dir;
      }

    }
  }

  private void awaitNav() {
    while (nav.isNavigating())
      sleep();
  }

  /**
   * Moves backwards a certain distance
   * 
   * @param dist The distance to move, in cm
   */
  private void moveBack(float dist) {
    double[] start = odo.getXYT();
    nav.setSpeeds(-SCAN_SPD, -SCAN_SPD);
    while (Navigation.dist(start, odo.getXYT()) < dist) {
      sleep();
    }
    nav.setSpeeds(0, 0);
  }

  /**
   * Lines the robot up with the can to be scanned.
   */
  private void alignWithCan() {
    nav.setSpeeds(30, 30);
    while (readUS() > Lab5.CAN_DIST) {
      sleep();
    }
    nav.setSpeeds(0, 0);
  }

  private void sleep() {
    try {
      sleep(SLEEP_TIME);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * Polls the ultrasonic sensor and returns the result
   * 
   * @return The US reading in cm
   */
  public float readUS() {
    float[] usData = new float[Lab5.US_FRONT.sampleSize()];
    Lab5.US_FRONT.fetchSample(usData, 0);
    Lab5.LCD.drawString("US:" + (usData[0] * 100.0) + ".........", 0, 0);
    if (usData[0] == 255) {
      return -1;
    }
    return usData[0] * 100f;
  }
}
