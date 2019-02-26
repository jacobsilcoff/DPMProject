package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.color.CanColor;
import ca.mcgill.ecse211.color.ColorClassifier;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;

/**
 * This is a thread that takes a robot, which we assume is on LL and calibrated,
 * and searches the
 * designated search region for cans. We will search in strips of predefined width,
 * using the light sensor to detect cans at each lattice point.
 * 
 * @author jacob
 *
 */
public class CanFinder {

  /**
   * The amount the robot moves over, in cm, for each pass it completes
   * This is set to the line spacing, but could be generalized to any number
   * for aribtrary search patterns.
   */
  public static final double PASS_WIDTH = OdometryCorrection.LINE_SPACING;
  /**
   * The amount of time the system sleeps between ticks
   */
  public static final int SLEEP_TIME = 15;
  /**
   * Speed used to scan a can
   */
  public static final int SCAN_SPD = 40;
  
  /**
   * The spacing between tiles. Copied from odometry correction to save characters.
   */
  public static final float GRID_WIDTH = OdometryCorrection.LINE_SPACING;

  private Navigation nav;
  private Odometer odo;
  private CanColor target;
  private int nextX;
  private int nextY;


  /**
   * Creates a can finder.
   * @param nav The navigation to use to control the robot
   * @param target The color of can the robot is looking for
   */
  public CanFinder(Navigation nav, CanColor target) {
    this.target = target;
    this.nav = nav;
    nextX = 0;
    nextY = 1;
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
  }

  /**
   * Searches the area specified by Lab5.URx/y and Lab5.LLx/y
   * Goes in vertical passes, using the light sensor to detect
   * cans at lattice points.
   */
  public void run() {
    /*
     * Implementation comment:
     * The variables nextX and nextY specify
     * the lattice point that the robot will check
     * for a can on the current pass through the loop.
     */
    while (nextX <= Lab5.URx - Lab5.LLx) {
      int dir = nextX % 2 == 0 ? 1 : -1;
      /*
       * This represents the case of apporaching a can horizontally
       */
      if (((nextY == 0 && dir == 1) || (nextY == Lab5.URy - Lab5.LLy && dir == -1))) {
        nav.travelTo((nextX + Lab5.LLx) * GRID_WIDTH - Lab5.CAN_DIST,
            (nextY + Lab5.LLy) * GRID_WIDTH);
        awaitNav();
        if (Lab5.CLASSIFIER.canDetected()) {
          if (Lab5.CLASSIFIER.classify() == target) {
            Sound.twoBeeps();
          } else {
            Sound.beep();
          }
          //Avoids hitting the can
          moveBack(15);
          nav.travelTo((nextX + Lab5.LLx) * GRID_WIDTH, (Lab5.LLy + nextY + dir * 0.5) * GRID_WIDTH);
          awaitNav();
        }
        nextY += dir;
      } 
      /*
       * Represents the case where a can is approached vertically
       */
      else {
        nav.travelTo((nextX + Lab5.LLx) * GRID_WIDTH,
            (nextY + Lab5.LLy) * GRID_WIDTH - dir * Lab5.CAN_DIST);
        awaitNav();

        if (Lab5.CLASSIFIER.canDetected()) {
          if (Lab5.CLASSIFIER.classify() == target) {
            Sound.twoBeeps();
          } else {
            Sound.beep();
          }
          /*
           * Follows a different can avoidance technique
           * if it is at the end of a vertical pass
           */
          if ((nextY == 0 && dir == -1) || (nextY == Lab5.URy - Lab5.LLy && dir == 1)) {
            //END OF A PASS:
            moveBack(15);
            nav.travelTo((nextX + Lab5.LLx + 0.5) * GRID_WIDTH, (Lab5.LLy + nextY) * GRID_WIDTH);
            awaitNav();
          } else {
            // MIDDLE OF A PASS:
            moveBack(10);
            nav.travelTo((nextX + Lab5.LLx - 0.5) * GRID_WIDTH, odo.getXYT()[1]);
            awaitNav();
            nav.travelTo(odo.getXYT()[0], (nextY + Lab5.LLy + 0.5) * GRID_WIDTH);
            awaitNav();
            nav.travelTo((nextX + Lab5.LLx) * GRID_WIDTH, odo.getXYT()[1]);
            awaitNav();
          }
        }
        if ((nextY == 0 && dir == -1) 
            || (nextY == Lab5.URy - Lab5.LLy && dir == 1)) {
          nextX ++;
        } else {
          nextY += dir;
        }
      }
    }
  }
  
  /**
   * Used internally to wait for the
   * navigation to finish navigating to its
   * target location
   */
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
   * Sleeps for one tick, as specified
   * by the SLEEP_TIME variable
   */
  private void sleep() {
    try {
      Thread.sleep(SLEEP_TIME);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
