package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;

/**
 * This is a thread that takes a robot, which we assume is
 * on LL and calibrated, and searches the designated search region
 * for cans. We will search in strips of predefined width
 * @author jacob
 *
 */
public class CanFinder extends Thread {
  
  /**
   * The amount the robot moves over, in cm, for each pass it completes
   */
  public static final double PASS_WIDTH = 10;
  /**
   * The amount of time the system sleeps between ticks
   */
  public static final int SLEEP_TIME = 15;
  /**
   * The distance at which point a can is detected
   */
  public static final int DETECTION_DIST = 25;
  
  private int passNum;
  private Navigation nav;
  private Odometer odo;
  
  
  
  
  public CanFinder(Navigation nav) {
    this.nav = nav;
    //pass num is the number of passes back & forth made by the robot
    passNum = 0;
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
  }
  
  public void run() {
    while (passNum * PASS_WIDTH < Lab5.URx) {
      
      nav.travelTo(passNum * PASS_WIDTH + Lab5.LLx, 
          (passNum % 2 == 0 ? Lab5.URy : Lab5.LLy) * OdometryCorrection.LINE_SPACING);
      
      while (nav.isNavigating()) {
        if (readUS() < DETECTION_DIST) { //initiate can handling sequence
          handleCan();
        }
        sleep();
      }
      
      nav.travelTo(odo.getXYT()[0] + PASS_WIDTH, odo.getXYT()[1]);
      while (nav.isNavigating()) sleep();
      
      passNum++;
    }
  }
  
  /**
   * Centers a can under the sensor,
   * reads its color, and navigates the robot
   * to the other side of the can so it can
   * continue with navigation.
   */
  public void handleCan() {
    //TODO: Implement this!
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
    Lab5.LCD.drawString("US:" + (usData[0] * 100.0) + ".........", 0, 7);
    if (usData[0] == 255) {
      return -1;
    }
    return usData[0] * 100f;
  }
}
