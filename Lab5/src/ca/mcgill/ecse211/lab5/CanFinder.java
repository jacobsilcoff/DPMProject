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
  public static final float GRID_WIDTH = OdometryCorrection.LINE_SPACING;
  
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
    while (passNum * PASS_WIDTH < Lab5.URx * GRID_WIDTH) {
      
      nav.travelTo(passNum * PASS_WIDTH + Lab5.LLx * GRID_WIDTH, 
          (passNum % 2 == 0 ? Lab5.URy : Lab5.LLy) * GRID_WIDTH);
      
      while (nav.isNavigating()) {
        if (readUS() < DETECTION_DIST) { //initiate can handling sequence
          nav.setSpeeds(0, 0);
          handleCan();
          nav.travelTo(passNum * PASS_WIDTH + Lab5.LLx * GRID_WIDTH, 
              (passNum % 2 == 0 ? Lab5.URy : Lab5.LLy) * GRID_WIDTH);
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
    /*
     * Implementation concept:
     * first, center the can by turning left and right to find the
     * 'edge' of the can with the ultrasonic sensor. The angle bisector
     * of the angles the robot was at when each edge was detected is the angle
     * pointing straight towards the can. Move until the can is the right 
     * distance away, as measured by the US sensor. Use the color identification
     * program in the color package to read the color. Then, use the navigation to
     * back up, turn 90deg ccw, move forward a certain distance to give clearance for the can,
     * turn 90deg cw, move forward a bit, turn 90deg cw, move forward.
     */ 
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
