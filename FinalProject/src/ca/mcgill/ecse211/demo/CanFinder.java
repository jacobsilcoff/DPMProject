package ca.mcgill.ecse211.demo;

import java.util.ArrayList;
import java.util.List;
import ca.mcgill.ecse211.canhandling.CanColor;
import ca.mcgill.ecse211.canhandling.Claw;
import ca.mcgill.ecse211.canhandling.ColorClassifier;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.wifi.GameSettings;
import lejos.hardware.Sound;

/**
 * Gives the robot the ability to search for cans
 * 
 * @author jacob
 *
 */
public class CanFinder implements Runnable {

  private Navigation nav;
  private Odometer odo;
  private CanColor target;
  private Point nextCan;
  private State state;


  /**
   * Creates a can finder.
   * @param nav The navigation to use to control the robot
   * @param target The color of can the robot is looking for
   */
  public CanFinder(Navigation nav, CanColor target) {
    this.target = target;
    this.nav = nav;
    nextCan = null;
    state = State.INIT;
    try {
      odo = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
  }
  
  private enum State {
    INIT, NAV_TO_SEARCH, FIND_CAN, GRAB_CAN, NAV_TO_START, DROPOFF;
  }
  
  /**
   * Continuously takes cans from the search zone
   * and returns them to the start area
   */
  public void run() {
    while (true) {
      switch (state) {
        case INIT:
          state = State.NAV_TO_SEARCH;
          break;
        case NAV_TO_SEARCH:
          goToSearchArea();
          if (GameSettings.searchZone.contains(odo.getXYT()[0], odo.getXYT()[1])) {
            state = State.FIND_CAN;
          }
          break;
        case FIND_CAN:
          search();
          state = State.GRAB_CAN;
          break;
        case GRAB_CAN:
          grabNextCan();
          state = State.NAV_TO_START;
          break;
        case NAV_TO_START:
          goToStart();
          state = State.DROPOFF;
          break;
        case DROPOFF:
          dropOffCan();
          state = State.INIT;
          break;
      }
      sleep();
    }
  }
  
  /**
   * Finds the next can to grab
   * 
   * Uses the sideways facing ultrasonic sensor
   */
  public void search() {
    /*
     * TODO: Scan along search zone to give nextCan a value
     */
  }
  
  /**
   * Transports the robot from the starting zone to 
   * the search area
   */
  public void goToSearchArea() {
    if (GameSettings.searchZone.contains(odo.getXYT()[0], odo.getXYT()[1])) {
      //Already in search area...
      return;
    }
    //TODO: Implement
  }
  
  /**
   * Transports the robot from the search zone to
   * the starting area
   */
  public void goToStart() {
    if (GameSettings.startZone.contains(odo.getXYT()[0], odo.getXYT()[1])) {
      //Already in start area...
      return;
    }
    //TODO: Implement
  }
  
  /** 
   * Navigates to the next can (assuming it is defined)
   * Analyzes its color and weight, and picks it up with the claw
   */
  public void grabNextCan() {
    if (nextCan == null) {
      return; //no next can defined
    } 
    
    //We no longer know what the next can is, because we just picked up the last one
    nextCan = null;
  }
  
  /**
   * Drops off a can in the start zone
   */
  public void dropOffCan() {
    //TODO: Implement
  }

  

  /**
   * Sleeps for one tick, as specified
   * by the SLEEP_TIME variable
   */
  private void sleep() {
    try {
      Thread.sleep(30);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
  
  /**
   * Polls the ultrasonic sensor and returns the result
   * 
   * @return The US reading in cm
   */
  private float readUS() {
    float[] usData = new float[Demo.US_FRONT.sampleSize()];
    Demo.US_FRONT.fetchSample(usData, 0);
    if (usData[0] == 255) {
      return -1;
    }
    return usData[0] * 100f;
  }
  
  /**
   * Represents a 2D Point
   * @author jacob
   */
  private class Point {
    float x;
    float y;
    Point(float x, float y) {
      this.x = x; 
      this.y = y;
    }
  }
}
