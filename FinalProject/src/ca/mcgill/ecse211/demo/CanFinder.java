package ca.mcgill.ecse211.demo;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import ca.mcgill.ecse211.canhandling.CanColor;
import ca.mcgill.ecse211.canhandling.Claw;
import ca.mcgill.ecse211.canhandling.ColorClassifier;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.wifi.GameSettings;
import ca.mcgill.ecse211.wifi.Rect;
import lejos.hardware.Sound;

/**
 * Gives the robot the ability to search for cans
 * 
 * @author jacob
 *
 */
public class CanFinder implements Runnable {

  private Odometer odo;
  private Point nextCan;
  private State state;
  public static final float GRID_WIDTH = FinalDemo.GRID_WIDTH;
  public static final int SCAN_SPEED = 50;
  public static final int CAN_STOP_DIST = 15;
  public static final int TURN_SPEED = 100;
  public static final double CAN_RAD = 5;

  /**
   * Creates a can finder.
   * @param target The color of can the robot is looking for
   */
  public CanFinder() {
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
    FinalDemo.NAV.turnTo(GameSettings.searchAngles[0]);
    FinalDemo.NAV.setSpeeds(SCAN_SPEED, -SCAN_SPEED);
    double minDist = Double.MAX_VALUE;
    double[] minPt = new double[2];
    double minT = -1;
    while (odo.getXYT()[2] < GameSettings.searchAngles[1] 
        && odo.getXYT()[2] + 2 >= GameSettings.searchAngles[0]) {
      float dist = readUS();
      double t = (odo.getXYT()[2] - Math.toDegrees(CAN_RAD/dist) + 360) % 360;
      double[] pt = pointFromDist(dist, t);
      if (GameSettings.searchZone.contains(pt)) {
        if (dist < minDist) {
          minDist = dist;
          minPt = pt;
          minT = t;
        }
      }
      sleep();
    }
    if (minT == - 1) {
      nextCan = null;
    } else {
      nextCan = new Point((float)minPt[0], (float)minPt[1]);
    }
  }
  
  /**
   * Gives the point located a given distance
   * from the front of the robot
   * @param d the distance from the robot
   * @return 
   */
  private double[] pointFromDist(double d) {
    double[] pt = odo.getXYT();
    pt[0] += d * Math.sin(Math.toRadians(pt[2]));
    pt[1] += d * Math.cos(Math.toRadians(pt[2]));
    return pt;
  }
  
  private double[] pointFromDist(double d, double t) {
    double[] pt = odo.getXYT();
    pt[0] += d * Math.sin(Math.toRadians(t));
    pt[1] += d * Math.cos(Math.toRadians(t));
    return pt;
  }
  
  /**
   * Gets the point to navigate to
   * to be a comfortable distance away from 
   * the can
   * @return
   */
  private double[] canStoppingPoint() {
    double[] pt = {nextCan.x, nextCan.y};
    double t = FinalDemo.NAV.angleTo(nextCan.x, nextCan.y);
    pt[0] -=  CAN_STOP_DIST * Math.sin(Math.toRadians(t));
    pt[1] -= CAN_STOP_DIST * Math.cos(Math.toRadians(t));
    return pt;
  }
  
  
  
  
  /**
   * Transports the robot from the starting zone to 
   * the search area
   * Must be in the starting area to work 
   */
  public void goToSearchArea() {
    FinalDemo.CLAW.close();
    if (GameSettings.initialized) {
      if (!GameSettings.island.contains(odo.getXYT())) {
        //Get to island through tunnel 
        preTunnelLocalize();
        FinalDemo.NAV.turnTo(0);
        FinalDemo.NAV.travelTo(GameSettings.tunnelEntrance[0], GameSettings.tunnelEntrance[1]);
        FinalDemo.NAV.waitUntilDone();
        boolean ocOn = FinalDemo.OC.getOn();
        FinalDemo.OC.setOn(false);
        FinalDemo.NAV.travelTo(GameSettings.tunnelExit[0], GameSettings.tunnelExit[1]);
        FinalDemo.NAV.waitUntilDone();
        FinalDemo.OC.setOn(ocOn);
      }
      //Get to search area from island
      FinalDemo.NAV.travelTo(GameSettings.startSearch[0] * GRID_WIDTH, 
                            GameSettings.startSearch[1] * GRID_WIDTH);
      FinalDemo.NAV.waitUntilDone();
    }
  }
  
  /**
   * Localizes the robot before traveling through the tunnel
   */
  private void preTunnelLocalize() {
    double[] locPoint = GameSettings.safeLoc;
    FinalDemo.NAV.travelTo(locPoint[0],locPoint[1]);
    FinalDemo.NAV.waitUntilDone();
    try {
      (new LightLocalizer((int) Math.round(locPoint[0]/FinalDemo.GRID_WIDTH - 1),
                         (int) Math.round(locPoint[1]/FinalDemo.GRID_WIDTH -1)))
      .run();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    
  }
  
  /**
   * Transports the robot from the search zone to
   * the starting area
   */
  public void goToStart() {
    FinalDemo.CLAW.close();
    if (GameSettings.initialized) {
      if (!GameSettings.startZone.contains(odo.getXYT())) {
        Sound.buzz();
        //Get to island through tunnel 
        FinalDemo.NAV.travelTo(GameSettings.tunnelExit[0], GameSettings.tunnelExit[1]);
        FinalDemo.NAV.waitUntilDone();
        boolean ocOn = FinalDemo.OC.getOn();
        FinalDemo.OC.setOn(false);
        FinalDemo.NAV.travelTo(GameSettings.tunnelEntrance[0], GameSettings.tunnelEntrance[1]);
        FinalDemo.NAV.waitUntilDone();
        FinalDemo.OC.setOn(ocOn);
      }
      FinalDemo.NAV.waitUntilDone();
    }
  }
  
  /** 
   * Navigates to the next can (assuming it is defined)
   * Analyzes its color and weight, and picks it up with the claw
   * @return Returns true if a can was acquired succesfully
   */
  public boolean grabNextCan() {
    if (nextCan != null) {
      double[] stop = canStoppingPoint();
      FinalDemo.NAV.travelTo(stop[0], stop[1]);
      FinalDemo.NAV.waitUntilDone();
      FinalDemo.CLAW.open();
      FinalDemo.NAV.turnTo(FinalDemo.NAV.angleTo(nextCan.x, nextCan.y) + 180);
      moveBackward(10);
      FinalDemo.CLAW.close();
    } 
    //We no longer know what the next can is, because we just picked up the last one
    nextCan = null;
    return FinalDemo.CLAW.hasCan();
  }
  
  private static double minAngle(double x, double y) {
    double a = ((x - y) + 360) % 360;
    return (a < 180) ? a : 360 - a;
  }
  
  /**
   * Returns whether or not a next can to pickup
   * has been found
   * @return
   */
  public boolean hasNextCan() {
    return nextCan != null;
  }
  
  /**
   * Drops off a can in the start zone
   */
  public void dropOffCan() {
    goToStart();
    Point2D startCorner = GameSettings.getStartingCornerPoint();
    FinalDemo.NAV.travelTo(startCorner.getX(), startCorner.getY());
    FinalDemo.NAV.waitUntilDone();
    FinalDemo.NAV.turnTo(odo.getXYT()[2] + 180);
    ejectCan();
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
   * Sleeps for one tick, as specified
   * by x
   * @param x the time to sleep in ms
   */
  private void sleep(int x) {
    try {
      Thread.sleep(x);
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
    float[] usData = new float[FinalDemo.US_FRONT.sampleSize()];
    FinalDemo.US_FRONT.fetchSample(usData, 0);
    if (usData[0] == 255) {
      return -1;
    }
    return usData[0] * 100f;
  }
  /**
   * Moves the robot backwards (straight) a certain distance, using the odometer.
   * 
   * @param dist
   */
  private void moveBackward(double dist) {
    FinalDemo.NAV.setSpeeds(TURN_SPEED,TURN_SPEED);
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
  
  /**
   * Moves the robot forwards (straight) a certain distance, using the odometer.
   * 
   * @param dist
   */
  private void moveForward(double dist) {
    FinalDemo.NAV.setSpeeds(TURN_SPEED,TURN_SPEED);
    double[] start = FinalDemo.NAV.getOdo().getXYT();

    FinalDemo.LEFT_MOTOR.forward();
    FinalDemo.RIGHT_MOTOR.forward();

    while (Navigation.dist(FinalDemo.NAV.getOdo().getXYT(), start) < Math.abs(dist)) {
      try {
        Thread.sleep(30);
      } catch (InterruptedException e) {
      }
    }
    FinalDemo.NAV.setSpeeds(0, 0);
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

  public void ejectCan() {
    FinalDemo.CLAW.open();
    moveForward(15);
    FinalDemo.CLAW.close();
    moveBackward(15);
  }
}
