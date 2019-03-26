package ca.mcgill.ecse211.demo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import ca.mcgill.ecse211.canhandling.CanColor;
import ca.mcgill.ecse211.canhandling.Claw;
import ca.mcgill.ecse211.canhandling.ColorClassifier;
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
  private CanColor target;
  private Point nextCan;
  private State state;
  public static final float GRID_WIDTH = BetaDemo.GRID_WIDTH;
  public static final int SCAN_SPEED = 50;
  public static final int CAN_STOP_DIST = 15;
  public static final int TURN_SPEED = 100;
  public static final double CAN_RAD = 5;

  /**
   * Creates a can finder.
   * @param target The color of can the robot is looking for
   */
  public CanFinder() {
    this.target = GameSettings.targetColor;
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
    BetaDemo.NAV.turnTo(90);
    BetaDemo.NAV.setSpeeds(-SCAN_SPEED, SCAN_SPEED);
    double minDist = Double.MAX_VALUE;
    double[] minPt = new double[2];
    double minT = -1;
    while (odo.getXYT()[2] > 0.5 && odo.getXYT()[2] < 180) {
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
    double t = BetaDemo.NAV.angleTo(nextCan.x, nextCan.y);
    pt[0] -=  CAN_STOP_DIST * Math.sin(Math.toRadians(t));
    pt[1] -= CAN_STOP_DIST * Math.cos(Math.toRadians(t));
    return pt;
  }
  
  /**
   * Transports the robot from the starting zone to 
   * the search area
   */
  public void goToSearchArea() {
    BetaDemo.CLAW.close();
    if (GameSettings.initialized /*&& !GameSettings.searchZone.contains(odo.getXYT())*/) {
      if (!GameSettings.island.contains(odo.getXYT())) {
        Sound.buzz();
        //Get to island through tunnel 
        Rect tunnel = GameSettings.tunnel;
        Rect start = GameSettings.startZone;
        Rect island = GameSettings.island;
        double[] llBlock = {(tunnel.LLx + .5) * GRID_WIDTH, 
                            (tunnel.LLy + .5) * GRID_WIDTH};
        double[] urBlock = {(tunnel.URx - .5) * GRID_WIDTH, 
                            (tunnel.URy - .5) * GRID_WIDTH};
        double[] N = translate(urBlock, 0, GRID_WIDTH);
        double[] S = translate(llBlock, 0, -GRID_WIDTH);
        double[] E = translate(urBlock, GRID_WIDTH, 0);
        double[] W = translate(llBlock, -GRID_WIDTH, 0);
        //strictly one of N, S, E, W is contained in start
        double[] entrance = N, exit = S;
        if (start.contains(N) && island.contains(S)) {
          entrance = N; exit = S;
        } else if (start.contains(S) && island.contains(N)) {
          entrance = S; exit = N;
        } else if (start.contains(E) && island.contains(W)) {
          entrance = E; 
          exit = W;
        } else if (start.contains(W) && island.contains(E)){
          entrance = W;
          exit = E;
        }
        BetaDemo.NAV.travelTo(entrance[0], entrance[1]);
        BetaDemo.NAV.waitUntilDone();
        boolean ocOn = BetaDemo.OC.getOn();
        BetaDemo.OC.setOn(false);
        BetaDemo.NAV.travelTo(exit[0], exit[1]);
        BetaDemo.NAV.waitUntilDone();
        BetaDemo.OC.setOn(ocOn);
      }
      //Get to search area from island
      BetaDemo.NAV.travelTo(GameSettings.searchZone.LLx * GRID_WIDTH, 
                            GameSettings.searchZone.LLy * GRID_WIDTH);
      BetaDemo.NAV.waitUntilDone();
    }
  }
  
  /**
   * Modifies a point by adding x and y
   * @param pt the original point
   * @param x the translation x amt
   * @param y the translation yamt
   * @return the translated point
   */
  private static double[] translate(double[] pt, double x, double y) {
    return new double[] {pt[0] + x, pt[1] + y};
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
   * @return Returns true if a can was acquired succesfully
   */
  public boolean grabNextCan() {
    if (nextCan != null) {
      double[] stop = canStoppingPoint();
      BetaDemo.NAV.travelTo(stop[0], stop[1]);
      BetaDemo.NAV.waitUntilDone();
      BetaDemo.CLAW.open();
      BetaDemo.NAV.turnTo(BetaDemo.NAV.angleTo(nextCan.x, nextCan.y) + 180);
      moveBackward(10);
      BetaDemo.CLAW.close();
    } 
    //We no longer know what the next can is, because we just picked up the last one
    nextCan = null;
    return BetaDemo.CLAW.hasCan();
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
    float[] usData = new float[BetaDemo.US_FRONT.sampleSize()];
    BetaDemo.US_FRONT.fetchSample(usData, 0);
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
    BetaDemo.NAV.setSpeeds(TURN_SPEED,TURN_SPEED);
    double[] start = BetaDemo.NAV.getOdo().getXYT();

    BetaDemo.LEFT_MOTOR.backward();
    BetaDemo.RIGHT_MOTOR.backward();

    while (Navigation.dist(BetaDemo.NAV.getOdo().getXYT(), start) < Math.abs(dist)) {
      try {
        Thread.sleep(30);
      } catch (InterruptedException e) {
      }
    }
    BetaDemo.NAV.setSpeeds(0, 0);
  }
  
  /**
   * Moves the robot forwards (straight) a certain distance, using the odometer.
   * 
   * @param dist
   */
  private void moveForward(double dist) {
    BetaDemo.NAV.setSpeeds(TURN_SPEED,TURN_SPEED);
    double[] start = BetaDemo.NAV.getOdo().getXYT();

    BetaDemo.LEFT_MOTOR.forward();
    BetaDemo.RIGHT_MOTOR.forward();

    while (Navigation.dist(BetaDemo.NAV.getOdo().getXYT(), start) < Math.abs(dist)) {
      try {
        Thread.sleep(30);
      } catch (InterruptedException e) {
      }
    }
    BetaDemo.NAV.setSpeeds(0, 0);
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
    BetaDemo.CLAW.open();
    moveForward(15);
    BetaDemo.CLAW.close();
    moveBackward(15);
  }
}
