package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.demo.Demo;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;


/**
 * A class used to navigate the robot according to waypoints and obstacles.
 * @author group 6
 */
public class Navigation extends Thread {
  /**
   * The motor speed of the robot when moving forward
   */
  private static final int FORWARD_SPEED = 250;
  /**
   * The motor speed used by the robot when turning
   */
  private static final int ROTATE_SPEED = 100;
  /**
   * The maximum distance between two points where they are considered to be roughly equal.
   */
  private static final double DIST_THRESH = 0.5;
  /**
   * The distance at which an object is considered to be close enough to the robot to initiate the
   * emergency obstacle avoidance sequence
   */
  public static final double EMERGENCY_THRESH = 17;
  /**
   * The max difference between two angles where they are considered to be roughly equal
   */
  private static final double T_THRESH = 0.8;

  /**
   * The amount of time, in ms, that the thread will sleep for in between cycles
   */
  private static final int SLEEP_TIME = 20;

  /**
   * The distance after which the robot will no longer attempt to update its heading
   */
  private static final int CORRECTION_DIST = 4;

  /**
   * The distance were the robot starts to slow down
   */
  private static final int SLOW_DIST = 5;
  /**
   * The slowest the robot will spin its motors
   */
  private static final int MIN_SPEED = 50;


  private boolean isNavigating;
  private Odometer odo;
  private double destX;
  private double destY;
  private double destT;
  private OdometryCorrection correction;
  private boolean on;

  /**
   * Default constructor for navigation (called in Lab3.java)
   * 
   * @param correction The odometer correction that is used for the robot
   * @throws OdometerExceptions
   */
  public Navigation(OdometryCorrection correction) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    isNavigating = false;
    destX = destY = destT = 0;
    this.correction = correction;
    on = true;
  }

  /**
   * Sets robot to travel to a given TILE point, updating the destination direction and position
   * It is recommended that this method is called in conjunction with
   * waitUntilDone()
   * @param x The desired x in cm
   * @param y The desired y in cm
   */
  public void travelTo(double x, double y) {
    destX = x ; 
    destY = y; // convert Y tile pt
    updateT();
    isNavigating = true;
    Demo.LCD.drawString("Dest:" + (int) destX + "," + (int) destY + "," + (int) destT, 0, 4);

  }

    /**
     * Kills the thread
     */
   public void end() {
     if (correction != null) {
       correction.setOn(true);
     }
     on = false;
   }
  /**
   * Returns the odometer corresponding to this navigation thread. Although this method is
   * technically redundant due to the fact that Odometer is represented as a singleton, it is useful
   * to keep this in place in case we want to modify odometer later (say, to have another odometer
   * keeping track of a different set of motors)
   * 
   * @return The odometer measuring the robots position
   */
  public Odometer getOdo() {
    return odo;
  }



  /**
   * This method checks isNavigating variable
   * 
   * @return true if another thread has called travelTo() or turnTo() and has yet to return;
   *         otherwise false
   */
  public boolean isNavigating() {
    return isNavigating;
  }


  /**
   * Turns the robot on the spot to a given angle theta using the MINIMUM angle
   * 
   * @param theta The desired absolute angle in degrees
   * @param speed The turning speed
   */
  public void turnTo(double theta, int speed) {
    if (correction != null) {
      correction.setOn(false); 
    }
    double presTheta = odo.getXYT()[2]; // get current heading
    double ang = (theta - presTheta + 360) % 360; // gets absolute angle required to turn
    Demo.LEFT_MOTOR.setSpeed(speed);
    Demo.RIGHT_MOTOR.setSpeed(speed);

    // turn using MINIMUM angle
    if (ang < 180) {
      Demo.LCD.drawString("Ang: " + ang + "deg  ", 0, 5);
      // increase angle
      Demo.LEFT_MOTOR.rotate(convertAngle(ang), true);
      Demo.RIGHT_MOTOR.rotate(-convertAngle(ang), false);
    } else {
      ang = 360 - ang;
      Demo.LCD.drawString("Ang: " + ang + "deg   ", 0, 5); // display angle of rotation
      // Need to check against odometer
      Demo.LEFT_MOTOR.rotate(-convertAngle(ang), true);
      Demo.RIGHT_MOTOR.rotate(convertAngle(ang), false);
    }
    updateT();// update new angle after turn;
  }

  /**
   * Turns the robot on the spot to a given angle theta at a default speed
   * 
   * @param theta The desired absolute angle in degrees
   * @param speed The turning speed
   */
  public void turnTo(double theta) {
    turnTo(theta, ROTATE_SPEED);
  }


  /**
   * An enumeration of states the robot can be in as it navigates from point to point
   * 
   * @author jacob silcoff
   */
  enum State {
    INIT, TURNING, TRAVELING
  }

  /**
   * Implements a state machine of initializing, turning traveling, or handling an emergency
   * obstacle
   */
  @Override
  public void run() {
    State state = State.INIT;
    while (on) {
      double[] lastPos = {0, 0};
      switch (state) {
        case INIT:
          Demo.LCD.drawString("State: INIT", 0, 6);
          if (isNavigating) {
            state = State.TURNING;
          }
          if (correction != null) {
            correction.setOn(false);
          }
          break;
        case TURNING:
          Demo.LCD.drawString("State: TURN", 0, 6);
          turnTo(destT);
          if (facing(destT)) {
            state = State.TRAVELING;
            lastPos = odo.getXYT();
          }
          break;
        case TRAVELING:
          Demo.LCD.drawString("State: TRVL", 0, 6);
          updateT();
          if (getDist() > CORRECTION_DIST && dist(lastPos, odo.getXYT()) > CORRECTION_DIST
              && !facing(destT)) {
            // re-check heading and finish turning
            state = State.TURNING;
          } else if (!checkIfDone()) {
            if (correction!=null) {
              correction.setOn(true);
            }
            updateTravel();
          } else { // Arrived
            setSpeeds(0, 0); // stop
            isNavigating = false; // finished traveling
            state = State.INIT; // return to initialize case
          }
          break;
      }
      try {
        sleep(SLEEP_TIME);
      } catch (InterruptedException e) {
      }
    }
  }
  
  /**
   * Waits until the thread is finished navigating
   * Uses 1/2 second intervals to save compute time
   */
  public void waitUntilDone() {
    while (isNavigating) {
      try {
        sleep(500);
      } catch (InterruptedException e) {
      }
    }
  }


  /**
   * Slows the motor speeds when nearing destination (<20cm)
   */
  private void updateTravel() {
    double dist = getDist();
    // slows down upon nearing destination
    if (dist > DIST_THRESH) {
      double speed = FORWARD_SPEED;
      if (dist < SLOW_DIST) {
        speed = dist / SLOW_DIST * (FORWARD_SPEED - MIN_SPEED) + MIN_SPEED;
      }
      setSpeeds((float) speed, (float) speed);
    } else {
      setSpeeds(0, 0);
    }
    Demo.LEFT_MOTOR.forward();
    Demo.RIGHT_MOTOR.forward();
  }

  /**
   * Gets distance from current position to destination
   * 
   * @return The distance from the current position to the destination, in cm
   */
  public double getDist() {
    return dist(new double[] {destX, destY}, odo.getXYT());
  }

  /**
   * Gets the angle from the robot to the destination
   * 
   * @return the angle from the robot to the destination, in cm
   */
  public double getDestT() {
    return destT;
  }


  /**
   * Updates the destT (heading) to reflect the real position of the robot
   */
  private void updateT() {
    double dx = destX - odo.getXYT()[0];
    double dy = destY - odo.getXYT()[1];
    if (dy == 0) {
      destT = (dx > 0) ? 90 : 270;
    } else {
      destT = Math.toDegrees(Math.atan(dx / dy)) + ((dy > 0) ? 0 : 180);
      destT = (destT + 360) % 360; // normalize theta
    }
  }
  
  public double angleTo(double x, double y) {
    double dx = x - odo.getXYT()[0];
    double dy = y - odo.getXYT()[1];
    if (dy == 0) {
      return (dx > 0) ? 90 : 270;
    } else {
      double d = Math.toDegrees(Math.atan(dx / dy)) + ((dy > 0) ? 0 : 180);
      return (d + 360) % 360; // normalize theta
    }
  }

  /**
   * Checks if the robot is facing a certain angle
   * 
   * @param ang The angle to check
   * @return True if the robot is facing the given angle, false otherwise
   */
  private boolean facing(double ang) {
    double diff = Math.abs(odo.getXYT()[2] - (ang + 360) % 360);
    diff = (diff + 360) % 360;
    return (diff < T_THRESH) || ((360 - diff) < T_THRESH);
  }

  /**
   * Checks if the robot has arrived at its destination
   * 
   * @return True if the robot has arrived, false otherwise
   */
  private boolean checkIfDone() {
    return dist(odo.getXYT(), new double[] {destX, destY}) < DIST_THRESH;
  }

  /**
   * Sets the speeds of both motors
   * 
   * @param l The desired speed of the left motor
   * @param r The desired speed of the right motor
   */
  public void setSpeeds(float l, float r) {
    Demo.LEFT_MOTOR.setSpeed(Math.abs(l));
    Demo.RIGHT_MOTOR.setSpeed(Math.abs(r));
    if (l > 0) {
      Demo.LEFT_MOTOR.forward();
    } else if (l < 0) {
      Demo.LEFT_MOTOR.backward();
    } else {
      Demo.LEFT_MOTOR.stop();
    }
    
    if (r > 0) {
      Demo.RIGHT_MOTOR.forward();
    } else if (r < 0) {
      Demo.RIGHT_MOTOR.backward();
    } else {
      Demo.RIGHT_MOTOR.stop();
    }
  }

  /**
   * Sets the speeds of both motors
   * 
   * @param l The desired speed of the left motor
   * @param r The desired speed of the right motor
   */
  private void setSpeeds(int l, int r) {
    setSpeeds((float) l, (float) r);
  }
  
  /**
   * Takes a given distance, and returns the number of degrees the robot's wheels need to turn to
   * move forward that distance.
   * 
   * @param distance The distance the robot moves forward, in cm
   * @return The number of degrees of wheel rotation needed for the given distance
   */
  private static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * Demo.WHEEL_RAD));
  }


  /**
   * Takes a given angle, and returns the number of degrees the robot's wheels need to turn for the
   * entire robot to rotate that angle.
   * 
   * @param angle The desired angle for the robot to turn, in degrees
   * @return The number of degrees of wheel rotation needed for the given angle
   */
  private static int convertAngle(double angle) {
    return convertDistance(Math.PI * Demo.TRACK * angle / 360.0);
  }

  /**
   * Gets distance (cm) between two coordinates
   * 
   * @param a position array 1 where a[0] is its x, and a[1] is its y
   * @param b position array 2 where b[0] is its x, and b[1] is its y
   * @return The distance between a and b, in cm
   */
  public static double dist(double[] a, double[] b) {
    if (a.length < 2 || b.length < 2) {
      return -1;
    }
    return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2)); // minimum distance
                                                                           // formula
  }

}
