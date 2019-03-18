/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.demo.BetaDemo;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;


  // Multiply by degrees to get distance moved by a single wheel
  private final double DIST_MULT;

  private double[] position;


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms, equiv to 40Hz

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param LEFT_MOTOR
   * @param RIGHT_MOTOR
   * @throws OdometerExceptions
   */
  private Odometer() throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z manipulation methods


    // reset tacho count in motor
    BetaDemo.LEFT_MOTOR.resetTachoCount();
    BetaDemo.RIGHT_MOTOR.resetTachoCount();

    // Reset the values of x, y and z to defaults
    odoData.setXYT(0, 0, 0);

    // reset tacho count in motor
    BetaDemo.LEFT_MOTOR.resetTachoCount();
    BetaDemo.RIGHT_MOTOR.resetTachoCount();

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.DIST_MULT = (Math.PI * BetaDemo.WHEEL_RAD / 180);

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer();
      return odo;
    }
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  @Override
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      // Measure differences then update
      int leftDiff = BetaDemo.LEFT_MOTOR.getTachoCount() - leftMotorTachoCount;
      int rightDiff = BetaDemo.RIGHT_MOTOR.getTachoCount() - rightMotorTachoCount;

      leftMotorTachoCount += leftDiff;
      rightMotorTachoCount += rightDiff;


      double leftDist = leftDiff * DIST_MULT; // left wheel distance traveled
      double rightDist = rightDiff * DIST_MULT; // right wheel distance traveled
      double disp = 0.5 * (leftDist + rightDist); // vehicle displacement in the forward direction
      // (average)

      double dx, dy, dt; // displacement components in the x, y, and theta direction (heading


      dt = Math.toDegrees((leftDist - rightDist) / BetaDemo.TRACK);

      dx = disp * Math.sin(Math.toRadians(odo.getXYT()[2] + dt));
      dy = disp * Math.cos(Math.toRadians(odo.getXYT()[2] + dt));


      odo.update(dx, dy, dt);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
