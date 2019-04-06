package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

/**
 * Enables the robot to be localized by
 * ramming into the walls
 * @author jacob
 *
 */
public class WallLocalizer {
  
  /**
   * Distance from front of US sensor to wheel center
   * in CM
   */
  public static final double US_OFFSET = 6.5;
  /**
   * The speed of the motors for the localization routine
   */
  public static final int MOTOR_SPEED =  250;
  
  public static void run() throws OdometerExceptions {
    FinalDemo.NAV.turnTo(270);
    moveForward(FinalDemo.GRID_WIDTH - US_OFFSET);
    Odometer.getOdometer().setTheta(270);
    Odometer.getOdometer().setX(US_OFFSET);
    moveForward(-(FinalDemo.GRID_WIDTH /2.5));
    
    FinalDemo.NAV.turnTo(180);
    moveForward(FinalDemo.GRID_WIDTH - US_OFFSET);
    Odometer.getOdometer().setTheta(180);
    Odometer.getOdometer().setY(US_OFFSET);
    moveForward(-(FinalDemo.GRID_WIDTH /2.));
 }
  
  /**
   * Moves the robot forward a certain distance
   * If a negative distance is entered, moves
   * backward that distance
   * @param dist The distance to move in cm
   */
  public static void moveForward(double dist) {
    FinalDemo.NAV.setSpeeds(MOTOR_SPEED,MOTOR_SPEED);
    double[] start = FinalDemo.NAV.getOdo().getXYT();
    
    if (dist < 0) {
      FinalDemo.LEFT_MOTOR.backward();
      FinalDemo.RIGHT_MOTOR.backward();
    } else {
      FinalDemo.LEFT_MOTOR.forward();
      FinalDemo.RIGHT_MOTOR.forward();
    }

    while (Navigation.dist(FinalDemo.NAV.getOdo().getXYT(), start) < Math.abs(dist)) {
      try {
        Thread.sleep(30);
      } catch (InterruptedException e) {
      }
    }
    FinalDemo.NAV.setSpeeds(0, 0);
  }
}
