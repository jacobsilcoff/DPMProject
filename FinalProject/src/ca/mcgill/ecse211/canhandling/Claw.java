package ca.mcgill.ecse211.canhandling;

import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.navigation.Navigation;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.UnregulatedMotor;

/**
 * Represents the robot's claw
 * @author jacob
 */
public class Claw {
  
  /**
   * The claw motor
   */
  private static final UnregulatedMotor CLAW_MOTOR =
      new UnregulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The can classifier used by the Claw
   */
  public static final ColorClassifier CLASSIFIER = new ColorClassifier();
  /**
   * The angle the claw is at when it is fully closed
   */
  public static final int CLOSED_ANGLE = 170;
  /**
   * The standard power used by the claw
   */
  public static final int CLAW_POWER = 25;
  /**
   * The time taken by the claw to weight the robot
   */
  public static final int WEIGHT_TIME = 3000;
  /**
   * Whether or not the tachometer for the claw has
   * been calibrated
   */
  private static boolean calibrated = false;

  /*
   * Weighing variables:
   */
  /**
   * The speed of the wheel motors used when
   * navigating during the weighing routine
   */
  public static final int MOTOR_SPEED = 100;
  /**
   * The distance backwards to move when weighing the can.
   */
  public static final int BACK_DISTANCE = 3;
  /**
   * The power value at which the robot can move
   * light cans but not heavy cans.
   * Relies on friction conditions of the demo table,
   * ie, this would need to be adjusted for the specific
   * surface used.
   */
  public static final int THRESH_POWER = 15;
  /**
   * The angle past which the robot can assume it is 
   * weighing a light can.
   */
  public static final int LIGHT_ANGLE = 165;
  
  /**
   * Creates a claw
   */
  public Claw() {
    if (!calibrated) {
      calibrate();
    }
  }
  
  /**
   * Calibrates the claw's tachometer by
   * moving the claw all the way open (to a known stop point)
   */
  public void calibrate() {
    CLAW_MOTOR.setPower(CLAW_POWER);
    CLAW_MOTOR.backward();
    sleep(2000);
    CLAW_MOTOR.setPower(0);
    CLAW_MOTOR.resetTachoCount();
    calibrated = true;
  }

  /**
   * Assuming a can is being currently held,
   * classifies the can based on mass and color
   * and beeps according to the final demo specs
   */
  public void classifyAndBeep() {
    CanColor c = getColor();
    int time = isHeavy() ? 1000 : 500;
    close();
    int numBeeps;
    Sound.setVolume(100);
    switch (c) {
      case RED:
        numBeeps = 4;
        break;
      case YELLOW:
        numBeeps = 3;
        break;
      case GREEN:
        numBeeps = 2;
        break;
      case BLUE:
        numBeeps = 1;
        break;
      default: 
        numBeeps = 0;
    }
    for (int i = 0; i < numBeeps; i++) {
        Sound.playTone(440, time);
        sleep(100);
    }
    Sound.setVolume(0);
  }
  /**
   * Closes the claw. If a jam is detected,
   * the claw will first ramp up the power,
   * and if the jam persists, the robot will open
   * the claw and move back before trying again.
   * This is to avoid the case where two cans are caught,
   * preventing propper functioning of the claw.
   */
  public void close() {
    CLAW_MOTOR.setPower(CLAW_POWER);
    CLAW_MOTOR.forward();
    int i = 0;
    while (CLAW_MOTOR.getTachoCount() < (CLOSED_ANGLE + LIGHT_ANGLE)/2.0) {
      sleep(30);
      i++;
      if (i*30 > 2000) {
        CLAW_MOTOR.setPower(45);
      }
      if (i*30 > 5000) {
        open();
        moveBackwards(5);
        CLAW_MOTOR.setPower(CLAW_POWER);
        CLAW_MOTOR.forward();
        i = 0;
      }
    }
    CLAW_MOTOR.setPower(CLAW_POWER);
  }
  /**
   * Moves the robot backwards (straight) a certain distance, using the odometer.
   * @param dist
   */
  private static void moveBackwards(double dist) {
    FinalDemo.NAV.setSpeeds(MOTOR_SPEED,MOTOR_SPEED);
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
   * Opens the claw
   */
  public void open() {
    CLAW_MOTOR.setPower(CLAW_POWER);
    CLAW_MOTOR.backward();
    while (CLAW_MOTOR.getTachoCount() > 3) {
      sleep(30);
    }
    CLAW_MOTOR.setPower(0);
    CLAW_MOTOR.stop();
  }

  /**
   * Returns whether or not a can is in the claw
   * @return True if a can is held in the claw
   */
  public boolean hasCan() {
    return CLASSIFIER.canDetected();
  }
  
  /**
   * Gets the color of the can currently held by 
   * the claw. If no can is detected, returns CanColor.UNKONWN
   * @return The color of the can held in the claw
   */
  public CanColor getColor() {
    return CLASSIFIER.classify();
  }

  /**
   * Checks if the can in the claw is heavy or not
   * by trying to pull the can inwards. When this method is called,
   * a can must be held in the claw so it is at a known position.
   * Reuqires the robot to have clearance of distance BACK_DISTANCE
   * directly behind it.
   * @param nav The navigation used to move the robot for this routine
   * @return true for heavy, else false
   */
  public boolean isHeavy() {
    open();
    moveForward(BACK_DISTANCE);
    CLAW_MOTOR.setPower(THRESH_POWER);
    CLAW_MOTOR.forward();
    try {
      Thread.sleep(WEIGHT_TIME);
    } catch (InterruptedException ie) {
      ie.printStackTrace();
    }
    return CLAW_MOTOR.getTachoCount() < LIGHT_ANGLE;
  }
  
  /**
   * Moves the robot forwards (straight) a certain distance, using the odometer.
   * 
   * @param dist
   */
  private void moveForward(double dist) {
    FinalDemo.NAV.setSpeeds(MOTOR_SPEED,MOTOR_SPEED);
    double[] start = FinalDemo.NAV.getOdo().getXYT();

    FinalDemo.LEFT_MOTOR.forward();
    FinalDemo.RIGHT_MOTOR.forward();

    while (Navigation.dist(FinalDemo.NAV.getOdo().getXYT(), start) < Math.abs(dist)) {
      sleep(30);
    }
    FinalDemo.NAV.setSpeeds(0, 0);
  }
  
  /**
   * Waits for a certain period of time 
   * @param amt The amount of time to wait in milliseconds
   */
  public void sleep(int amt) {
    try {
      Thread.sleep(amt);
    } catch (InterruptedException e) {
      
    }
  }
}
