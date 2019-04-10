package ca.mcgill.ecse211.canhandling;

import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.navigation.Navigation;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.UnregulatedMotor;

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
  public static final int CLOSED_ANGLE = 170;
  public static final int CLAW_POWER = 25;
  public static final int WEIGHT_TIME = 3000;
  private static boolean calibrated = false;

  /*
   * Weighing variables:
   */
  public static final int MOTOR_SPEED = 100;
  public static final int BACK_DISTANCE = 3;
  public static final int THRESH_POWER = 15;
  public static final int LIGHT_ANGLE = 165;
  
  /**
   * Creates a claw
   */
  public Claw() {
    if (!calibrated) {
      calibrate();
    }
  }

  private void calibrate() {
    CLAW_MOTOR.setPower(CLAW_POWER);
    CLAW_MOTOR.backward();
    sleep(2000);
    CLAW_MOTOR.setPower(0);
    CLAW_MOTOR.resetTachoCount();
    calibrated = true;
  }

  /**
   * Assuming a can is being currently held,
   * classifies the can and beeps accordingly
   */
  public void classifyAndBeep() {
    CanColor c = getColor();
    int time = isHeavy() ? 1000 : 500;
    close();
    int numBeeps;
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
  }
  /**
   * Closes the claw
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
      else if (i*30 > 5000) {
        open();
        CLAW_MOTOR.setPower(CLAW_POWER);
        CLAW_MOTOR.forward();
        i = 0;
      }
    }
    CLAW_MOTOR.setPower(CLAW_POWER);
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

  public CanColor getColor() {
    return CLASSIFIER.classify();
  }

  /**
   * Checks if the can in the claw is heavy or not
   * by trying to push it
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
  
  public void sleep(int amt) {
    try {
      Thread.sleep(amt);
    } catch (InterruptedException e) {
      
    }
  }
}
