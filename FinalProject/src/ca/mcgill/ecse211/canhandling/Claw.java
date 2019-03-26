package ca.mcgill.ecse211.canhandling;

import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.navigation.Navigation;
import lejos.hardware.lcd.LCD;

public class Claw {

  /**
   * The can classifier used by the Claw
   */
  public static final ColorClassifier CLASSIFIER = new ColorClassifier();
  public static final int CLOSED_ANGLE = 180;
  private boolean open;
  private static boolean calibrated = false;

  /**
   * Creates a claw
   */
  public Claw() {
    open = false;
    if (!calibrated) {
      calibrate();
    }
  }

  private void calibrate() {
    FinalDemo.CLAW_MOTOR.setSpeed(150);
    FinalDemo.CLAW_MOTOR.backward();
    try {
      Thread.sleep(2000);
    } catch (InterruptedException ie) {
      ie.printStackTrace();
    }
    FinalDemo.CLAW_MOTOR.stop();
    FinalDemo.CLAW_MOTOR.flt();
    FinalDemo.CLAW_MOTOR.resetTachoCount();
    calibrated = true;
  }

  /**
   * Closes the claw
   */
  public void close() {
    FinalDemo.CLAW_MOTOR.setSpeed(150);
    FinalDemo.CLAW_MOTOR.rotateTo(CLOSED_ANGLE, false);
    FinalDemo.CLAW_MOTOR.stop();
    open = false;

  }

  /**
   * Opens the claw
   */
  public void open() {
    FinalDemo.CLAW_MOTOR.setSpeed(150);
    FinalDemo.CLAW_MOTOR.rotateTo(3);
    open = true;
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
  public boolean isHeavy(Navigation nav) {
    open();
    /*
     * TODO: Move robot forward a bit, check for button press
     */
    //Move backwards 3cm and turn 180 degrees
    nav.travelTo(nav.getOdo().getXYT()[0],nav.getOdo().getXYT()[1]-3);
    nav.turnTo(180);
    try {
      Thread.sleep(500);
    } catch (InterruptedException ie) {
      ie.printStackTrace();
    }
    close();
    return false;
  }
}
