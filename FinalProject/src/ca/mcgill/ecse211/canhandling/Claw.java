package ca.mcgill.ecse211.canhandling;

import ca.mcgill.ecse211.navigation.Navigation;

public class Claw {
  
  /**
   * The can classifier used by the Claw
   */
  public static final ColorClassifier CLASSIFIER = new ColorClassifier();
  
  private boolean open;
  
  /**
   * Creates a claw
   */
  public Claw() {
    CLASSIFIER.calibrate();
    //TODO: Write code to calibrate claw
    open = false;
  }
  
  /**
   * Closes the claw
   */
  public void close() {
    open = false;
  }
  
  /**
   * Opens the claw
   */
  public void open() {
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
    close();
    return false;
  }
}
