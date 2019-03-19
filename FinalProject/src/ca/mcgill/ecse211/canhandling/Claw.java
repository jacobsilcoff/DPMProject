package ca.mcgill.ecse211.canhandling;

import ca.mcgill.ecse211.demo.BetaDemo;
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
		open = false;
	}

	/**
	 * Closes the claw
	 */
	@SuppressWarnings("deprecation")
	public void close() {
		if(open) {
			BetaDemo.CLAW_MOTOR.setSpeed(75);
			BetaDemo.CLAW_MOTOR.forward();
			try {
				Thread.sleep(4500);
			} catch (InterruptedException ie) {
				ie.printStackTrace();
			}
			BetaDemo.CLAW_MOTOR.stop();
			BetaDemo.CLAW_MOTOR.lock(100);
			open = false;
		}
	}

	/**
	 * Opens the claw
	 */
	public void open() {
		if(!open) {
			BetaDemo.CLAW_MOTOR.setSpeed(75);
			BetaDemo.CLAW_MOTOR.backward();
			try {
				Thread.sleep(4500);
			} catch (InterruptedException ie) {
				ie.printStackTrace();
			}
			BetaDemo.CLAW_MOTOR.stop();
			BetaDemo.CLAW_MOTOR.flt();
			open = true;
		}    
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
