package ca.mcgill.ecse211.canhandling;

import ca.mcgill.ecse211.demo.BetaDemo;
import ca.mcgill.ecse211.navigation.Navigation;
import lejos.hardware.lcd.LCD;

public class Claw {

	/**
	 * The can classifier used by the Claw
	 */
	public static final ColorClassifier CLASSIFIER = new ColorClassifier();
	public static final int CLOSED_ANGLE = 100;
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
	  BetaDemo.CLAW_MOTOR.setSpeed(150);
      BetaDemo.CLAW_MOTOR.backward();
      try {
          Thread.sleep(2000);
      } catch (InterruptedException ie) {
          ie.printStackTrace();
      }
      BetaDemo.CLAW_MOTOR.stop();
      BetaDemo.CLAW_MOTOR.flt();
      BetaDemo.CLAW_MOTOR.resetTachoCount();
      calibrated = true;
	}

	/**
	 * Closes the claw
	 */
	public void close() {
		if(open) {
			BetaDemo.CLAW_MOTOR.setSpeed(150);
			BetaDemo.CLAW_MOTOR.forward();
			try {
				Thread.sleep(2200);
			} catch (InterruptedException ie) {
				ie.printStackTrace();
			}
			BetaDemo.CLAW_MOTOR.stop();
			open = false;
			LCD.drawString("<"+BetaDemo.CLAW_MOTOR.getTachoCount(), 0, 5);
		}
	}

	/**
	 * Opens the claw
	 */
	public void open() {
		BetaDemo.CLAW_MOTOR.rotateTo(3);
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
