package ca.mcgill.ecse211.demo;

import ca.mcgill.ecse211.canhandling.CanColor;
import ca.mcgill.ecse211.canhandling.Claw;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.wifi.GameSettings;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.MirrorMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class BetaDemo {
  /**
   * The robot's left motor
   */
  public static final RegulatedMotor LEFT_MOTOR = 
      MirrorMotor.invertMotor(new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C")));
  /**
   * The robot's right motor
   */
  public static final RegulatedMotor RIGHT_MOTOR =
      MirrorMotor.invertMotor(new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B")));
  /**
   * The claw motor
   */
  public static final EV3LargeRegulatedMotor CLAW_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  
  /**
   * The motor used to spin cans
   */
  public static final EV3LargeRegulatedMotor CAN_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  
  /**
   * The robot's color-detecting light sensor
   */
  public static final SampleProvider COLOR_SENSOR;
  /**
   * The robot's line-detecting light sensor
   */
  public static final SampleProvider LINE_SENSOR;
  /**
   * The robot's touch sensor to detect heavy cans
   */
  public static final SampleProvider TOUCH_SENSOR;
  /**
   * The robot's front-facing ultrasonic sensor
   */
  public static final SampleProvider US_FRONT;
  /**
   * Represents the radius of each wheel, in cm
   */
  public static final double WHEEL_RAD = 2.18;
  /**
   * Represents the distance between the wheels, in cm
   */
  public static final double TRACK = 10.33;
  /**
   * The offset between the robot turning center and the line sensor in
   * the Y direction, in cm. Note: magnitude only.
   */
  public static final double LINE_OFFSET_Y = 7.01;
  /**
   * The offset between the robot turning center and the line sensor in
   * the X direction, in cm. Note: magnitude only.
   */
  public static final double LINE_OFFSET_X = TRACK/2;

  /**
   * The can classifier used by the program
   */
  public static final Claw CLAW = new Claw();

  static {
    @SuppressWarnings("resource")
    SensorModes colorSensorMode = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
    COLOR_SENSOR = colorSensorMode.getMode("RGB");

    @SuppressWarnings("resource")
    SensorModes lineSensorMode = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    LINE_SENSOR = lineSensorMode.getMode("Red");

    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
    US_FRONT = usSensor.getMode("Distance");

    @SuppressWarnings("resource")
    SensorModes touchSensorMode = new EV3TouchSensor(LocalEV3.get().getPort("S4"));
    TOUCH_SENSOR = touchSensorMode.getMode("Touch");
  }
  /**
   * The LCD used to output during the robot's journey
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();

  /**
   * The Odometry correction system for the robot
   */
  public static final OdometryCorrection OC = getOC();

  /**
   * The navigation thread used by the robot
   */
  public static final Navigation NAV = getNav();
  private static Navigation getNav() {
    try {
      return new Navigation(OC);
    } catch (OdometerExceptions e) {
      return null;
    }
  }
  private static OdometryCorrection getOC() {
    try {
      return new OdometryCorrection();
    } catch (OdometerExceptions e) {
      return null;
    }
  }
  
  /**
   * Distance between lines in cm
   */
  public static final float GRID_WIDTH = 30.48f;

  /**
   * Localizes the robot using US and light,
   * then moves to a specified search area and searches for cans
   * @param args not used
   * @throws OdometerExceptions 
   * @throws InterruptedException
   */
  public static void main(String[] args) throws OdometerExceptions, InterruptedException {
    init();
    //localize();
    resetOdo();
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
      CLAW.open();
      CLAW.close();
      LCD.clear();

      LCD.drawString("<: " + CLAW_MOTOR.getTachoCount(), 0, 4);
      Claw.CLASSIFIER.getData();
    }
    System.exit(0);
  }
  
  /**
   * Initializes the robot by getting game settings,
   * starting the odometry thread & the nav thread
   * @throws OdometerExceptions
   */
  private static void init() throws OdometerExceptions {
    //GameSettings.init();
    (new Thread(Odometer.getOdometer())).start();
    NAV.start();
    OC.start();
    OC.setOn(false);
  }
  
  /**
   * Localizes the robot, and starts the correction when
   * done
   * @throws OdometerExceptions
   */
  private static void localize() throws OdometerExceptions {
    OC.setOn(false);
    (new UltrasonicLocalizer()).run();
    OC.setOn(false);
    (new LightLocalizer(0,0)).run();
    OC.setOn(true);
  }

  
  
  /**
   * Calculates the center of the sensor from the position of the
   * robot, denoted as an array
   * @param robot An array of the form {x,y,t} representing the
   * position of the sensor
   * @return
   */
  public static double[] toSensor(double[] robot) {
    double[] result = new double[3];
    if (robot.length == 3) {
      double t = robot[2];
      result[0] = robot[0] 
          - BetaDemo.LINE_OFFSET_X * Math.cos(Math.toRadians(t))
          - BetaDemo.LINE_OFFSET_Y * Math.sin(Math.toRadians(t));
      result[1] = robot[1] 
          + BetaDemo.LINE_OFFSET_X * Math.sin(Math.toRadians(t))
          - BetaDemo.LINE_OFFSET_Y * Math.cos(Math.toRadians(t));
      result[2] = t;
    }
    return result;
  }
  
  /**
   * Calculates the center of the robot from the position of the
   * line sensor, denoted as an array
   * @param sensor An array of the form {x,y,t} representing the
   * position of the sensor
   * @return
   */
  public static double[] toRobot(double[] sensor) {
    double[] result = new double[3];
    if (sensor.length == 3) {
      double t = sensor[2];
      result[0] = sensor[0] 
          + BetaDemo.LINE_OFFSET_X * Math.cos(Math.toRadians(t))
          + BetaDemo.LINE_OFFSET_Y * Math.sin(Math.toRadians(t));
      result[1] = sensor[1] 
          - BetaDemo.LINE_OFFSET_X * Math.sin(Math.toRadians(t))
          + BetaDemo.LINE_OFFSET_Y * Math.cos(Math.toRadians(t));
      result[2] = t;
    }
    return result;
  }
  
  
  
  /*
   * ***********************
   * TESTING METHODS
   * ***********************
   */
  
  /**
   * Drives in a square
   * FOR TESTING
   * @param ocOn Whether or not to use correction
   */
  private static void squareDrive(boolean ocOn) {
    OC.setOn(ocOn);
    NAV.travelTo(1*GRID_WIDTH, 3*GRID_WIDTH);
    NAV.waitUntilDone();
    NAV.travelTo(3*GRID_WIDTH,3*GRID_WIDTH);
    NAV.waitUntilDone();
    NAV.travelTo(3*GRID_WIDTH, 1*GRID_WIDTH);
    NAV.waitUntilDone();
    NAV.travelTo(1*GRID_WIDTH, 1*GRID_WIDTH);
    NAV.waitUntilDone();
    OC.setOn(true);
  }
  
  /**
   * Spins the robot around 360 * x, where
   * x is a number of times
   * FOR TESTING
   * to test the parameters for the wheels
   */
  private static void rotateX(int x) {
    int t = 0;
    for (int i = 0; i < 3 * x + 1; i++) {
      NAV.turnTo(t);
      t = (t + 120) % 360;
    }
  }
  
  /**
   * Sets the odometer to 1,1,0
   * @throws OdometerExceptions 
   */
  private static void resetOdo() throws OdometerExceptions {
    Odometer.getOdometer().setXYT(GRID_WIDTH, GRID_WIDTH, 0);
  }
}
