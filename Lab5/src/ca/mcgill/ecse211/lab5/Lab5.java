package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.color.CanColor;
import ca.mcgill.ecse211.color.ColorClassifier;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {
  /**
   * Demo variables:
   */
  public static final int LLx = 1;
  public static final int LLy = 2;
  public static final int URx = 4;
  public static final int URy = 4;
  public static final int TR = 0;
  
  /**
   * The robot's left motor
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  /**
   * The robot's right motor
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  /**
   * The light sensor motor
   */
  public static final EV3LargeRegulatedMotor SENSOR_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  /**
   * The robot's color-detecting light sensor
   */
  public static final SampleProvider COLOR_SENSOR;
  /**
   * The robot's line-detecting light sensor
   */
  public static final SampleProvider LINE_SENSOR;

  /**
   * The robot's front-facing ultrasonic sensor
   */
  public static final SampleProvider US_FRONT;
  /**
   * The robot's right-facing ultrasonic sensor
   */
  //public static final SampleProvider US_RIGHT;
  /**
   * Represents the radius of each wheel, in cm
   */
  public static final double WHEEL_RAD = 2.18;
  /**
   * Represents half the distance between the wheels, in cm Will need updating
   */
  public static final double TRACK = 13.6;
  /**
   * The offset between the robot turning center and the line sensor in
   * the Y direction, in cm. Note: magnitude only.
   */
  public static final double LINE_OFFSET_Y = 10.5;
  /**
   * The offset between the robot turning center and the line sensor in
   * the X direction, in cm. Note: magnitude only.
   */
  public static final double LINE_OFFSET_X = TRACK/2;
  /**
   * The offset between the robot turning center and the
   * center of where the can should be for measurment
   */
  public static final double CAN_DIST = 7;
  
  /**
   * The can classifier used by the program
   */
  public static final ColorClassifier CLASSIFIER = new ColorClassifier();

  static {
    @SuppressWarnings("resource")
    SensorModes lightSensorMode = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    COLOR_SENSOR = lightSensorMode.getMode("RGB");

    @SuppressWarnings("resource")
    SensorModes lightSensorMode2 = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
    LINE_SENSOR = lightSensorMode2.getMode("Red");

    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
    US_FRONT = usSensor.getMode("Distance");

    //@SuppressWarnings("resource")
    //SensorModes usSensor2 = new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));
    //US_RIGHT = usSensor2.getMode("Distance");

  }
  /**
   * The LCD used to output during the robot's journey
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();

  public static void main(String[] args) throws OdometerExceptions, InterruptedException {
    (new Thread(Odometer.getOdometer())).start();
    OdometryCorrection oc = new OdometryCorrection();
    Navigation nav = new Navigation(oc);
    nav.start();
    //Start localization
    Button.waitForAnyPress();
    CLASSIFIER.calibrate(false);
    //Localizes robot
    UltrasonicLocalizer ul = new UltrasonicLocalizer(oc);
    LightLocalizer ll = new LightLocalizer(oc, 0,0);
    ul.run();  
    ll.run();
    nav.travelTo(OdometryCorrection.LINE_SPACING, OdometryCorrection.LINE_SPACING);
    while (nav.isNavigating()) Thread.sleep(100);
    nav.turnTo(0);
    //move to ll
    Button.waitForAnyPress();
    
    //Start the correction
    (new Thread(oc)).start();
    
    nav.travelTo(LLx * OdometryCorrection.LINE_SPACING, LLy * OdometryCorrection.LINE_SPACING);
    Sound.beep();
    
    //Start can finder
    Button.waitForAnyPress();
    
    CanFinder finder = new CanFinder(nav, CanColor.fromNumber(TR));
    finder.run();
    
    nav.travelTo(URx * OdometryCorrection.LINE_SPACING, URy * OdometryCorrection.LINE_SPACING);
    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
    
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
          - Lab5.LINE_OFFSET_X * Math.cos(Math.toRadians(t))
          + Lab5.LINE_OFFSET_Y * Math.sin(Math.toRadians(t));
      result[1] = sensor[1] 
          + Lab5.LINE_OFFSET_X * Math.sin(Math.toRadians(t))
          + Lab5.LINE_OFFSET_Y * Math.cos(Math.toRadians(t));
      result[2] = t;
    }
    return result;
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
          + Lab5.LINE_OFFSET_X * Math.cos(Math.toRadians(t))
          - Lab5.LINE_OFFSET_Y * Math.sin(Math.toRadians(t));
      result[1] = robot[1] 
          - Lab5.LINE_OFFSET_X * Math.sin(Math.toRadians(t))
          - Lab5.LINE_OFFSET_Y * Math.cos(Math.toRadians(t));
      result[2] = t;
    }
    return result;
  }
}
