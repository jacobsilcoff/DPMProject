package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.color.ColorClassifier;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {
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
  public static final SampleProvider US_RIGHT;
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

  static {
    @SuppressWarnings("resource")
    SensorModes lightSensorMode = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    COLOR_SENSOR = lightSensorMode.getMode("RGB");

    @SuppressWarnings("resource")
    SensorModes lightSensorMode2 = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
    LINE_SENSOR = lightSensorMode2.getMode("Red");

    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));
    US_FRONT = usSensor.getMode("Distance");

    @SuppressWarnings("resource")
    SensorModes usSensor2 = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
    US_RIGHT = usSensor2.getMode("Distance");

  }
  /**
   * The LCD used to output during the robot's journey
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();

  public static void main(String[] args) {
    ColorClassifier cc = new ColorClassifier();
    cc.start();
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
