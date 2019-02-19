package ca.mcgill.ecse211;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {
  /**
   * The robot's light sensor
   */
  public static final SampleProvider LIGHT_SENSOR;
  static {
    Port lightSensorPort = LocalEV3.get().getPort("S3");

    @SuppressWarnings("resource")
    SensorModes lightSensorMode = new EV3ColorSensor(lightSensorPort);
    LIGHT_SENSOR = lightSensorMode.getMode("RGB");
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