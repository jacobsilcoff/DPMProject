package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.demo.AveragedBuffer;
import ca.mcgill.ecse211.demo.FinalDemo;
import lejos.hardware.Sound;

/**
 * Represents a thread that uses a light sensor to update the values of a robots odometer to reduce
 * error.
 * 
 * @author Group 6
 */
public class OdometryCorrection extends Thread {

  /**
   * This represents the minimum difference from the mean for a light sensor reading to be
   * considered significant
   */
  private static final float LIGHT_THRESHOLD = 0.22f;//was .08
  /**
   * This represents the distance between lines on the grid, in cm
   */
  public static final float LINE_SPACING = 30.48f;
  /**
   * This represents the minimum distance for the robot to travel before reading another line
   */
  private static final float DIST_THRESHOLD = 5;
  /**
   * The time period between polling of the sensor, in ms
   */
  private static final long CORRECTION_PERIOD = 7;
  /**
   * The maximum amount that the OC will round
   */
  private static final float ROUND_LIMIT = 10;

  private Odometer odometer;

  private boolean on;

  /**
   * This is the default class constructor. An existing instance of the odometer is used to ensure
   * thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    on = false;
  }

  /**
   * Run method for odometer correction (required for Thread). This continuously looks for the
   * detection of a line, and upon seeing one will round either the x or the y position (whichever
   * is closer) to an integer multiple of the distance between grid lines.
   * 
   * @throws OdometerExceptions
   */
  @Override
  public void run() {
    int lineCount = 0;
    long correctionStart, correctionEnd;
    double[] lastPos = null;
    float[] sample = new float[FinalDemo.LINE_SENSOR.sampleSize()];
    AveragedBuffer<Float> samples = new AveragedBuffer<Float>(100);

    while (true) {
      correctionStart = System.currentTimeMillis();
      FinalDemo.LINE_SENSOR.fetchSample(sample, 0);
      double[] pos = odometer.getXYT(); // current odo-position

      /*
       * To avoid a single line triggering this many times, verify that either we haven't seen a
       * line yet at all (lastPos == null) or we're sufficiently far from the last line.
       */
      if (on && (sample[0] < samples.getAvg() - LIGHT_THRESHOLD
          && (lastPos == null || dist(pos, lastPos) > DIST_THRESHOLD))) {
        // update last pos of line detected
        lastPos = pos;
        lineCount++;


        double[] sensor = FinalDemo.toSensor(pos);
        if (lineCount != 1) {
          double roundedX = Math.round(sensor[0] / LINE_SPACING) * LINE_SPACING;
          double roundedY = Math.round(sensor[1] / LINE_SPACING) * LINE_SPACING;
          if (Math.abs(sensor[0] - roundedX) < Math.abs(sensor[1] - roundedY)) {
            // here we round the x position
            if (Math.abs(sensor[0] - roundedX) < ROUND_LIMIT) {
              sensor[0] = roundedX;
              odometer.setX(FinalDemo.toRobot(sensor)[0]);
              if (FinalDemo.DEBUG_ON) {
                Sound.beepSequenceUp();
              }
            } else {
              //indicates severe error
              if (FinalDemo.DEBUG_ON) {
                Sound.buzz();
              }
            }
          } else {
            // here we round the y position
            if (Math.abs(sensor[1] - roundedY) < ROUND_LIMIT) {
              sensor[1] = roundedY;
              odometer.setY(FinalDemo.toRobot(sensor)[1]);
              if (FinalDemo.DEBUG_ON) {
                Sound.beepSequenceUp();
              }
            } else {
              //indicates severe error
              if (FinalDemo.DEBUG_ON) {
                Sound.buzz();
              }
            }
          }
        }
      }

      // Add the sample to the rolling average
      samples.add(sample[0]);

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
        }
      }

    } // end loop
  }// end run method

  /**
   * Sets the correction to be either on or off
   * 
   * @param value True turns the correction on, false is off
   */
  public void setOn(boolean t) {
    on = t;
  }

  public boolean getOn() {
    return on;
  }


  /**
   * Calculates distance between two positions
   * 
   * @param a The first point, as an array of coordinates
   * @param b The second point, as an array of coordinates
   * @return The distance, in cm, between a and b
   */
  private static double dist(double[] a, double[] b) {
    if (a.length < 2 || b.length < 2) {
      return -1;
    }
    return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2)); // fixed distance formula
  }
}
