package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab5.Lab5;

/**
 * Represents a thread that uses a light sensor to update the values of a robots odometer to reduce
 * error.
 * 
 * @author Group 71
 */
public class OdometryCorrection implements Runnable {

  /**
   * This represents the minimum difference from the mean for a light sensor reading to be
   * considered significant
   */
  private static final float LIGHT_THRESHOLD = 0.18f;
  /**
   * This represents the distance between lines on the grid, in cm
   */
  private static final float LINE_SPACING = 30.48f;
  /**
   * This represents the minimum distance for the robot to travel before reading another line
   */
  private static final float DIST_THRESHOLD = 3;
  /**
   * The time period between polling of the sensor, in ms
   */
  private static final long CORRECTION_PERIOD = 7;

  private Odometer odometer;
  private float[] sample;
  private CircularArray samples;
  private double[] lastPos;
  private boolean on;


  /**
   * This is the default class constructor. An existing instance of the odometer is used to ensure
   * thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    sample = new float[Lab5.LINE_SENSOR.sampleSize()];
    samples = new CircularArray();
    on = true;
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


    while (true) {
      correctionStart = System.currentTimeMillis();
      Lab5.LINE_SENSOR.fetchSample(sample, 0);
      double[] pos = odometer.getXYT(); // current odo-position

      /*
       * To avoid a single line triggering this many times, verify that either we haven't seen a
       * line yet at all (lastPos == null) or we're sufficiently far from the last line.
       */
      if (on && sample[0] < samples.avg - LIGHT_THRESHOLD
          && (lastPos == null || dist(pos, lastPos) > DIST_THRESHOLD)) {
        // Indicate detection of a line
        lineCount++;
        Lab5.LCD.drawString(lineCount + " line(s) detected.", 0, 8);

        /*
         * Concept is to figure out which x/y is closest to desired target, and round that one. We
         * ignore the first line because it is the sensor moving about starting point
         */
        if (lineCount != 1) {
          if (pos[0] % LINE_SPACING < pos[1] % LINE_SPACING) {
            // here we round the x position
            odometer.setX(Math.round(pos[0] / LINE_SPACING) * LINE_SPACING);
          } else {
            // here we round the y position
            odometer.setY(Math.round(pos[1] / LINE_SPACING) * LINE_SPACING);
          }

          // update last pos of line detected
          lastPos = odometer.getXYT();
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
          // there is nothing to be done here
        }
      }

    }// end loop
  }// end run method

  /**
   * Sets the correction to be either on or off
   * 
   * @param value True turns the correction on, false is off
   */
  public void setOn(boolean value) {
    on = value;
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

  /**
   * Stores a circular array of light values. Buffered to keep a rolling average.
   * 
   * We assume N is small enough that populating the buffer with data takes a trivial amount of
   * time, although making the average work for less than N samples would be trivial with a
   * conditional and a for loop.
   */
  private class CircularArray {
    private static final int N = 5;
    private float[] samples;
    private int sampleIndex;
    private float avg;

    /**
     * Creates a circular array of a constant length N
     */
    public CircularArray() {
      samples = new float[N];
      sampleIndex = 0;
      avg = 0;
    }

    /**
     * Adds a measurement to the buffer and updates the average
     * 
     * @param x The data sample to add to the buffer
     */
    public void add(float x) {
      avg = avg + 1f / N * (x - samples[sampleIndex]);
      samples[sampleIndex] = x;
      sampleIndex = (sampleIndex + 1) % N;
    }
  }

}