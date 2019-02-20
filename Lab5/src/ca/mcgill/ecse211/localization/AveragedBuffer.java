package ca.mcgill.ecse211.localization;


/**
 * An AveragedBuffer stores a buffer of data with constant time lookup for the average value. This
 * can be used to implement a rolling average filter.
 * 
 * @author jacob
 */
public class AveragedBuffer {
  /**
   * This value stores the default number of samples that are stored in the buffer
   */
  private static final int DEFAULT_N = 10;
  private int n;
  private float[] samples;
  private int sampleIndex;
  private float avg;

  /**
   * Creates a buffer that will store a default number of samples, as specified by DEFAULT_N
   */
  public AveragedBuffer() {
    n = DEFAULT_N;
    samples = new float[n];
    sampleIndex = 0;
    avg = 0;
  }

  /**
   * Creates a buffer that will store a specified number of samples,
   * 
   * @param n The number of samples stored in the buffer
   */
  public AveragedBuffer(int n) {
    samples = new float[n];
    this.n = n;
    sampleIndex = 0;
    avg = 0;
  }

  /**
   * Adds a measurement to the buffer and updates the average
   * 
   * @param x The data sample to add to the buffer
   */
  public void add(float x) {
    avg = avg + 1f / n * (x - samples[sampleIndex]);
    samples[sampleIndex] = x;
    sampleIndex = (sampleIndex + 1) % n;
  }

  /**
   * Returns the average of the buffer
   * 
   * @return The average of the buffer
   */
  public float getAvg() {
    return avg;
  }
  
  /**
   * Gets the size of the buffer
   * @return the size of the buffer
   */
  public int getN() {
    return n;
  }
}