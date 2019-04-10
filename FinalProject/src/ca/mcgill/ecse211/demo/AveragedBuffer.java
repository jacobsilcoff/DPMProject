package ca.mcgill.ecse211.demo;


/**
 * An AveragedBuffer stores a buffer of data with constant time lookup 
 * for the average value. This can be used to implement a rolling average 
 * filter.
 * 
 * @author jacob
 */
public class AveragedBuffer<T extends Number> {
  /**
   * This value stores the default number of samples that are stored in the buffer
   */
  private static final int DEFAULT_N = 10;
  private int n;
  private T[] samples;
  private int sampleIndex;
  private double avg;
  private int size;

  /**
   * Creates a buffer that will store a default number of samples, as specified by DEFAULT_N
   */
  public AveragedBuffer() {
    n = DEFAULT_N;
    samples = (T[]) new Number[n];
    sampleIndex = 0;
    avg = 0;
    size = 0;
  }

  /**
   * Creates a buffer that will store a specified number of samples
   * 
   * @param n The number of samples stored in the buffer
   */
  public AveragedBuffer(int n) {
    samples = (T[]) new Number[n];
    this.n = n;
    sampleIndex = 0;
    avg = 0;
  }

  /**
   * Adds a measurement to the buffer and updates the average
   * 
   * @param x The data sample to add to the buffer
   */
  public void add(T x) {
    if (size < n) {
      avg = (avg * size + x.doubleValue())/(size + 1);
      size++;
    } else {
      avg = avg + 1.0 / n * (x.doubleValue() - samples[sampleIndex].doubleValue());
    }
    samples[sampleIndex] = x;
    sampleIndex = (sampleIndex + 1) % n;
  }

  /**
   * Returns the average of the buffer
   * 
   * @return The average of the buffer
   */
  public double getAvg() {
    return avg;
  }
  
  /**
   * Gets the size of the buffer
   * @return the size of the buffer
   */
  public int getN() {
    return n;
  }
  
  /**
   * Resets buffer
   */
  public void clear() {
    samples = (T[]) new Number[n];
    sampleIndex = 0;
    avg = 0;
    size = 0;
  }
}