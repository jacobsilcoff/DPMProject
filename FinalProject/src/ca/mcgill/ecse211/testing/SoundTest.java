package ca.mcgill.ecse211.testing;

import lejos.hardware.Button;
import lejos.hardware.Sound;

public class SoundTest {
  public static void main(String[] args) throws InterruptedException {
    for (int i = 0; i < 5; i++) {
      Sound.beep();
      Thread.sleep(100);
    }
  }
}
