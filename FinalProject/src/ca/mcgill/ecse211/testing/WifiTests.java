package ca.mcgill.ecse211.testing;

import ca.mcgill.ecse211.wifi.GameSettings;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

public class WifiTests {
  public static void main(String[] args) {
    GameSettings.init();
    LCD.clear();
    LCD.drawString(GameSettings.redTeam + "", 0, 7);
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
      GameSettings.init();
      LCD.clear();
      LCD.drawString(GameSettings.redTeam + "", 0, 7);
    }
  }
}
