package ca.mcgill.ecse211.wifi;

import java.awt.geom.Point2D;
import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.canhandling.CanColor;
import ca.mcgill.ecse211.demo.FinalDemo;
import ca.mcgill.ecse211.navigation.Navigation;
import lejos.hardware.lcd.LCD;

/**
 * Gets and holds values from the server to set up a game.
 * 
 * Also performs pre-processing on server data to find way-points 
 * on the map that can be used for localization, search, and 
 * tunnel crossing.
 * @author jacob
 */
public abstract class GameSettings {

  /**
   * The IP address of the server
   */
  private static final String SERVER_IP = "192.168.2.4";
  /**
   * Our robot's team number (6)
   */
  private static final int TEAM_NUMBER = 6;
  
  /**
   * Stores whether or not data has been transmitted from the server
   */
  public static boolean initialized = false;
  /**
   * Stores whether or not we are on the red team (or, 
   * in the inverse case, the green team)
   */
  public static boolean redTeam = false;
  /**
   * Stores the corner of the robot
   */
  public static int corner = -1;
  /**
   * A rectangle representing our robot's start zone
   */
  public static Rect startZone = null;
  /**
   * A rectangle representing the island
   */
  public static Rect island = null;
  /**
   * A rectangle representing the search zone for 
   * our robot.
   */
  public static Rect searchZone = null;
  /**
   * A rectangle representing the tunnel for our robot
   */
  public static Rect tunnel = null;
  
  /**
   * Represents a point of form {x,y} in the 
   * start zone that the robot can safely navigate to
   * before moving straight to tunnelExit in order to
   * cross the tunnel.
   */
  public static double[] tunnelEntrance;
  /**
   * Represents a point of form {x,y} in the 
   * island that the robot can safely navigate to
   * before moving straight to tunnelEntrance in order to
   * cross the tunnel.
   */
  public static double[] tunnelExit;
  /**
   * This is a safe point of form {x,y} to localize on 
   * the start zone using light localization. It is optimized 
   * to be as close to the tunnel entrance as possible
   */
  public static double[] safeLocStart;
  /**
   * This is a safe point of form {x,y} to localize on 
   * the island using light localization. It is optimized 
   * to be as close to the tunnel exit as possible
   */
  public static double[] safeLocIsland;
  /**
   * This is a point of form {x,y} from which the robot can
   * scan the search zone for cans.
   */
  public static double[] startSearch;
  /**
   * This is the range of angles the robot must turn between
   * to scan the search zone for cans in the form {start angle,
   * stop angle}
   */
  public static double[] searchAngles;

  /**
   * Communicates with the server to get access to game information
   * Initializes public static fields, so that once this method has 
   * been called, all information about the game map can be accessed
   * using the static fields of this class.
   */
  public static void init() {

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, false);

    // Connect to server and get the data, catching any errors that might occur
    try {
      Map data = conn.getData();
      LCD.clear();

      //Get team assignment

      long rTeam = ((Long) data.get("RedTeam")).intValue();
      long greenTeam = ((Long) data.get("GreenTeam")).intValue();
      //targetColor = CanColor.fromNumber((int) greenTeam);
      if (rTeam == TEAM_NUMBER) {
        redTeam = true;
      } else if (greenTeam != TEAM_NUMBER) {
        return;
      }

      String color = (redTeam? "Red" : "Green");
      char colorAbrv = (redTeam? 'R' : 'G');

      corner = ((Long) data.get(color + "Corner")).intValue();
      System.out.println("Corner: " + corner);

      startZone = new Rect(color, data);
      island = new Rect("Island", data);
      tunnel = new Rect("TN" + colorAbrv , data);
      searchZone = new Rect("SZ" + colorAbrv, data);

      double[][] entranceAndExit = tunnelEntranceAndExit();
      tunnelEntrance = entranceAndExit[0];
      tunnelExit = entranceAndExit[1];
      safeLocStart = safeLightLocalizationPointStart();
      safeLocIsland = safeLightLocalizationPointIsland();
      setSearchParams();
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
      initialized = false;
    }
    initialized = true;

  }
  
  /**
   * Computes the starting corner point as a 
   * Point2D object.
   * @return a Point2D representing the real coordinate position of the start corner
   */
  public static Point2D getStartingCornerPoint() {
    if (GameSettings.initialized) {
      switch (GameSettings.corner) {
        case 1:
          return new Point2D.Double(14*FinalDemo.GRID_WIDTH,
              FinalDemo.GRID_WIDTH);
        case 2:
          return new Point2D.Double(14*FinalDemo.GRID_WIDTH,
              8*FinalDemo.GRID_WIDTH);
        case 3:
          return new Point2D.Double(FinalDemo.GRID_WIDTH,
              8*FinalDemo.GRID_WIDTH);
        default:
          return new Point2D.Double(FinalDemo.GRID_WIDTH, FinalDemo.GRID_WIDTH);
      }
    } 
    return new Point2D.Double(FinalDemo.GRID_WIDTH, FinalDemo.GRID_WIDTH);
  }

  /**
   * Finds a safe point for light localization
   * outside the tunnel.
   * @return A point of the form {x1,y1}
   */
  private static double[] safeLightLocalizationPointStart() {
    double g = FinalDemo.GRID_WIDTH;
    double[] bestPoint = {g,g};
    double bestDist = Navigation.dist(bestPoint, tunnelEntrance);
    for (int x = startZone.LLx; x < startZone.URx; x++) {
      for (int y = startZone.LLy; y < startZone.URy; y++) {
        if (startZone.contains((x-.5)*g, (y-.5)*g) &&
            startZone.contains((x+.5)*g, (y-.5)*g) &&
            startZone.contains((x-.5)*g, (y+.5)*g) &&
            startZone.contains((x+.5)*g, (y+.5)*g) &&
            !tunnel.contains((x-.5)*g, (y-.5)*g) &&
            !tunnel.contains((x+.5)*g, (y-.5)*g) &&
            !tunnel.contains((x-.5)*g, (y+.5)*g) &&
            !tunnel.contains((x+.5)*g, (y+.5)*g)) {
          double dist = Navigation.dist(tunnelEntrance, new double[] {x*g,y*g});
          if (dist < bestDist) {
            bestDist = dist;
            bestPoint = new double[] {x*g,y*g};
          }
        }
      }
    }
    return bestPoint;
  }

  /**
   * Finds a safe point for light localization
   * before going back through the tunnel.
   * @return A point of the form {x1,y1}
   */
  private static double[] safeLightLocalizationPointIsland() {
    double g = FinalDemo.GRID_WIDTH;
    double[] bestPoint = {g,g};
    double bestDist = Navigation.dist(bestPoint, tunnelExit);
    for (int x = island.LLx; x < island.URx; x++) {
      for (int y = island.LLy; y < island.URy; y++) {
        if (island.contains((x-.5)*g, (y-.5)*g) &&
            island.contains((x+.5)*g, (y-.5)*g) &&
            island.contains((x-.5)*g, (y+.5)*g) &&
            island.contains((x+.5)*g, (y+.5)*g) &&
            !tunnel.contains((x-.5)*g, (y-.5)*g) &&
            !tunnel.contains((x+.5)*g, (y-.5)*g) &&
            !tunnel.contains((x-.5)*g, (y+.5)*g) &&
            !tunnel.contains((x+.5)*g, (y+.5)*g)) {
          double dist = Navigation.dist(tunnelExit, new double[] {x*g,y*g});
          if (dist < bestDist) {
            bestDist = dist;
            bestPoint = new double[] {x*g,y*g};
          }
        }
      }
    }
    return bestPoint;
  }

  /**
   * Returns an array of the form 
   * {{x1,y1},{x2,y2}}
   * where (x1,y1) is the start side of the tunnel
   * and x2,y2 is the island side of the tunnel
   * @return the entrance & exit to the tunnel
   */
  private static double[][] tunnelEntranceAndExit() {
    Rect tunnel = GameSettings.tunnel;
    Rect startZone = GameSettings.startZone;
    Rect island = GameSettings.island;
    double[] llBlock = {(tunnel.LLx + .5) * FinalDemo.GRID_WIDTH, 
        (tunnel.LLy + .5) * FinalDemo.GRID_WIDTH};
    double[] urBlock = {(tunnel.URx - .5) * FinalDemo.GRID_WIDTH, 
        (tunnel.URy - .5) * FinalDemo.GRID_WIDTH};
    double[] N = translate(urBlock, 0, FinalDemo.GRID_WIDTH);
    double[] S = translate(llBlock, 0, -FinalDemo.GRID_WIDTH);
    double[] E = translate(urBlock,FinalDemo.GRID_WIDTH, 0);
    double[] W = translate(llBlock, -FinalDemo.GRID_WIDTH, 0);
    //strictly one of N, S, E, W is contained in start
    double[] entrance = N, exit = S;
    if (startZone.contains(N) && island.contains(S)) {
      entrance = N; exit = S;
    } else if (startZone.contains(S) && island.contains(N)) {
      entrance = S; exit = N;
    } else if (startZone.contains(E) && island.contains(W)) {
      entrance = E; 
      exit = W;
    } else if (startZone.contains(W) && island.contains(E)){
      entrance = W;
      exit = E;
    }
    return new double[][] {entrance,exit};
  }

  /**
   * Modifies a point by adding x and y
   * @param pt the original point
   * @param x the translation x amt
   * @param y the translation yamt
   * @return the translated point
   */
  private static double[] translate(double[] pt, double x, double y) {
    return new double[] {pt[0] + x, pt[1] + y};
  }

  /**
   * Sets up the data needed to safely search w/o hitting cans
   */
  private static void setSearchParams() {
    double[][] opts = {{searchZone.LLx, searchZone.LLy},
        {searchZone.URx, searchZone.LLy},
        {searchZone.URx, searchZone.URy},
        {searchZone.LLx, searchZone.URy}};
    double[][] angles = {{0, 90},
        {270, 360},
        {180, 270},
        {90, 180}};
    startSearch = opts[0];
    int bestInd = 0;
    double bestDist = Double.MAX_VALUE;
    double g = FinalDemo.GRID_WIDTH;
    for (int i = 0; i < 4; i++) {
      double x = opts[i][0];
      double y = opts[i][1];
      double d = Point2D.distance(x*g, y*g, tunnelExit[0], tunnelExit[1]);
      if (island.contains((x-.5)*g, (y-.5)*g) &&
          island.contains((x+.5)*g, (y-.5)*g) &&
          island.contains((x-.5)*g, (y+.5)*g) &&
          island.contains((x+.5)*g, (y+.5)*g) &&
          !tunnel.contains((x-.5)*g, (y-.5)*g) &&
          !tunnel.contains((x+.5)*g, (y-.5)*g) &&
          !tunnel.contains((x-.5)*g, (y+.5)*g) &&
          !tunnel.contains((x+.5)*g, (y+.5)*g) &&
          (d < bestDist) && x > 0 && x < 15
          && x > 0 && x < 9) {
        bestInd = i;
        bestDist = d;
      }
    }
    if (bestDist == Double.MAX_VALUE) {
      opts = new double[][]{{searchZone.LLx + 1, searchZone.LLy + 1},
        {searchZone.URx - 1, searchZone.LLy + 1},
        {searchZone.URx - 1 , searchZone.URy - 1},
        {searchZone.LLx + 1, searchZone.URy - 1}};
        for (int i = 0; i < 4; i++) {
          double d = Point2D.distance(opts[i][0]*g, opts[i][1]*g, 
              tunnelExit[0], tunnelExit[1]);
          if (d < bestDist) {
            bestInd = i;
            bestDist = d;
          }
        }
    }
    startSearch = opts[bestInd];
    searchAngles = angles[bestInd];
  }
}
