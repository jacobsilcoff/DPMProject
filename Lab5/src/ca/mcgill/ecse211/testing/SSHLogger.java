package ca.mcgill.ecse211.testing;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

public class SSHLogger {
  
  private PrintWriter writer;
  
  public SSHLogger() throws FileNotFoundException, UnsupportedEncodingException {
    writer = new PrintWriter("data.csv", "UTF-8");
  }
  
  public void print(String s) {
    writer.write(s);
  }
  
  public void println(String s) {
    writer.write(s + "\n");
  }
  
  public void close() {
    writer.close();
  }
  
}
