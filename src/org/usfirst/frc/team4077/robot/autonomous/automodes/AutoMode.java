package org.usfirst.frc.team4077.robot.autonomous.automodes;

import edu.wpi.first.wpilibj.DriverStation;

public abstract class AutoMode {
  public enum AutoStartPosition { START_LEFT, START_CENTER, START_RIGHT }

  private boolean mIsActive;

  public void init(DriverStation.Alliance alliance, String gameSpecificData,
                   AutoStartPosition autoStartPosition) {
    mIsActive = true;
    System.out.println("Auto mode started");
  }

  public void executeLoop() {}

  public void done() { mIsActive = false; }

  public boolean isActive() { return mIsActive; }

  public String getAutoName() {
    return splitCamelCase(this.getClass().getName());
  }

  static String splitCamelCase(String s) {
    return s.replaceAll(String.format("%s|%s|%s", "(?<=[A-Z])(?=[A-Z][a-z])",
                                      "(?<=[^A-Z])(?=[A-Z])",
                                      "(?<=[A-Za-z])(?=[^A-Za-z])"),
                        " ");
  }
}
