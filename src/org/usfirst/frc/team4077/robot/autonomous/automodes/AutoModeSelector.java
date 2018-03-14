package org.usfirst.frc.team4077.robot.autonomous.automodes;

import org.usfirst.frc.team4077.robot.components.Drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class AutoModeSelector {
  private DriverStation.Alliance mAlliance;
  private AutoMode.AutoStartPosition mAutoStartPosition;
  private Drive mDrive;
  private long mDelayMillis;

  private AutoMode mAutoModes[] = new AutoMode[] {};

  private Preferences mPrefs;

  private EncoderBasePassAutoLine mPassAutoLine;

  public AutoModeSelector(DriverStation.Alliance alliance, Drive drive) {
    mAlliance = alliance;
    mDrive = drive;
    mPrefs = Preferences.getInstance();
  }

  public void initAutoFromSmartDashboard() {
    int autoStartPos = mPrefs.getInt("AutoStartPos", 0);
    mDelayMillis = mPrefs.getLong("AutoDelayMilliseconds", 0);
    String autoMode = mPrefs.getString("AutoMode", "");

    switch (autoStartPos) {
    case 0:
      mAutoStartPosition = AutoMode.AutoStartPosition.START_LEFT;
      break;
    case 1:
      mAutoStartPosition = AutoMode.AutoStartPosition.START_CENTER;
      break;
    case 2:
      mAutoStartPosition = AutoMode.AutoStartPosition.START_RIGHT;
      break;
    }

    switch (autoMode) {
    case "PassAutoLine":
      break;
    case "Test":
      break;
    default:
      break;
    }
  }
}
