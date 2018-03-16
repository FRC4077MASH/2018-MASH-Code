package org.usfirst.frc.team4077.robot.autonomous.automodes;

import org.usfirst.frc.team4077.robot.components.Drive;
import org.usfirst.frc.team4077.robot.components.Lift;
import org.usfirst.frc.team4077.robot.components.Manipulator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class AutoModeSelector {
  public enum AutoStartPosition { START_LEFT, START_CENTER, START_RIGHT }

  private AutoStartPosition mAutoStartPosition;
  private Drive mDrive;
  private Lift mLift;
  private Manipulator mManipulator;
  private long mDelayMillis;
  private boolean mRunAuto;
  private boolean mDoSwitch;
  private boolean mDoScale;
  private static long mAutoStartTime;

  private Preferences mPrefs;

  /* Auto Mode Objects Here */
  private AutoDoSwitch mAutoDoSwitch;
  /* Auto Mode Objects Here */

  public AutoModeSelector(Drive drive, Lift lift, Manipulator manipulator) {
    mDrive = drive;
    mLift = lift;
    mManipulator = manipulator;
    mPrefs = Preferences.getInstance();

    /* Auto Mode Objects Here */
    mAutoDoSwitch = new AutoDoSwitch(mDrive, mLift, mManipulator);
    /* Auto Mode Objects Here */

    mPrefs.putInt("AutoStartPos", 0);
    mPrefs.putLong("AutoDelayMilliseconds", 0);
    mPrefs.putBoolean("AutoRunAuto", true);
    mPrefs.putBoolean("AutoDoSwitch", false);
    mPrefs.putBoolean("AutoDoScale", false);
  }

  public void initAutoFromSmartDashboard() {
    int autoStartPos = mPrefs.getInt("AutoStartPos", 0);
    mDelayMillis = mPrefs.getLong("AutoDelayMilliseconds", 0);
    mRunAuto = mPrefs.getBoolean("AutoRunAuto", true);
    mDoSwitch = mPrefs.getBoolean("AutoDoSwitch", false);
    mDoScale = mPrefs.getBoolean("AutoDoScale", false);

    switch (autoStartPos) {
    case 0:
      mAutoStartPosition = AutoStartPosition.START_LEFT;
      break;
    case 1:
      mAutoStartPosition = AutoStartPosition.START_CENTER;
      break;
    case 2:
      mAutoStartPosition = AutoStartPosition.START_RIGHT;
      break;
    }

    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    System.out.println("Game specific data:" + gameData);

    if (mDoSwitch && mDoScale) {
      // Do both
      System.out.println("Running auto: Both");
    } else if (mDoSwitch && !mDoScale) {
      // Do Switch
      System.out.println("Running auto: Switch");
      mAutoDoSwitch.init(DriverStation.getInstance().getAlliance(), gameData,
                         mAutoStartPosition, mDelayMillis);
    } else if (!mDoSwitch && mDoScale) {
      // Do Scale
      System.out.println("Running auto: Scale");
    } else {
      // Neither
      System.out.println("Running auto: None");
    }

    mAutoStartTime = System.currentTimeMillis();
  }

  public void executeLoop() {
    if (mRunAuto) {
      if (mDoSwitch && mDoScale) {
        // Do both
        System.out.println("Running auto: Both");
      } else if (mDoSwitch && !mDoScale) {
        // Do Switch
        System.out.println("Running auto: Switch");
        mAutoDoSwitch.executeLoop();
      } else if (!mDoSwitch && mDoScale) {
        // Do Scale
        System.out.println("Running auto: Scale");
      } else {
        // Neither
        System.out.println("Running auto: None");
      }
    }
  }

  public static long getAutoTime() {
    return System.currentTimeMillis() - mAutoStartTime;
  }
}
