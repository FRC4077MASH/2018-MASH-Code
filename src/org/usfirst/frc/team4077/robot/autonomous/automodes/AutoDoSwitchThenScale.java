package org.usfirst.frc.team4077.robot.autonomous.automodes;

import org.usfirst.frc.team4077.robot.autonomous.NavigatePID;
import org.usfirst.frc.team4077.robot.components.Drive;
import org.usfirst.frc.team4077.robot.components.Lift;
import org.usfirst.frc.team4077.robot.components.Manipulator;
import org.usfirst.frc.team4077.robot.autonomous.automodes.AutoModeSelector.AutoStartPosition;
import org.usfirst.frc.team4077.robot.common.NavXSensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDoSwitchThenScale {
  private NavigatePID mNavigatorPID;
  private Lift mLift;
  private Manipulator mManipulator;
  private NavXSensor mNavX;

  private AutoModeSelector.AutoStartPosition mAutoPosition;

  private int mStepCount;
  private int mSubStateCount;
  private long mDelayMillis;
  private long mStepStartTime;

  // For state machine, start non-navigation step values at 5 just to be safe
  private double mStepsFromLeftToLeftToLeft[][] = new double[][] {
      {0, 0.5, 144.0}, {2, 0.5, -90},  {0, 0.5, 18},     {5, 0, 0},
      {0, -0.5, -3.0}, {2, 0.5, -10},  {0, 0.5, 55.0},   {2, 0.5, -150},
      {6, 0, 0},       {0, 0.5, 38.0}, {2, 0.5, -140.0}, {2, 0.5, -150.0},
      {0, -0.5, -3.0}, {7, 0, 0},      {0, -0.5, -72},   {2, 0.5, -60},
      {0, 0.5, 12},    {8, 0, 0},      {-1, 0.0, 0.0}};
  private double mStepsFromLeftToLeftToRight[][] = new double[][] {
      {0, 0.5, 144.0}, {2, 0.5, -90},  {0, 0.5, 18},    {5, 0, 0},
      {0, -0.5, -3.0}, {2, 0.5, -10},  {0, 0.5, 55.0},  {2, 0.5, -150},
      {6, 0, 0},       {0, 0.5, 40.0}, {0, -0.5, -3.0}, {7, 0, 0},
      {0, -0.5, -25},  {2, 0.5, -90},  {0, 0.5, 215},   {2, 0.5, 0},
      {0, 0.5, 48},    {2, 0.5, 60},   {8, 0, 0},       {-1, 0.0, 0.0}};

  private double mStepsFromLeftToRightToLeft[][] = new double[][] {
      {0, 0.5, 214.0}, {2, 0.5, -90},  {0, 0.5, 179},  {2, 0.5, -180},
      {0, 0.5, 12},    {5, 0, 0},      {0, -0.5, -3},  {6, 0, 0},
      {0, 0.5, 6},     {2, 0.5, -200}, {2, 0.5, -180}, {7, 0, 0},
      {0, -0.5, 12},   {2, 0.5, 90},   {0, 0.5, 179},  {8, 0, 0},
      {-1, 0.0, 0.0}};
  private double mStepsFromLeftToRightToRight[][] = new double[][] {
      {0, 0.5, 214.0}, {2, 0.5, -90},  {0, 0.5, 179},  {2, 0.5, -180},
      {0, 0.5, 12},    {5, 0, 0},      {0, -0.5, -3},  {6, 0, 0},
      {0, 0.5, 6},     {2, 0.5, -200}, {2, 0.5, -180}, {7, 0, 0},
      {0, -0.5, -14},  {2, 0.5, -60},  {0, 0.5, 72},   {2, 0.5, 60},
      {8, 0, 0},       {-1, 0.0, 0.0}};

  public AutoDoSwitchThenScale(Drive drive, Lift lift, Manipulator manipulator,
                               NavXSensor navX) {
    mLift = lift;
    mManipulator = manipulator;

    mNavX = navX;
    mNavigatorPID = new NavigatePID(drive, mNavX, 5);
    mNavigatorPID.setTunings(0.02, 0.0001, 0.005); //(0.0375, 0.00007,0.004);
    // For comp robot: (0.02,
    // 0.0001, 0.005);
  }

  public void init(DriverStation.Alliance alliance, String gameSpecificData,
                   AutoStartPosition autoStartPosition, long delayMillis) {
    mDelayMillis = delayMillis;
    mAutoPosition = autoStartPosition;
    // Choose which navigation to run
    if (gameSpecificData.charAt(0) == 'L') {
      if (gameSpecificData.charAt(1) == 'L') {
        switch (autoStartPosition) {
        case START_LEFT:
          mNavigatorPID.initialize(mStepsFromLeftToLeftToLeft);
          break;
        case START_CENTER:
          break;
        case START_RIGHT:
        }
      } else {
        switch (autoStartPosition) {
        case START_LEFT:
          mNavigatorPID.initialize(mStepsFromLeftToLeftToRight);
          break;
        case START_CENTER:
          break;
        case START_RIGHT:
        }
      }
    } else {
      if (gameSpecificData.charAt(1) == 'L') {
        switch (autoStartPosition) {
        case START_LEFT:
          mNavigatorPID.initialize(mStepsFromLeftToRightToLeft);
          break;
        case START_CENTER:
          break;
        case START_RIGHT:
        }
      } else {
        switch (autoStartPosition) {
        case START_LEFT:
          mNavigatorPID.initialize(mStepsFromLeftToRightToRight);
          break;
        case START_CENTER:
          break;
        case START_RIGHT:
        }
      }
    }
    // TODO Add switch to select paths
    // mNavigatorPID.initialize(mStepsFromLeftToRight);
    mStepCount = 0;
    mSubStateCount = 0;
  }

  public void executeLoop() {
    SmartDashboard.putString("Navigation Step: ", Integer.toString(mStepCount));
    SmartDashboard.putString("Current Time: ",
                             Long.toString(AutoModeSelector.getAutoTime()));
    switch (mStepCount) {
    case 0:
      if (AutoModeSelector.getAutoTime() >= mDelayMillis) {
        mStepStartTime = AutoModeSelector.getAutoTime();
        mStepCount++;
      } else {
        mNavigatorPID.fullStop();
      }
      break;
    case 1: // Navigate
      if (mNavigatorPID.getCurrentType() == 5) {
        mStepStartTime = AutoModeSelector.getAutoTime();
        mStepCount++;
      } else {
        mNavigatorPID.loopNavigation();
      }
      break;
    case 2: // Raise, deposit, lower
      switch (mSubStateCount) {
      case 0: // Lift
        if (AutoModeSelector.getAutoTime() >=
            (mAutoPosition == AutoStartPosition.START_CENTER ? 1500 : 1750) +
                mStepStartTime) {
          mLift.liftWithLimitSwitchLimits(0.0, true);
          mStepStartTime = AutoModeSelector.getAutoTime();
          mSubStateCount++;
        } else {
          mLift.liftWithLimitSwitchLimits(0.75, true);
          System.out.println("LIFTING");
        }
        break;
      case 1: // Deposit
        if (AutoModeSelector.getAutoTime() >= 500 + mStepStartTime) {
          mManipulator.intake(0.0);
          mStepStartTime = AutoModeSelector.getAutoTime();
          mSubStateCount++;
        } else {
          mManipulator.intake(-1.0);
          System.out.println("SHOOTING");
        }
        break;
      case 2: // Lower
        if (AutoModeSelector.getAutoTime() >= 1000 + mStepStartTime) {
          mLift.liftWithLimitSwitchLimits(0.0, true);
          mStepStartTime = AutoModeSelector.getAutoTime();
          mSubStateCount++;
        } else {
          mLift.liftWithLimitSwitchLimits(-0.75, true);
          System.out.println("LOWERING");
        }
        break;
      default:
        mSubStateCount = 0;
        mStepStartTime = AutoModeSelector.getAutoTime();
        mNavigatorPID.nextMovement();
        mStepCount++;
        break;
      }
      break;
    case 3: // Navigate
      if (mNavigatorPID.getCurrentType() == 6) {
        mManipulator.intake(1.0);
        System.out.println("INTAKING");
        mStepStartTime = AutoModeSelector.getAutoTime();
        mStepCount++;
        mNavigatorPID.nextMovement();
      } else {
        mNavigatorPID.loopNavigation();
      }
      break;
    case 4: // Navigate
      if (mNavigatorPID.getCurrentType() == 7) {
        mManipulator.intake(0.0);
        mStepStartTime = AutoModeSelector.getAutoTime();
        mStepCount++;
        mNavigatorPID.nextMovement();
      } else {
        mNavigatorPID.loopNavigation();
      }
      break;
    case 5: // Navigate
      if (mNavigatorPID.getCurrentType() == 8) {
        mManipulator.intake(0.0);
        mStepStartTime = AutoModeSelector.getAutoTime();
        mStepCount++;
      } else {
        mNavigatorPID.loopNavigation();
      }
      break;
    case 6: // Raise, deposit, lower
      switch (mSubStateCount) {
      case 0: // Lift
        if (mLift.isAtMax()) {
          mLift.liftWithLimitSwitchLimits(0.0, true);
          mStepStartTime = AutoModeSelector.getAutoTime();
          mSubStateCount++;
        } else {
          mManipulator.intake(0.0);
          mLift.liftWithLimitSwitchLimits(0.75, true);
          System.out.println("LIFTING");
        }
        break;
      case 1: // Deposit
        if (AutoModeSelector.getAutoTime() >= 1000 + mStepStartTime) {
          mManipulator.intake(0.0);
          mStepStartTime = AutoModeSelector.getAutoTime();
          mSubStateCount++;
        } else {
          mManipulator.intake(-1.0);
          System.out.println("SHOOTING");
        }
        break;
      case 2: // Lower
        if (AutoModeSelector.getAutoTime() >= 1000 + mStepStartTime) {
          mLift.liftWithLimitSwitchLimits(0.0, true);
          mStepStartTime = AutoModeSelector.getAutoTime();
          mSubStateCount++;
        } else {
          mLift.liftWithLimitSwitchLimits(-0.75, true);
          System.out.println("LOWERING");
        }
        break;
      default:
        mSubStateCount = 0;
        mStepStartTime = AutoModeSelector.getAutoTime();
        mNavigatorPID.nextMovement();
        mStepCount++;
        break;
      }
      break;
    case 7:
      mNavigatorPID.loopNavigation();
      break;
    }
  }

  public boolean getIsDone() {
    if (mNavigatorPID.getCurrentType() == -1) {
      return true;
    } else {
      return false;
    }
  }
}
