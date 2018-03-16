package org.usfirst.frc.team4077.robot.autonomous.automodes;

import org.usfirst.frc.team4077.robot.autonomous.NavigatePID;
import org.usfirst.frc.team4077.robot.components.Drive;
import org.usfirst.frc.team4077.robot.components.Lift;
import org.usfirst.frc.team4077.robot.components.Manipulator;
import org.usfirst.frc.team4077.robot.autonomous.automodes.AutoModeSelector.AutoStartPosition;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDoSwitch {
  private NavigatePID mNavigatorPID;
  private Drive mDrive;
  private Lift mLift;
  private Manipulator mManipulator;

  private int mStepCount;
  private int mSubStateCount;
  private long mDelayMillis;
  private long mStepStartTime;

  // For state machine, start non-navigation step values at 5 just to be safe
  private double mStepsFromLeftToLeft[][] = new double[][] {
      {0, 1.0, 156.0}, {2, 1.0, -90}, {0, 1.0, 18}, {-1, 0.0, 0.0}};
  private double mStepsFromLeftToRight[][] =
      new double[][] {{0, 1.0, 220.0}, {2, 1.0, -90},  {0, 1.0, 168},
                      {2, 1.0, 0},     {0, -1.0, -12}, {-1, 0.0, 0.0}};

  private double mStepsFromCenterToLeft[][] =
      new double[][] {{0, 1.0, 120.0}, {-1, 0.0, 0.0}};
  private double mStepsFromCenterToRight[][] =
      new double[][] {{0, 1.0, 120.0}, {-1, 0.0, 0.0}};

  private double mStepsFromRightToLeft[][] =
      new double[][] {{0, 1.0, 120.0}, {-1, 0.0, 0.0}};
  private double mStepsFromRightToRight[][] = new double[][] {
      {0, 0.5, 156.0}, {2, 0.5, 90}, {0, 0.5, 18}, {-1, 0.0, 0.0}};

  public AutoDoSwitch(Drive drive, Lift lift, Manipulator manipulator) {
    mLift = lift;
    mDrive = drive;
    mManipulator = manipulator;

    mNavigatorPID = new NavigatePID(drive, 5);
    mNavigatorPID.setTunings(0.04, 0.00003, 0.01);
  }

  public void init(DriverStation.Alliance alliance, String gameSpecificData,
                   AutoStartPosition autoStartPosition, long delayMillis) {
    mDelayMillis = delayMillis;
    // Choose which navigation to run
    /*if(gameSpecificData.charAt(0) == 'L') {
            switch (autoStartPosition) {
            case START_LEFT:
                    mNavigatorPID.initialize(mStepsFromLeftToLeft);
                    break;
            case START_CENTER:
                    mNavigatorPID.initialize(mStepsFromCenterToLeft);
                    break;
            case START_RIGHT:
                    mNavigatorPID.initialize(mStepsFromRightToLeft);
            }
    } else {
            switch (autoStartPosition) {
            case START_LEFT:
                    mNavigatorPID.initialize(mStepsFromLeftToRight);
                    break;
            case START_CENTER:
                    mNavigatorPID.initialize(mStepsFromCenterToRight);
                    break;
            case START_RIGHT:
                    mNavigatorPID.initialize(mStepsFromRightToRight);
            }
    }*/
    // TODO Add switch to select paths
    mNavigatorPID.initialize(mStepsFromLeftToRight);
    mStepCount = 0;
    mSubStateCount = 0;
  }

  public void executeLoop() {
    SmartDashboard.putString("Navigation Step: ",
                             Integer.toString(mNavigatorPID.getCurrentStep()));
    /*switch (mStepCount) {
    case 0: // Delay
            if (System.currentTimeMillis() >= mDelayMillis) {
                    mNavigatorPID.nextMovement();
                    mStepStartTime = System.currentTimeMillis();
                    mStepCount++;
            } else {
                    // Do nothing until the time runs out
            }
            break;
    case 1: // Navigate
            if (mNavigatorPID.getCurrentType() == 5) { // Lift
                    mStepStartTime = System.currentTimeMillis();
                    mStepCount++;
            } else {
                    mNavigatorPID.loopNavigation();
            }
    case 2: // Raise, deposit, lower
            switch (mSubStateCount) {
            case 0: // Lift
                    if (System.currentTimeMillis() >= 500 + mStepStartTime) {
                            mLift.liftWithLimitSwitchLimits(0.0, true);
                            mStepStartTime = System.currentTimeMillis();
                            mSubStateCount++;
                    } else {
                            mLift.liftWithLimitSwitchLimits(0.5, true);
                    }
                    break;
            case 1: // Deposit
                    if (System.currentTimeMillis() >= 500 + mStepStartTime) {
                            mManipulator.intake(0.0);
                            mStepStartTime = System.currentTimeMillis();
                            mSubStateCount++;
                    } else {
                            mManipulator.intake(-1.0);
                    }
                    break;
            case 2: // Lower
                    if (System.currentTimeMillis() >= 500 + mStepStartTime) {
                            mLift.liftWithLimitSwitchLimits(0.0, true);
                            mStepStartTime = System.currentTimeMillis();
                            mSubStateCount++;
                    } else {
                            mManipulator.intake(-1.0);
                    }
                    break;
            default:
                    mSubStateCount = 0;
                    mStepStartTime = System.currentTimeMillis();
                    mStepCount++;
                    break;
            }
            break;
    case 3:*/
    mNavigatorPID.loopNavigation();
    // break;
    //}
  }
}
