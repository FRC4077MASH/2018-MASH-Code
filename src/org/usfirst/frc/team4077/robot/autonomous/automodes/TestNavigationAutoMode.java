package org.usfirst.frc.team4077.robot.autonomous.automodes;

import org.usfirst.frc.team4077.robot.autonomous.NavigatePID;
import org.usfirst.frc.team4077.robot.components.Drive;
import org.usfirst.frc.team4077.robot.autonomous.automodes.AutoModeSelector.AutoStartPosition;

import edu.wpi.first.wpilibj.DriverStation;

public class TestNavigationAutoMode {
  private NavigatePID mNavigatorPID;
  private Drive mDrive;

  private double mMovementArray[][] =
      new double[][] {{0, 1.0, 120}, {2, 1.0, -90}, {0, 1.0, 12}, {-1, 0, 0}};

  public TestNavigationAutoMode(Drive drive) {
    mNavigatorPID = new NavigatePID(drive, 5);
    mNavigatorPID.setTunings(0.04, 0, 0);
  }

  public void init(DriverStation.Alliance alliance, String gameSpecificData,
                   AutoStartPosition autoStartPosition) {
    mNavigatorPID.initialize(mMovementArray);
  }

  public void executeLoop() {
    mNavigatorPID.loopNavigation();
    mDrive.printTelemetry();
  }
}
