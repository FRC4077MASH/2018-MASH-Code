package org.usfirst.frc.team4077.robot.components;

import com.ctre.phoenix.motorcontrol.can.*;

public class Climber {
  private static final double DEADBAND = 0.02;

  // NOTE Private Objects
  private static Climber mInstance = new Climber();

  private WPI_TalonSRX mClimber = new WPI_TalonSRX(5);

  // NOTE Public Constants

  // NOTE Private Variables
  private boolean mIsEnabled = false;

  // NOTE Constructor
  private Climber() { mClimber.setInverted(true); }

  public static Climber getInstance() { return mInstance; }

  // NOTE Setters
  public void enableComponent(boolean enabled) { mIsEnabled = enabled; }

  public void climb(double power) {
    if (mIsEnabled) {
      double powerVal = applyDeadband(power, DEADBAND);
      mClimber.set(powerVal);
    }
  }

  public void resetAll() {
    resetEncoders();
    resetSensors();
  }

  public void resetEncoders() {
    // Nothing yet
  }

  public void resetSensors() {
    // Nothing yet
    // TODO Add code for other sensors
  }

  // NOTE Getters
  public boolean getEnableStatus() { return mIsEnabled; }

  // NOTE Private conversion Methods
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
