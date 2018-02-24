package org.usfirst.frc.team4077.robot.components;

import com.ctre.phoenix.motorcontrol.can.*;

public class Manipulator {
  private static final double DEADBAND = 0.1;

  // NOTE Private Objects
  private static Manipulator mInstance = new Manipulator();

  private WPI_TalonSRX mLeftIntake = new WPI_TalonSRX(9);
  private WPI_TalonSRX mRightIntake = new WPI_TalonSRX(10);

  // NOTE Public Constants

  // NOTE Private Variables
  private boolean mIsEnabled = false;

  // NOTE Constructor
  private Manipulator() {
    mLeftIntake.setInverted(true);
    mRightIntake.setInverted(false);
  }

  public static Manipulator getInstance() { return mInstance; }

  // NOTE Setters
  public void enableComponent(boolean enabled) { mIsEnabled = enabled; }

  public void intake(double power) {
    if (mIsEnabled) {
      double powerVal = applyDeadband(power, DEADBAND);
      mLeftIntake.set(powerVal);
      mRightIntake.set(powerVal);
    }
  }

  public void setIndividualMotorPower(String motorAbbreviation, double power) {
    if (mIsEnabled) {
      switch (motorAbbreviation) {
      case "L":
        mLeftIntake.set(power);
        break;
      case "R":
        mRightIntake.set(power);
        break;
      default:
        break;
      }
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
