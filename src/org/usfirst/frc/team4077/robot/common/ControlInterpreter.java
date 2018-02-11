package org.usfirst.frc.team4077.robot.common;
import edu.wpi.first.wpilibj.Joystick;

public class ControlInterpreter {
  private static ControlInterpreter mInstance = new ControlInterpreter();

  private Joystick mDriveStick = new Joystick(0);
  private Joystick mOperatorStick = new Joystick(0);

  // NOTE Constructor
  public ControlInterpreter() {}
  public static ControlInterpreter getInstance() { return mInstance; }

  // NOTE Drive Controls
  public double getThrottle() { return mDriveStick.getRawAxis(1); }

  public double getStrafe() { return mDriveStick.getRawAxis(0); }

  public double getTurn() { return mDriveStick.getRawAxis(4); }

  public boolean getSlowButton() { return mDriveStick.getRawButton(5); }

  // NOTE TODO Operator Controls
}
