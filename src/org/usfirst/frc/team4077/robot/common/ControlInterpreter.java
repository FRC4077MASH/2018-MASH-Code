package org.usfirst.frc.team4077.robot.common;
import edu.wpi.first.wpilibj.Joystick;

public class ControlInterpreter {
  private static ControlInterpreter mInstance = new ControlInterpreter();

  private Joystick mDriveStick = new Joystick(0);
  private Joystick mOperatorStick = new Joystick(1);

  public static final double THROTTLE_SCALE = 1.0;
  public static final double STRAFE_SCALE = 1.0;
  public static final double TURN_SCALE = 1.0;
  public static final boolean CUBE_ROTATION = true;

  // NOTE Constructor
  public ControlInterpreter() {}
  public static ControlInterpreter getInstance() { return mInstance; }

  // NOTE Drive Controls
  public double getThrottle() {
    return -mDriveStick.getRawAxis(1) * THROTTLE_SCALE;
  }

  public double getStrafe() { return mDriveStick.getRawAxis(0) * STRAFE_SCALE; }

  public double getTurn() {
    if (CUBE_ROTATION) {
      return Math.pow((mDriveStick.getRawAxis(4) * TURN_SCALE), 1);
    } else {
      return mDriveStick.getRawAxis(4) * TURN_SCALE;
    }
  }

  public boolean getSlowButton() { return mDriveStick.getRawButton(5); }

  public boolean getPanicButton() { return mDriveStick.getRawButton(6); }

  // NOTE TODO Operator Controls
  public double getLift() { return -mOperatorStick.getRawAxis(1); }

  public boolean getClimbUp() { return mOperatorStick.getRawButton(5); }

  public boolean getClimbDown() { return mOperatorStick.getRawButton(6); }

  public double getCollect() { return mOperatorStick.getRawAxis(2); }

  public double getEjection() { return mOperatorStick.getRawAxis(3); }
}
