/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * Add your docs here.
 */
public class Camera_Servo extends PIDSubsystem {

  private Servo servo;
  private double angle;
  private double max_angle;
  private double min_angle;

  /**
   * Add your docs here.
   */
  public Camera_Servo() {
    // Intert a subsystem name and PID values here
    super("SubsystemName", 1, 2, 3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return 0.0;
  }

  @Override
  protected void usePIDOutput(double output) {
        // constrain [-1, 1]
        output = Math.min(Math.max(output, -1), 1);
        // map to servo angle [-1, 1] -> [min_angle, max_angle]
        angle = output * (max_angle - min_angle) / 2 + (max_angle + min_angle) / 2;
        servo.set(angle);
  }
}
