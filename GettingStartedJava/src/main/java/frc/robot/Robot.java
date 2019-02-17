/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  // private final DifferentialDrive m_robotDrive
  // = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private final TalonSRX drive2 = new TalonSRX(2);
  private final VictorSPX drive4 = new VictorSPX(4);
  private final VictorSPX drive5 = new VictorSPX(5);
  private final TalonSRX drive3 = new TalonSRX(3);
  private final DigitalInput line_middle = new DigitalInput(0);
  private final DigitalInput line_left = new DigitalInput(1);
  private final DigitalInput line_right = new DigitalInput(2);
  private final Solenoid solenoid = new Solenoid(0);
  private final Relay vacuum = new Relay(0);
  private final VictorSP flip = new VictorSP(2);

  // 2018
  private final TalonSRX talon0 = new TalonSRX(0);
  private final TalonSRX talon1 = new TalonSRX(1);
  private final TalonSRX talon2 = new TalonSRX(2);
  private final TalonSRX talon3 = new TalonSRX(3);

  NetworkTable table;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("ChickenVision");
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    // if (m_timer.get() < 2.0) {
    // m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    // } else {
    // m_robotDrive.stopMotor(); // stop robot
    // }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {

  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    //SmartDashboard.putNumber("tapeTiltnew", table.getEntry("tapeTilt").getNumber(0).doubleValue());
    // m_robotDrive.arcadeDrive(m_stick.gRelay.Value.kForwardetY(), m_stick.getX());
    // talon2.set(ControlMode.PercentOutput, m_stick.getY());
    // talon3.set(ControlMode.PercentOutput, m_stick.getX());
    SmartDashboard.putBoolean("Right Line Sensor", line_right.get());
    SmartDashboard.putBoolean("Left Line Sensor", line_left.get());

    if (m_stick.getRawButton(7)) {
      // solenoid.set(true);
      vacuum.set(Relay.Value.kOff);
      SmartDashboard.putString("vacuum", "off");
    } else if (m_stick.getRawButton(6)) {
      // solenoid.set(false);
      vacuum.set(Relay.Value.kForward);
      SmartDashboard.putString("vacuum", "forward");
    } else if (m_stick.getRawButton(8)) {
      vacuum.set(Relay.Value.kReverse); // kReverse when red is on tube side
      SmartDashboard.putString("vacuum", "reverse");
    }

    if (m_stick.getTrigger()) {
      // double tilt = SmartDashboard.getNumber("tapeTilt", 0);
      // double yaw = SmartDashboard.getNumber("tapeYaw", 0);
      double tilt = table.getEntry("tapeTilt").getNumber(0).doubleValue();
      double yaw = table.getEntry("tapeYaw").getNumber(0).doubleValue();

      // tilt = tilt / 5;

      // Exponential decay towards .2 from below (gets there tilt ~= .5)
      // double goal_yaw = -.5 * Math.pow(.2 / 100, tilt) + .2;
      double max_spin = .075;
      double max_tilt = .25;
      // Sigmoid function y=2*max_spin/(1+3^(-4*tilt/max_tilt))-max_spin
      double goal_yaw = 2 * max_spin/(1 + Math.pow(3, -4*tilt/max_tilt)) - max_spin;
      double corrected_yaw = yaw / 240;
      SmartDashboard.putNumber("corrected_yaw", corrected_yaw);
      SmartDashboard.putNumber("goal_yaw", goal_yaw);

      drive(speed, goal_yaw + corrected_yaw);
    } else {
      drive(-m_stick.getY(), m_stick.getX());
    }

    // // Set flip motor to Z axis
    // if(m_stick.getTrigger()) {
    // flip.set(m_stick.getZ());
    // } else {
    // flip.set(0);
    // }

    // // Line following on trigger
    // if(m_stick.getTrigger()) {
    // if(!line_left.get() && line_right.get()) {
    // turn = -1;
    // }
    // else if(line_left.get() && !line_right.get()) {
    // turn = 1;
    // }
    // else {
    // turn = 0;
    // }
    // drive(speed, direction * turn);
    // }
    // else {
    // drive(-m_stick.getY(), m_stick.getX());
    // }

  }

  // direction(right->1, left->-1)
  private void drive(double speed, double direction) {
    SmartDashboard.putNumber("direction", direction);
    SmartDashboard.putNumberArray("drive outputs",
        new double[] { constrain(speed - direction), constrain(speed + direction) });
    // 2019 Bot
    // drive3.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); //
    // right
    // drive5.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); //
    // right
    // drive2.set(ControlMode.PercentOutput, constrain(speed + direction)); // left
    // drive4.set(ControlMode.PercentOutput, constrain(speed + direction)); // left
    
    // 2018 Bot
    // talon0.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // right
    // talon2.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // right
    // talon1.set(ControlMode.PercentOutput, constrain(speed + direction)); // left
    // talon3.set(ControlMode.PercentOutput, constrain(speed + direction)); // left
    // 2018 Bot
    talon0.set(ControlMode.PercentOutput, constrain(speed + direction)); // right
    talon2.set(ControlMode.PercentOutput, constrain(speed + direction)); // right
    talon1.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // left
    talon3.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // left
  }

  private double constrain(double num) {
    if (num > 1) {
      return 1;
    }
    if (num < -1) {
      return -1;
    }
    return num;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  private double direction = .15;
  private double turn = 0; // (1 = right, -1 = left)
  private double speed = .25;
}