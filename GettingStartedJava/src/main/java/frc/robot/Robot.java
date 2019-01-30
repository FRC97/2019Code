/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
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
  //private final DifferentialDrive m_robotDrive
  //    = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private final TalonSRX talon0 = new TalonSRX(0);
  private final TalonSRX talon1 = new TalonSRX(1);
  private final TalonSRX talon2 = new TalonSRX(2);
  private final TalonSRX talon3 = new TalonSRX(3);
  private final DigitalInput linesensor = new DigitalInput(0);
  private final VictorSP victor = new VictorSP(5); 

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
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
    //   m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    // } else {
    //   m_robotDrive.stopMotor(); // stop robot
    // }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    direction = .25;
    log = "";
  }

  private final double init_direction = .15;
  private double direction = .15;
  private double turn = 1; // (1 = right, -1 = left)
  private boolean prev_on_tape = false;
  private final double speed = .15;
  private Timer time_on_tape = new Timer();
  private String log = "";
  private int count = 0;

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    count++;
    //m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
    //talon2.set(ControlMode.PercentOutput, m_stick.getY());
    //talon3.set(ControlMode.PercentOutput, m_stick.getX());
    SmartDashboard.putBoolean("Line Sensor", linesensor.get());
    if(m_stick.getTrigger()) {
      if(!linesensor.get()) {
        if(prev_on_tape) { // reverse direction based on time_on_tape
          turn *= -1;
          direction = turn * init_direction * .25 * Math.pow(.9, time_on_tape.get());
          log += "(" + time_on_tape.get() + ", " + count + "), ";
          SmartDashboard.putString("auto log", log);
        }
        prev_on_tape = false;
      }
      else {
        if(!prev_on_tape) {
          time_on_tape.reset();
          time_on_tape.start();
          count = 0;
        }
        prev_on_tape = true;
      }
      drive(speed, direction);
    }
    else {
      drive(-m_stick.getY(), m_stick.getX());
    }
    SmartDashboard.putNumber("z axis", (-m_stick.getZ() + 1) / 2);
    if(!m_stick.getRawButton(7)) {
      victor.set((-m_stick.getZ() + 1) / 4);
    } else {
      victor.set(-1 * (-m_stick.getZ() + 1) / 4);      
    }
  }

  // direction(right->1, left->-1)
  private void drive(double speed, double direction) {
    SmartDashboard.putNumber("direction", direction);
    SmartDashboard.putNumberArray("drive outputs", new double[] {constrain(speed - direction), constrain(speed + direction)});
    talon0.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // right
    talon2.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // right
    talon1.set(ControlMode.PercentOutput, constrain(speed + direction)); // left
    talon3.set(ControlMode.PercentOutput, constrain(speed + direction)); // left
  }

  private double constrain(double num) {
    if(num > 1) {
      return 1;
    }
    if(num < -1) {
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
}
