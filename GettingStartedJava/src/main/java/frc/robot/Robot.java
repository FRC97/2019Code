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
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Ultrasonic;

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
  private final Solenoid solenoid0 = new Solenoid(0);
  private final Solenoid solenoid1 = new Solenoid(1);
  private final VictorSP vacuum_a = new VictorSP(0);
  private final VictorSP vacuum_b = new VictorSP(2);
  private final VictorSP flip = new VictorSP(1);
  private final Relay valve = new Relay(0);
  private final Ultrasonic right_ultra = new Ultrasonic(3, 4);
  private final Ultrasonic left_ultra = new Ultrasonic(5, 6);

  // Camera Stuff:
  private final Servo camera_servo = new Servo(8);
  private boolean tape_previously_detected = false;
  private double current_camera_angle = camera_servo.getAngle();
  private double auto_turn;
  private final PIDController camera_pid = new PIDController(.004, 0, .004, new PID_NetworkTables(),
      new PID_Servo_Out(camera_servo, 45, 140), 0.02);

  // 2018
  private final TalonSRX talon0 = new TalonSRX(0);
  private final TalonSRX talon1 = new TalonSRX(1);
  private final TalonSRX talon2 = new TalonSRX(2);
  private final TalonSRX talon3 = new TalonSRX(3);

  // private final PIDController drive_pid = new PIDController(.5, 0, 0, new PID_Servo_In(camera_servo, 45, 140),
  //     new PID_Drive(talon0, talon2, talon1, talon3, .2), 0.02);
  private final PIDController drive_pid = new PIDController(.5, 0, 0, new PID_NetworkTables(),
      new PID_Drive(talon0, talon2, talon1, talon3, .2), 0.02);

  NetworkTable table;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("ChickenVision");
    LiveWindow.add(camera_pid);
    LiveWindow.add(drive_pid);
    SmartDashboard.putNumber("tilt_thresh", .1);
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    teleopInit();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }
    // Drive for 2 seconds
    // if (m_timer.get() < 2.0) {
    // m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    // } else {
    // m_robotDrive.stopMotor(); // stop robot
    // }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    m_timer.reset();
    m_timer.start();

    camera_servo.setAngle(92.5);

    LiveWindow.setEnabled(true);
    camera_pid.disable();
    camera_pid.setSetpoint(0);
    // camera_pid.setInputRange(-35, 35);
    camera_pid.setName("camera_pid");
    camera_pid.setPercentTolerance(7.5);

    drive_pid.disable();
    drive_pid.setSetpoint(0);
    drive_pid.setName("drive_pid");
    drive_pid.setPercentTolerance(7.5);
    // camera_pid.setOutputRange(-1, 1);

    vacuum_a.set(1);
    vacuum_b.set(1);

    right_ultra.setAutomaticMode(true);
    left_ultra.setAutomaticMode(true);
  }

  private final double max_turn = .8;
  private double tilt_thresh = .2;
  private boolean drive_pid_enabled = false;
  private boolean camera_pid_enabled = false;

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    tilt_thresh = SmartDashboard.getNumber("tilt_thresh", 0);
    boolean tape_detected = table.getEntry("tapeDetected").getBoolean(false);

    // Enable/Disable drive_pid?
    // if ((m_stick.getTrigger() && tape_detected) && !drive_pid_enabled) {
    //   drive_pid.enable();
    //   drive_pid_enabled = true;
    // } else if (!m_stick.getTrigger() && drive_pid_enabled) {
    //   drive_pid.disable();
    //   drive_pid_enabled = false;
    // }

    // Enable/Disable camera_pid?
    // if (m_stick.getTrigger() && !camera_pid_enabled) {
    //   camera_pid.enable();
    //   //camera_pid_enabled = true;
    // } else if (!m_stick.getTrigger() && camera_pid_enabled) {
    //   camera_pid.disable();
    //   camera_pid_enabled = false;
    //   camera_servo.setAngle(92.5);
    // }

    // Drive controls for vp
    // if(drive_pid_enabled) {
    //   double tilt = table.getEntry("tapeTilt").getNumber(0).doubleValue();
    //   if (tilt > tilt_thresh) {
    //     drive_pid.setSetpoint(-max_turn);
    //     SmartDashboard.putBoolean("centerline_found", false);
    //   } else if (tilt < -tilt_thresh) {
    //     drive_pid.setSetpoint(max_turn);
    //     SmartDashboard.putBoolean("centerline_found", false);
    //   } else {
    //     drive_pid.setSetpoint(0);
    //     SmartDashboard.putBoolean("centerline_found", true);
    //   }
    // } else if(m_stick.getTrigger()){
    //   drive(.15, 0);
    // }
    // else {
    //   double trim = (1 - m_stick.getZ()) / 2;
    //   joy_drive(-m_stick.getY(), m_stick.getX(), trim);
    // }

    double right_dist = right_ultra.getRangeInches();
    double left_dist = left_ultra.getRangeInches();
    double wall_angle = Math.cbrt(Math.atan((right_dist - left_dist)/18));
    double wall_thresh = .05;
    SmartDashboard.putNumber("right_dist", right_dist);
    SmartDashboard.putNumber("left_dist", left_dist);
    SmartDashboard.putNumber("wall_angle", wall_angle);

    // Drive controls for line sensing
    if(m_stick.getTrigger()) {
      if(line_middle.get()) {
        if(Math.abs(wall_angle) < wall_thresh) {
          // Line following
          drive(.15, 0);
        } else {
          drive(0, wall_angle);
        }
      } else {
        double trim = (1 - m_stick.getZ()) / 2;
        joy_drive(.55, m_stick.getX(), trim);
      }
    } else {
      double trim = (1 - m_stick.getZ()) / 2;
      joy_drive(-m_stick.getY(), m_stick.getX(), trim);
    }

    // Flipping
    if(m_stick.getRawButton(6)) {
      double trim = (1 - m_stick.getZ()) / 2;
      flip.set(-1 * trim);
    } else if(m_stick.getRawButton(7)) {
      double trim = (1 - m_stick.getZ()) / 2;
      flip.set(1 * trim);
    } else {
      flip.set(0);
    }
    
    // Vacuum
    if(m_stick.getRawButton(2)) {
      vacuum_a.set(0);
      vacuum_b.set(0);
      valve.set(Value.kForward);
    } else {
      vacuum_a.set(1);
      vacuum_b.set(1);
      valve.set(Value.kOff);
    }

    // // Valve
    // if(m_stick.getRawButton(5)) {
    //   valve.set(Value.kForward);
    // } else if(m_stick.getRawButton(4)) {
    //   valve.set(Value.kReverse);
    // } else {
    //   valve.set(Value.kOff);
    // }
  }
  // SmartDashboard.putNumber("M_Timer", m_timer.get());
  // //SmartDashboard.putNumber("tapeTiltnew",
  // table.getEntry("tapeTilt").getNumber(0).doubleValue());
  // // m_robotDrive.arcadeDrive(m_stick.gRelay.Value.kForwardetY(),
  // m_stick.getX());
  // // talon2.set(ControlMode.PercentOutput, m_stick.getY());
  // // talon3.set(ControlMode.PercentOutput, m_stick.getX());
  // SmartDashboard.putBoolean("Right Line Sensor", line_right.get());
  // SmartDashboard.putBoolean("Left Line Sensor", line_left.get());

  // if (m_stick.getRawButton(7)) {
  // // solenoid.set(true);
  // vacuum.set(Relay.Value.kOff);
  // SmartDashboard.putString("vacuum", "off");
  // } else if (m_stick.getRawButton(6)) {
  // // solenoid.set(false);
  // vacuum.set(Relay.Value.kForward);
  // SmartDashboard.putString("vacuum", "forward");
  // } else if (m_stick.getRawButton(8)) {
  // vacuum.set(Relay.Value.kReverse); // kReverse when red is on tube side
  // SmartDashboard.putString("vacuum", "reverse");
  // }

  // if (m_stick.getRawButton(4)) {
  // solenoid0.set(true);
  // }
  // else {
  // solenoid0.set(false);
  // }

  // if (m_stick.getRawButton(5)) {
  // solenoid1.set(true);
  // }
  // else {
  // solenoid1.set(false);
  // }

  // if (m_stick.getTrigger()) {
  // // double tilt = SmartDashboard.getNumber("tapeTilt", 0);
  // // double yaw = SmartDashboard.getNumber("tapeYaw", 0);
  // double tilt = table.getEntry("tapeTilt").getNumber(0).doubleValue();
  // // double yaw = table.getEntry("tapeYaw").getNumber(0).doubleValue();

  // double yaw = (current_camera_angle - 92.5)/2;
  // // tilt = tilt / 5;

  // // Exponential decay towards .2 from below (gets there tilt ~= .5)
  // // double goal_yaw = -.5 * Math.pow(.2 / 100, tilt) + .2;
  // double max_spin = .085;
  // double max_tilt = .25;
  // // Sigmoid function y=2*max_spin/(1+3^(-4*tilt/max_tilt))-max_spin
  // double goal_yaw = 2 * max_spin/(1 + Math.pow(3, -4*tilt/max_tilt)) -
  // max_spin;
  // double corrected_yaw = Math.pow(yaw / 240, 3);
  // SmartDashboard.putNumber("corrected_yaw", corrected_yaw);
  // SmartDashboard.putNumber("goal_yaw", goal_yaw);

  // auto_turn = goal_yaw + corrected_yaw;

  // drive(speed, goal_yaw + corrected_yaw);
  // } else {
  // drive(-m_stick.getY(), m_stick.getX());
  // }

  // // servo follow target
  // if(true) {
  // current_camera_angle = camera_servo.getAngle();
  // double target_angle = 47.5 + 45; // middle

  // boolean tape_detected = table.getEntry("tapeDetected").getBoolean(false);
  // // if(tape_previously_detected && !tape_detected) {

  // // m_timer.stop();
  // // m_timer.reset();
  // // m_timer.start();
  // // }

  // SmartDashboard.putNumber("current_camera_angle", current_camera_angle);
  // // tape_previously_detected = tape_detected;
  // // if(tape_detected) {
  // // double yaw = table.getEntry("tapeYaw").getNumber(0).doubleValue();
  // // SmartDashboard.putNumber("yaw", yaw);
  // // target_angle = current_camera_angle + Math.copySign(Math.pow(yaw/25, 2),
  // yaw);
  // // if(target_angle > 140) {
  // // target_angle = 140;
  // // } else if (target_angle < 45) {
  // // target_angle = 45;
  // // }
  // // }
  // // camera_servo.setAngle(target_angle);

  // }

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

  private void joy_drive(double speed, double direction, double trim) {
    drive(trim * Math.pow(speed, 3), trim * .75 * Math.pow(direction, 3));
  }

  // direction(right->1, left->-1)
  private void drive(double speed, double direction) {
    SmartDashboard.putNumber("direction", direction);
    SmartDashboard.putNumber("speed", speed);
    // SmartDashboard.putNumberArray("drive outputs",
    // new double[] { constrain(speed - direction), constrain(speed + direction) });
    // 2019 Bot
    // drive3.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // right
    // drive5.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // right
    // drive2.set(ControlMode.PercentOutput, constrain(speed + direction)); // left
    // drive4.set(ControlMode.PercentOutput, constrain(speed + direction)); // left

    // Weird
    // if(speed < 0) {
    // direction *= -1;
    // }
    // direction = direction * direction * direction;
    // speed = speed * speed * speed;
    // 2018 Bot
    // talon0.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); //
    // right
    // talon2.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); //
    // right
    // talon1.set(ControlMode.PercentOutput, constrain(speed + direction)); // left
    // talon3.set(ControlMode.PercentOutput, constrain(speed + direction)); // left

    // 2018 Bot Reverse
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
  private double speed = .20;
}