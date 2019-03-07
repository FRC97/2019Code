/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives a servo between two angles
 */
public class PID_Servo_Out implements PIDOutput{
    private Servo servo;
    private double angle;
    private double max_angle;
    private double min_angle;

    /**
     * 
     * @param servo A servo motor
     * @param min_angle Minimum angle that the servo can turn without borking
     * @param max_angle Maximum angle that the servo can turn without borking
     */
    public PID_Servo_Out(Servo servo, double min_angle, double max_angle) {
        this.servo = servo;
        this.min_angle = min_angle;
        this.max_angle = max_angle;
    }

    /**
     * Writes an angle to the servo
     * @param pidInput Value between -1 and 1
     */
    public void pidWrite(double pidInput) {
        SmartDashboard.putNumber("pid_angle", pidInput);
        // angle = servo.getAngle();
        // constrain [-1, 1]
        pidInput = Math.min(Math.max(pidInput, -1), 1);
        pidInput *= -1;
        // map to servo angle [-1, 1] -> [min_angle, max_angle]
        angle = pidInput * (max_angle - min_angle) / 2 + (max_angle + min_angle) / 2;
        servo.setAngle(angle);
    }
}
