/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Servo;

/**
 * Add your docs here.
 */
public class PID_Servo_In implements PIDSource {
    Servo servo;
    private double max_angle;
    private double min_angle;

    public PID_Servo_In(Servo servo, double min_angle, double max_angle) {
        this.servo = servo;
        this.min_angle = min_angle;
        this.max_angle = max_angle;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return
        (servo.getAngle() - min_angle - .5 * (max_angle - min_angle)) /
        (.5 * (max_angle - min_angle));
    }
}
