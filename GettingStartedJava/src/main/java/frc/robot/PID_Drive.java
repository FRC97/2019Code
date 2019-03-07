/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class PID_Drive implements PIDOutput {
    TalonSRX fr;
    TalonSRX br;
    TalonSRX fl;
    TalonSRX bl;
    

    public PID_Drive(TalonSRX fr, TalonSRX br, TalonSRX fl, TalonSRX bl) {
        this.fr = fr;
        this.br = br;
        this.fl = fl;
        this.bl = bl;
    }

    @Override
    public void pidWrite(double output) {
        output = constrain(output);
        output = output * Math.PI/2;
        drive(Math.cos(output), Math.sin(output));
    }

    // direction(right->1, left->-1)
    private void drive(double speed, double direction) {
        SmartDashboard.putNumber("direction", direction);
        SmartDashboard.putNumber("speed", speed);

        // 2018 Bot Reverse
        fr.set(ControlMode.PercentOutput, constrain(speed + direction)); // right
        br.set(ControlMode.PercentOutput, constrain(speed + direction)); // right
        fl.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // left
        bl.set(ControlMode.PercentOutput, -1 * constrain(speed - direction)); // left
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
}
