/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helpers;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Add your docs here.
 */
public class CustomPIDOutput extends PIDOutput {
    private double output_value = 0.0;
    @Override
    private void pidWrite(double c){
        output_value  = c;
    }
    public double getOutput(){
        return output_value;
    }
}
