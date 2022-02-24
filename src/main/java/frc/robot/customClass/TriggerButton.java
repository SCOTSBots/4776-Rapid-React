// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.customClass;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/** Add your docs here. */
public class TriggerButton extends Button{

    private XboxController controller;
    private XboxController.Axis axis;


    public TriggerButton(XboxController controller, XboxController.Axis axis) {
        this.controller = controller;
        this.axis = axis;        
    }

    @Override
    public boolean get(){
        return (controller.getRawAxis(axis.value) >= 0.5);
    }
}
