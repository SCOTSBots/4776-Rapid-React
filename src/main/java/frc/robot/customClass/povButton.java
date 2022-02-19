// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.customClass;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/** Add your docs here. */
public class povButton extends Button{

    private XboxController controller;
    private double targetAngle;

    public enum POVDirection {
        Left,
        Right,
        Up,
        Down,
        UpLeft,
        UpRight,
        DownLeft,
        DownRight
    }

    public povButton(XboxController controller, POVDirection direction) {
        this.controller = controller;

        switch (direction) {
            case Up:
                this.targetAngle = 0;
                break;
            case UpRight:
                this.targetAngle = 45;
                break;
            case Right:
                this.targetAngle = 90;
                break;
            case DownRight:
                this.targetAngle = 135;
                break;
            case Down:
                this.targetAngle = 180;
                break;
            case DownLeft:
                this.targetAngle = 225;
                break;
            case Left:
                this.targetAngle = 270;
                break;
            case UpLeft:
                this.targetAngle = 315;
                break;
        }
    }


    @Override
    public boolean get(){
        if(controller.getPOV() == targetAngle){
            return true;
        }
        else {
            return false;
        }
    }
}
