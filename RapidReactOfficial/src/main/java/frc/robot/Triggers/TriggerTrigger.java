package frc.robot.Triggers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerTrigger extends Trigger {

    public enum Hand {
        kLeft,
        kRight;
    }

    XboxController controller;
    Hand hand;
    double deadZone;
 

    /**
     * WPITrigger that triggers if the given ControllerTrigger is above the deadZone. 
     *  
     * @param controller the controller to pull the trigger axis from
     * @param hand which side the trigger is on (Hand.kLeft or Hand.kRight)
     * @param deadZone the minimum value this WPITrigger will activate at
     */
    public TriggerTrigger(XboxController controller, Hand hand, double deadZone) {
        super();
        this.controller = controller;
        this.hand = hand;
        this.deadZone = deadZone;
    }

    @Override
    public boolean get() {
        if (hand == Hand.kLeft) {
            return controller.getLeftTriggerAxis() > deadZone ? true : false;
        } else {
            return controller.getRightTriggerAxis() > deadZone ? true : false;
        }
        
    }

    
}