package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

public class Arm {
	// Initializing Variables
	private final CANSparkMax liftMotor;
	private final CANSparkMax extendMotor;
	private final WPI_TalonSRX intakeMotor;
	private final DigitalInput upperLiftLimit;
	private final DigitalInput lowerLiftLimit;
	private final DigitalInput minExtensionLimit;
	private final DigitalInput maxExtensionLimit;
	private final AddressableLED ledStrip;
	private final AddressableLEDBuffer ledBuffer;

	public Arm(int liftCanId, int extendCanId, int intakeCanId, int ledPWM) {
		liftMotor = new CANSparkMax(liftCanId, CANSparkMaxLowLevel.MotorType.kBrushless); // controlled by CAN SparkMax
		extendMotor = new CANSparkMax(extendCanId,CANSparkMaxLowLevel.MotorType.kBrushless);// controlled by CAN SparkMax
		intakeMotor = new WPI_TalonSRX(intakeCanId); // controlled by TalonSRX (for claw)
		upperLiftLimit = new DigitalInput(0);
		lowerLiftLimit = new DigitalInput(1);
		maxExtensionLimit = new DigitalInput(2); // limit switch
		minExtensionLimit = new DigitalInput(3); // limit switch
		ledStrip = new AddressableLED(ledPWM);   //LEDs
		ledStrip.setLength(1);    //Replace 1 with number of LEDs
		ledBuffer = new AddressableLEDBuffer(ledPWM);

	}

	public void update(XboxController controller) { 
		//reads the left joystick's y-axis value and checks limit switches (arm itself)
		if (
			(upperLiftLimit.get() && controller.getLeftY() > 0) ||
			(lowerLiftLimit.get() && controller.getLeftY() < 0)
		) {
			liftMotor.set(0); //stop moving
		} else {
			liftMotor.set(controller.getLeftY());
		}

		// reads the left joystick's y-axis value and checks limit switches (extension)
		if ((minExtensionLimit.get() && controller.getRightY() < 0) ||
				(maxExtensionLimit.get() && controller.getRightY() > 0)) {
			extendMotor.set(0); //default speed
		} else {
			extendMotor.set(controller.getRightY());
		}
		//determine if claw is moving
		if (controller.getRightTriggerAxis() > 0) {
			intakeMotor.set(controller.getRightTriggerAxis());
		} else if (controller.getLeftTriggerAxis() > 0) {
			intakeMotor.set(-controller.getLeftTriggerAxis());
		} else {
			intakeMotor.set(0);
		}
		
		if(controller.getXButton()){
			// purple
			for (int i = 0; i < ledBuffer.getLength(); i++){
				ledBuffer.setRGB(i, 255, 0, 255);
			}
			ledStrip.setData(ledBuffer);
			while (controller.getXButton()) {
				ledStrip.start();
			}
			ledStrip.stop();
		}
		if(controller.getYButton()){
			//yellow 
			for (int i = 0; i < ledBuffer.getLength(); i++){
				ledBuffer.setRGB(i, 255, 255, 0);
			}
			ledStrip.setData(ledBuffer);
			while (controller.getYButton()) {
				ledStrip.start();
			}
			ledStrip.stop();
		}
	}
}
