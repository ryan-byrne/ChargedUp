
/* 
 * 
 * package frc.robot;
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

XboxController exampleXbox = new XboxController(0);

public class LEDs {

	I2C i2c;
	byte[] toSend = new byte[1];

	public void robotInit (){
		DigitalModule module = DigitalModule.getInstance(2);
		i2c = module.getI2C(168);
			toSend[0] = 16;
			Timer.delay(1.2);
			toSend[0] = 1;
	}


	public void Autonomous(){
		boolean on = false;
		System.out.println("ChatGPT got nothing on me");
		while (isAutonomous()){
			if (on)
				toSend[0] = 2;
			else
				toSend[0] = 1;
			on = ! on;
			i2c.transaction(toSend, 1, null, 0);
			Timer.delay(.0005);
		}
	}

	public void teleOp(boolean cfault, boolean brownout, boolean moving){
		boolean on = false;
		System.out.println("Vroom Vroom");
		while (operatorControl()){

			if (XboxController.getYButtonPressed)
				toSend[0] = 6;
			else if (XboxController.getXButtonPressed)
				toSend[0] = 8;
			else if (XboxController.getAButtonPressed)
				toSend[0] = 9;
			else if (XboxController.getBButtonPressed)
				toSend[0] = 7;
			else if (XboxController.getXButtonReleased)
				toSend[0] = 1;
			else if (XboxController.getYButtonReleased)
				toSend[0] = 1;
			else if (XboxController.getAButtonReleased)
				toSend[0] = 1;
			else if (XboxController.getBButtonReleased)
				toSend[0] = 1;
			else if (cfault);
				toSend[0] = 10;
			else if (brownout);
				toSend[0] = 11;
			else if (moving);
				toSend[0] = 12;
			else ()
				toSend[0] = 1;
			on = ! on;
			i2c.transaction(toSend, 1, null, 0);
			Timer.delay(.0005);

	}
}
*/