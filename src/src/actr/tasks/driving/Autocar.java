package actr.tasks.driving;

import java.awt.Color;
import java.awt.Graphics;

/**
 * An automated car that drives itself down the road.
 *  
 * @author Dario Salvucci
 */
public class Autocar extends Vehicle
{
	boolean braking = false;
	double lastBrakeTime = 0;
	double brakeSpacing = 4; // time between braking events
	
	static double nextBrakeTime = 999999;

	public Autocar()
	{
		super ();
		speed = 0;
	}

	void update (Env env)
	{
		if (Env.scenario.leadCarBrakes)
		{
			if (!braking && (lastBrakeTime + brakeSpacing <= env.time))
			{
				braking = true;
				lastBrakeTime = env.time;
			}
			else if (braking && (lastBrakeTime + 2.0 <= env.time)) braking = false;
		}
		
		if (Env.scenario.simCarConstantSpeed || (env.simcar.speed < 10.0))
		{
			speed = env.simcar.speed;
			fracIndex = env.simcar.fracIndex + 20.0;
		}
		else
		{
			if (Env.scenario.leadCarConstantSpeed)
			{
				double fullspeed = Utilities.mph2mps (Env.scenario.leadCarMPH);
				if (speed < fullspeed) speed += .1;
				else speed = fullspeed;
			}
			else
			{
				// from CSR 2002
				speed = 20 + 5*Math.sin(fracIndex/100.0) + 5*Math.sin(13.0+fracIndex/53.0) + 5*Math.sin(37.0+fracIndex/141.0);
			}

			//speed = 20.0;

			fracIndex += speed * Env.sampleTime;
			if (fracIndex < env.simcar.fracIndex + 11.0) fracIndex = env.simcar.fracIndex + 11.0;
		}

		//fracIndex = env.simcar.fracIndex + 15.0;

		p = env.road.middle (fracIndex);
		p.y = .65;
		
		h = env.road.heading (fracIndex);
	}

	void draw (Graphics g, Env env)
	{
		Position pos1 = Road.location (fracIndex, 2.2);
		pos1.y = 0.0;
		Coordinate im1 = env.world2image (pos1);

		Position pos2 = Road.location (fracIndex, 2.8);
		pos2.y = 1.0;
		Coordinate im2 = env.world2image (pos2);

		g.setColor (Color.blue);
		g.fillRect (im1.x, im2.y, im2.x-im1.x, im1.y-im2.y);
	}
}
