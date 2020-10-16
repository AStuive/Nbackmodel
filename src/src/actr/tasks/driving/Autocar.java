package actr.tasks.driving;

import java.awt.Color;
import java.awt.Graphics;

/**
 * An automated car that drives itself down the road.
 * 
 * @author Dario Salvucci
 */
public class Autocar extends Vehicle {

	long roadIndex;
	boolean visible = false;
	int lane = 2;
	Double runningTime = 0.0;
	String distance;

	public Autocar() {
		super();
		speed = 0;
	}

	void update(Env env) {
		// autocar follows 0back
		speed = Integer.parseInt(env.speedsign.speedlimit);
		speed = Utilities.mph2mps(Utilities.kph2mph(speed));

		fracIndex += speed * Env.sampleTime;
		runningTime += Env.sampleTime;

		// autocar switch lane
		if (env.simcar.fracIndex > fracIndex) {
			if (lane == env.simcar.lane && lane != 1) {
				lane = env.simcar.lane - 1;
			} else {
				speed = env.simcar.speed;
			}
		}
		p = env.road.middle(fracIndex, lane);
		p.y = .65;

		h = env.road.heading(fracIndex);
		distance = fracIndex - env.simcar.fracIndex < 20 ? "close" : "far";
	}

	void draw(Graphics g, Env env) {

		Position pos1 = Road.location(fracIndex, lane + 0.2);
		pos1.y = 0.0;
		Coordinate im1 = env.world2image(pos1);

		Position pos2 = Road.location(fracIndex, lane + 0.8);
		pos2.y = 1.0;
		Coordinate im2 = env.world2image(pos2);

		if (im1 != null && im2 != null) {
			g.setColor(Color.blue);
			g.fillRect(im1.x, im2.y, im2.x - im1.x, im1.y - im2.y);
		} else {

		}
	}
}
