package actr.tasks.driving;

/**
 * A general vehicle class (subclassed by other classes).
 *  
 * @author Dario Salvucci
 */
public class Vehicle
{
	Position p;
	Position h;
	double speed;
	double fracIndex;
	
	public Vehicle()
    {
		p = new Position (0, 0);
		h = new Position (1, 0);
		fracIndex = 0;
		speed = 0;
	}
}
