package actr.model;

/**
 * The TemporalTwo module representing ACT-R's perception of time.
 *  
 * @author Dario Salvucci
 */
public class Temporal2 extends Module
{
	private Model model;
	private int ticks = 0;
	private double tick = 0;

	double timeNoise = .015;
	double timeMultiplier = 1.1;
	double timeMasterStartIncrement = .011;

	Temporal2 (Model model)
	{
		this.model = model;
	}

	void update ()
	{
		// model gets time from java side, doesn't know itself (int)simulation.env.time;
		
		Chunk request = model.getBuffers().get (Symbol.temporal2);
		if (request==null || !request.isRequest()) return;
		request.setRequest (false);
		model.getBuffers().clear (Symbol.temporal2);

		if (request.get(Symbol.isa) == Symbol.get("time"))
		{
			tick = timeMasterStartIncrement;
			ticks = 0;

			Chunk timeChunk = new Chunk (Symbol.getUnique("time"), model);
			timeChunk.set (Symbol.isa, Symbol.time);
			timeChunk.set (Symbol.ticks, Symbol.get (0));
			model.getBuffers().set (Symbol.temporal2, timeChunk);

			model.getBuffers().setSlot (Symbol.temporal2State, Symbol.buffer, Symbol.full);
			model.getBuffers().setSlot (Symbol.temporal2State, Symbol.state, Symbol.busy);
			
			model.removeEvents ("TemporalTwo");
			queueTickIncrement();
		}
	}

	void queueTickIncrement ()
	{
		model.addEvent (new Event (model.getTime() + tick, "TEMPORAL2", "increment ticks [" + (ticks+1) + "]") {
			public void action()
			{
				ticks ++;
				tick *= timeMultiplier;
				tick += Utilities.getNoise (timeNoise * tick);

				model.getBuffers().setSlot (Symbol.temporal2, Symbol.ticks, Symbol.get (ticks));

				queueTickIncrement();
			}
		});
	}
	
	public int getTicks()
	{
		return this.ticks; 
	}
}
