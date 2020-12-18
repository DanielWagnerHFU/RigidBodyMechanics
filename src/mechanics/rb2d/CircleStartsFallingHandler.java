package mechanics.rb2d;

public class CircleStartsFallingHandler implements Runnable {

	private RigidBody circle;
	
	public CircleStartsFallingHandler(RigidBody circle) {
		this.circle = circle;
	}
	
	@Override
	public void run() {
		circle.state = BodyState.FLYING;
		circle.a.set(0,circle.g);
	}

}
