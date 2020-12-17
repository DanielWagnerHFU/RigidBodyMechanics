package mechanics.rb2d;

import java.awt.geom.Line2D;

public class CircleStartsFallingHandler implements Runnable {

	private RigidBody circle;
	private Line2D.Double line;
	
	public CircleStartsFallingHandler(RigidBody circle) {
		this.circle = circle;
//		this.line = ip.impactEdgeLine;
	}
	
	@Override
	public void run() {
		System.out.println("start Falling");
		circle.state = BodyState.FLYING;
		circle.a.set(0,circle.g);
	}

}
