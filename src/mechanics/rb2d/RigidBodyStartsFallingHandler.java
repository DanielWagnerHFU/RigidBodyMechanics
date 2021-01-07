package mechanics.rb2d;

import java.awt.geom.Line2D;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.components.VectorMath;

public class RigidBodyStartsFallingHandler implements Runnable {

	private RigidBody rb;
	private Line2D.Double impactEdge;

	public RigidBodyStartsFallingHandler(RigidBody rb, Line2D.Double impactEdge) {
		this.rb = rb;
		this.impactEdge = rb.lastImpactEdge;
	}

	@Override
	public void run() {
		rb.state = BodyState.FLYING;
		rb.a.set(0, rb.g);
		rb.r.y += 0.01;

		if (rb.r.x < impactEdge.x1 && rb.r.x < impactEdge.x2) {
			rb.phi += 0.1;
			rb.r.x -= 0.01;

		} else {
			rb.phi -= 0.1;
			rb.r.x += 0.01;
		}
//		double torque = getTorque(rb, impactEdge);
//		
//		rb.alpha = torque/rb.I;

	}

	private double getTorque(RigidBody rb, Line2D.Double impactEdge) {
		Line2D.Double forceLine = new Line2D.Double(rb.r.x, rb.r.y, rb.r.x + rb.Fg.x, rb.r.y + rb.Fg.y);
		double dist1 = forceLine.ptLineDist(impactEdge.x1, impactEdge.y1);
		double dist2 = forceLine.ptLineDist(impactEdge.x2, impactEdge.y2);
		double dist;
		if (dist1 < dist2)
			dist = dist1;
		else
			dist = dist2;

		return dist * rb.Fg.abs();
	}

}
