package mechanics.rb2d;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.components.VectorMath;
import mechanics.rb2d.misc.BodyState;
import mechanics.rb2d.shapes.Circle;
import mechanics.rb2d.shapes.Polygon;

public class RigidBodyStartsFallingHandler implements Runnable {

	private RigidBody rb;
	private Line2D.Double impactEdge;

	public RigidBodyStartsFallingHandler(RigidBody rb, Line2D.Double impactEdge) {
		this.rb = rb;
		this.impactEdge = rb.lastImpactEdge;
	}

	@Override
	public void run() {
		System.out.println("Start Flying");
		rb.state = BodyState.FLYING;
		rb.a.set(0, rb.g);		

		if (Polygon.class.isAssignableFrom(rb.shape.getClass())) {
			double edgeLength = rb.slidingEdge.abs();
			double v = rb.v.abs();
			if(edgeLength >= v) {
				rb.v.y = edgeLength/v/10;
			}else {
				rb.v.y = v/edgeLength/10;
			}
			System.out.println(rb.v.y);
			if (rb.r.x > impactEdge.x1 && rb.r.x > impactEdge.x2) {
//				rb.phi -= Math.toRadians(5);
				rb.omega = -1;
			} else {
//				rb.phi += Math.toRadians(5);
				rb.omega = 1;
			}
		} else {
			// Circle
			rb.v.y += 0.1;
		}

	}

	private double getTorque(RigidBody rb, Line2D.Double impactEdge) {
		System.out.println();
		Line2D.Double forceLine = new Line2D.Double(rb.r.x, rb.r.y, rb.r.x + rb.Fg.y, rb.r.y + rb.Fg.y);
		System.out.println(forceLine.x1 + " " + forceLine.y1 + " " + forceLine.x2 + " " + forceLine.y2);
		double dist1 = Math.abs(rb.r.x - impactEdge.x1);
		double dist2 = Math.abs(rb.r.x - impactEdge.x2);
		double dist;
		if (dist1 < dist2)
			dist = dist1;
		else
			dist = dist2;
//		System.out.println("dist 1 " + dist1);
//		System.out.println("dist 2 " + dist2);
//		System.out.println("dist " + dist);

		return dist * rb.Fg.abs();
	}

}
