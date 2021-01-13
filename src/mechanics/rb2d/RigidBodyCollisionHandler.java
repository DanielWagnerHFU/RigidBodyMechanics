package mechanics.rb2d;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.components.VectorMath;
import mechanics.rb2d.misc.BodyDirection;
import mechanics.rb2d.misc.BodyState;
import mechanics.rb2d.shapes.Circle;
import mechanics.rb2d.shapes.Polygon;
import sun.security.util.Length;

import static java.lang.Math.*;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class RigidBodyCollisionHandler implements Runnable {

	private double k = 0.6;
	private RigidBody rb_p;
	private RigidBody rb_e;
	private Impactpoint ip;
	private boolean showInfo = false;
	private boolean impactCoords = false;

	private int cornerNumber = -1;

	private double friction = 0.03;

	public RigidBodyCollisionHandler(Impactpoint ip) {
		this.rb_p = ip.rb_point;
		this.rb_e = ip.rb_edge;
		this.ip = ip;
	}

	@Override
	public void run() {
		rb_p.state = BodyState.FLYING;
		rb_e.state = BodyState.FLYING;

		if (rb_p.g != 0) {
			rb_p.Fh.set(0, 0);
			rb_p.Fn.set(0, 0);
			rb_p.Fr.set(0, 0);
			rb_p.Fres.set(0, 0);
			rb_p.a.set(0, rb_p.g);
		}

		if (rb_e.g != 0) {
			rb_e.Fh.set(0, 0);
			rb_e.Fn.set(0, 0);
			rb_e.Fr.set(0, 0);
			rb_e.Fres.set(0, 0);
			rb_e.a.set(0, rb_e.g);
		}

		if (showInfo) {
			System.out.println("Collision between:");
			System.out.println(rb_p);
			System.out.println(rb_e);
		}

		Vector2D p = ip.impactPoint;
		Vector2D collisionEdge = VectorMath.normalize(ip.impactEdge);

		// 1. Transformation in das Stoßkoordinatensystem
		Vector2D r1m = VectorMath.sub(rb_p.r, p);
		Vector2D r2m = VectorMath.sub(rb_e.r, p);

		double rot = VectorMath.angle(new Vector2D(0, 1), collisionEdge);
		if (collisionEdge.x < 0)
			rot = -rot;

		Vector2D r1mr = rotateVector2D(r1m, rot);
		Vector2D r2mr = rotateVector2D(r2m, rot);

		Vector2D v1r = rotateVector2D(rb_p.v, rot);
		Vector2D v2r = rotateVector2D(rb_e.v, rot);

//Is Vertex on right hand side?	

		if (!checkVertexIsRight(rb_p, r1mr, rot)) {
			rot += Math.PI;
			r1mr = rotateVector2D(r1mr, Math.PI);
			r2mr = rotateVector2D(r2mr, Math.PI);
			v1r = rotateVector2D(v1r, Math.PI);
			v2r = rotateVector2D(v2r, Math.PI);
		}

		// Zustandsbestimmung nach Stoss
//		if ((Math.abs(v1r.x) + Math.abs(v2r.x) < 0.01)) {
//			rb_p.state = BodyState.STOPPED;
//			rb_e.state = BodyState.STOPPED;
//			if (this.showInfo)
//				System.out.println("Stopped: " + "v1r.x=" + v1r.x + ", v2r.x=" + v2r.x);
//		}

		if (Polygon.class.isAssignableFrom(rb_p.shape.getClass())) {
//			System.out.println("----V.abs " + Math.abs(v1r.x));
			if (Math.abs(v1r.x) < 0.5) {
				if (linesAreParallel(ip, rb_p)) {
					if (rbIsUpside(rb_p, ip.impactEdgeLine)) {
						System.err.println("Parallel");
//				System.out.println("START Sliding");
						startSliding(rb_p, collisionEdge);
						return;
//				}
					} else {

						if (rb_p.r.x > ip.impactPoint.x)
							rb_p.omega = getTorque(rb_p, ip.impactPoint);
						else
							rb_p.omega = -getTorque(rb_p, ip.impactPoint);
					}
				}
			}

		}
		if (Circle.class.isAssignableFrom(rb_p.shape.getClass())) {
			if (Math.abs(v1r.x) < 0.1) {
//				System.out.println("start Rrolling");
				startSliding(rb_p, collisionEdge);
				return;
			}
		}

		System.out.println(v1r);
		if (Math.abs(v1r.x) + Math.abs(v2r.x) < 0.01) {
			rb_p.state = BodyState.STOPPED;
			rb_e.state = BodyState.STOPPED;
			return;
		}

		// 2. Berechnung der neuen Größen im Stoßkoordinatensystem
		double a1 = -r1mr.y;
		double a2 = -r2mr.y;

		double Fx = impulseFx(v1r.x, v2r.x, a1, a2);

		Vector2D V1r = new Vector2D(v1r.x - (Fx / rb_p.m), v1r.y);
		Vector2D V2r = new Vector2D(v2r.x + (Fx / rb_e.m), v2r.y);

		double Omega1 = rb_p.omega + ((a1 * Fx) / rb_p.I);
		double Omega2 = rb_e.omega - ((a2 * Fx) / rb_e.I);

		// 3. Ruecktransformation ins Inertialsystem
		Vector2D V1r_ = rotateVector2D(V1r, -rot);
		Vector2D V2r_ = rotateVector2D(V2r, -rot);


		// 4. Setzen der neuen Werte

		if (impactCoords) {
			// Stosskoordinatensystem
			rb_p.v.set(V1r);
			rb_e.v.set(V2r);

			rb_p.omega = Omega1;
			rb_e.omega = Omega2;

			rb_p.phi = rb_p.phi + rot;
			rb_e.phi = rb_e.phi + rot;

			rb_p.r.set(r1mr);
			rb_e.r.set(r2mr);

		} else {

			// Normale Koordinatensystem
			rb_p.v.set(V1r_);
			rb_e.v.set(V2r_);

			rb_p.omega = Omega1;
			rb_e.omega = Omega2;
		}

		if (rb_p.v.x > 0)
			rb_p.direction = BodyDirection.RIGHT;
		else if (rb_p.v.x < 0)
			rb_p.direction = BodyDirection.LEFT;
		if (rb_e.v.x > 0)
			rb_e.direction = BodyDirection.RIGHT;
		else if (rb_e.v.x < 0)
			rb_e.direction = BodyDirection.LEFT;

		if (ip.impactEdgeLine != null) {
			rb_p.lastImpactEdge.setLine(ip.impactEdgeLine.x1, ip.impactEdgeLine.y1, ip.impactEdgeLine.x2,
					ip.impactEdgeLine.y2);
		}
	}

	private boolean rbIsUpside(RigidBody r1, Line2D.Double impactEdge) {
		if (r1.r.y > impactEdge.y1 || r1.r.y > impactEdge.y2)
			return true;
		return false;
	}

	private double getTorque(RigidBody rb, Vector2D ip) {
		System.out.println("++++++++++++++++++");
		rb.r.y += 0.1;
		double dist = Math.abs(rb.r.x - ip.x);
		return dist * rb.m * rb.g / 20;
	}

	private boolean linesAreParallel(Impactpoint ip, RigidBody rb) {
		Polygon polygonShape = (Polygon) rb.shape;
		Point2D.Double[] vertices = verticesToInertialSystem(polygonShape.vertices, rb.phi, rb.r);

		Point2D.Double vertexPre = getPreVertex(vertices, ip.impactPoint.x, ip.impactPoint.y);
		Point2D.Double vertexNext = getNextVertex(vertices, ip.impactPoint.x, ip.impactPoint.y);

		Vector2D vecPre = new Vector2D(vertexPre.x, vertexPre.y);
		Vector2D vecNext = new Vector2D(vertexNext.x, vertexPre.y);

		Vector2D edgeToPre = VectorMath.sub(ip.impactPoint, vecPre);
		Vector2D edgeToNext = VectorMath.sub(ip.impactPoint, vecNext);
//		System.out.println("PREEEEEE " + edgeToPre);

//		double edgeAngle = VectorMath.angle(new Vector2D(1, 0), ip.impactEdge);
		double angleEdgeToPre = Math.toDegrees(VectorMath.angle(ip.impactEdge, edgeToPre));
		double angleEdgeToNext = Math.toDegrees(VectorMath.angle(ip.impactEdge, edgeToNext));

//		System.out.println("AngelPre " + angleEdgeToPre);
//		System.out.println("AngelNext " + angleEdgeToNext);
		double delta = 2;
		if (angleEdgeToPre >= 0 && angleEdgeToPre <= delta
				|| angleEdgeToPre >= 180 - delta && angleEdgeToPre <= 180 + delta) {
			rb_p.slidingEdge = (edgeToPre);
			return true;
		} else if (angleEdgeToNext >= 0 && angleEdgeToNext <= delta
				|| angleEdgeToNext >= 180 - delta && angleEdgeToNext <= 180 + delta) {
			rb_p.slidingEdge = (edgeToNext);
			return true;
		}
		return false;
	}

	private boolean checkVertexIsRight(RigidBody rigidBody, Vector2D r, double rot) {
		double phi = rb_p.phi + rot;
		RigidBody rb = new RigidBody(rigidBody.shape, r, phi);
		if (Circle.class.isAssignableFrom(rb_p.shape.getClass())) {
			System.out.println(rb.r);
			if (rb.r.x > 0)
				return true;
			else
				return false;
		} else {
			Polygon polygonShape = (Polygon) rb.shape;
			Point2D.Double[] vertices = verticesToInertialSystem(polygonShape.vertices, rb.phi, rb.r);

			Point2D.Double vertexPre = getPreVertex(vertices, 0, 0);
			Point2D.Double vertexNext = getNextVertex(vertices, 0, 0);

			if (vertexPre.x > 0 && vertexNext.x > 0)
				return true;
			else
				return false;
		}
	}

	private Point2D.Double getNextVertex(Point2D.Double[] vertices, double x, double y) {
		for (int i = 0; i < vertices.length; i++) {
			if (Math.round(vertices[i].x * 10) * 10 == x && Math.round(vertices[i].y * 10) * 10 == y) {
				cornerNumber = i;
			} else if (cornerNumber == -1) {
				cornerNumber = i;
			}
		}
		if (cornerNumber == vertices.length - 1) {
			return vertices[0];
		} else {
			return vertices[cornerNumber + 1];
		}
	}

	private Point2D.Double getPreVertex(Point2D.Double[] vertices, double x, double y) {
		for (int i = 0; i < vertices.length; i++) {
			if (Math.round(vertices[i].x * 10) * 10 == x && Math.round(vertices[i].y * 10) * 10 == y) {
				cornerNumber = i;
			} else if (cornerNumber == -1) {
				cornerNumber = i;
			}
		}

		if (cornerNumber == 0) {
			return vertices[vertices.length - 1];
		} else
			return vertices[cornerNumber - 1];
	}

	private Point2D.Double[] verticesToInertialSystem(Vector2D[] vertices, double phi, Vector2D translation) {
		Point2D.Double[] newVertices = new Point2D.Double[vertices.length];
		for (int i = 0; i < vertices.length; i++) {
			newVertices[i] = vertexToInertialSystem(vertices[i], phi, translation);
		}
		return newVertices;
	}

	private Point2D.Double vertexToInertialSystem(Vector2D vertex, double phi, Vector2D translation) {
		double x = translation.x + cos(phi) * vertex.x + sin(phi) * vertex.y;
		double y = translation.y + sin(phi) * vertex.x - cos(phi) * vertex.y;
		return new Point2D.Double(x, y);
	}

	private void startSliding(RigidBody rb, Vector2D collisionEdge) {
		System.out.println("Start Sliding");

		rb.state = BodyState.SLIDING;
		rb.omega = 0;
		rb.alpha = 0;
//		rb.r.y += 0.1;

		// Plane
//		System.out.println(collisionEdge);
		if (collisionEdge.y == 0) {
			rb.Fr.x = -9.81 * rb.m * friction * (signum(rb.v.x));
			rb.v.y = 0;
			rb.a.set(rb.Fr.x / rb.m, 0);
		} else {
			// Incline
//			System.out.println("pos " + rb.r);
			System.out.println("Incline");
//			rb.v.y = 10;

			rb.Fg.set(0, rb.m * rb.g);
			double angle = VectorMath.angle(collisionEdge, new Vector2D(1, 0));

			if (collisionEdge.x > 0 && collisionEdge.y > 0)
				rb.Fh.set(rotateVector2D(rb.Fg, angle + Math.PI / 2));
			else if (collisionEdge.x > 0 && collisionEdge.y < 0)
				rb.Fh.set(rotateVector2D(rb.Fg, -angle + Math.PI / 2));

			Vector2D FhN = VectorMath.normalize(rb.Fh);
			double FgA = VectorMath.abs(rb.Fg);
			double FgAsin = FgA * sin(angle);
			rb.Fh.set(VectorMath.mult(FgAsin, FhN));
			if (collisionEdge.y > 0)
				rb.Fh.mult(-1);

			rb.Fn.set(rotateVector2D(rb.Fg, -angle));
			Vector2D FnN = VectorMath.normalize(rb.Fn);
			double FgAcos = FgA * cos(angle);
			rb.Fn.set(VectorMath.mult(FgAcos, FnN));

			double FnA = VectorMath.abs(rb.Fn);
			double Frx = FnA * cos(angle) * friction * (-signum(rb.Fh.x));
			double Fry = FnA * sin(angle) * friction * (-signum(rb.Fh.y));

			rb.Fr.set(Frx, Fry);
//			System.out.println("Fr " + rb.Fr.abs());
//			System.out.println("Fh " + rb.Fh.abs());

			if (rb.Fr.abs() < rb.Fh.abs()) {
				rb.Fres.set(VectorMath.sub(rb.Fh, rb.Fr));
				rb.a.x = rb.Fh.x / rb.m;
				rb.a.y = rb.Fh.y / rb.m;
			} else {
				rb.state = BodyState.STOPPED;
			}

			Vector2D vr = new Vector2D(rb.v.x, 0);
			double helper = -VectorMath.angle(vr, rb.a);

			if (rb.v.x < 0)
				helper = -helper;

			if (rb.a.x < 0) {
				vr = rotateVector2D(vr, helper);
			} else {
				vr = rotateVector2D(vr, helper);
			}
			if (rb.v.x < 0 && vr.x > 0 || rb.v.x > 0 && vr.x < 0) {
				vr.invert();
			}

			rb.v.set(vr);

		}

		if (Circle.class.isAssignableFrom(rb_p.shape.getClass())) {
			rb.alpha = rb.a.abs() / rb.shape.getRadius();
			if (rb.v.x > 0)
				rb.alpha = -rb.alpha;
		}

		rb.groundSlidingEdge = ip.impactEdgeLine;

	}

//	private void startRolling(RigidBody rb, Vector2D collisionEdge) {
//		rb.state = BodyState.ROLLING;
//		rb.r.y += 0.0001;
//
////		System.out.println("start Rolling");
//
//		// Plane
//		if (collisionEdge.y == 0) {
//			rb.Fr.x = -9.81 * rb.m * friction * (signum(rb.v.x));
//			rb.v.y = 0;
//			rb.a.set(rb.Fr.x / rb.m, 0);
//
//		} else {
//			// Incline
//			rb.Fg.set(0, rb.m * -9.81);
//
//			double angle = VectorMath.angle(collisionEdge, new Vector2D(1, 0));
//
//			if (collisionEdge.x > 0 && collisionEdge.y > 0)
//				rb.Fh.set(rotateVector2D(rb.Fg, toRadians(-90) + angle));
//			else if (collisionEdge.x > 0 && collisionEdge.y < 0)
//				rb.Fh.set(rotateVector2D(rb.Fg, toRadians(-90) - angle));
//
//			Vector2D FhN = VectorMath.normalize(rb.Fh);
//			double FgA = VectorMath.abs(rb.Fg);
//			double FgAsin = FgA * sin(angle);
//			rb.Fh.set(VectorMath.mult(FgAsin, FhN));
//			if (collisionEdge.y < 0)
//				rb.Fh.mult(-1);
//
//			rb.Fn.set(rotateVector2D(rb.Fg, -angle));
//			Vector2D FnN = VectorMath.normalize(rb.Fn);
//			double FgAcos = FgA * cos(angle);
//			rb.Fn.set(VectorMath.mult(FgAcos, FnN));
//
//			double FnA = VectorMath.abs(rb.Fn);
//			double Frx = FnA * cos(angle) * friction * (-signum(rb.Fh.x));
//			double Fry = FnA * sin(angle) * friction * (-signum(rb.Fh.y));
//
//			rb.Fr.set(Frx, Fry);
//
//			rb.Fres.set(VectorMath.sub(rb.Fh, rb.Fr));
//
//			rb.a.x = rb.Fh.x / rb.m;
//			rb.a.y = rb.Fh.y / rb.m;
//
//			if (rb.Fr.abs() < rb.Fh.abs()) {
//
//				rb.Fres.set(VectorMath.sub(rb.Fh, rb.Fr));
//
//				rb.a.x = rb.Fh.x / rb.m;
//				rb.a.y = rb.Fh.y / rb.m;
//			} else {
//				rb.state = BodyState.STOPPED;
//			}
//
//			rb.alpha = rb.a.abs() / rb.shape.getRadius();
//			if (rb.v.x > 0)
//				rb.alpha = -rb.alpha;
//
//			double helper = VectorMath.angle(collisionEdge, rb.v);
//			Vector2D vr = rotateVector2D(rb.v, helper);
//			System.out.println(VectorMath.angle(vr, rb.a));
//			vr = VectorMath.mult(-1, vr);
//			rb.v.set(vr);
//
//		}
//	}

	private Vector2D rotateVector2D(Vector2D r, double rot) {
		return new Vector2D(r.x * cos(rot) - r.y * sin(rot), r.x * sin(rot) + r.y * cos(rot));
	}

	private double impulseFx(double v1x, double v2x, double a1, double a2) {
		double a1omega1 = a1 * rb_p.omega;
		double a2omega2 = a2 * rb_e.omega;
		double zaehler = v1x - v2x - a1omega1 + a2omega2;
		double nenner = (1 / rb_p.m) + (1 / rb_e.m) + (a1 * a1 / rb_p.I) + (a2 * a2 / rb_e.I);
		double Fx = (1 + k) * (zaehler / nenner);
		return Fx;
	}
}
