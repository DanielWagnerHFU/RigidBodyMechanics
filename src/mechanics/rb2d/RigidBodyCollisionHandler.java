package mechanics.rb2d;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.components.VectorMath;
import mechanics.rb2d.shapes.Circle;
import mechanics.rb2d.shapes.Polygon;

import static java.lang.Math.*;

public class RigidBodyCollisionHandler implements Runnable {

	private double k = 0.6;
	private RigidBody rb1;
	private RigidBody rb2;
	private Impactpoint ip;
	private boolean showInfo = true;

	private double friction = 0.03;

	public RigidBodyCollisionHandler(RigidBody r1, RigidBody r2, Impactpoint ip) {
		this.rb1 = r1;
		this.rb2 = r2;
		this.ip = ip;
	}

	@Override
	public void run() {
		rb1.state = BodyState.FLYING;
		rb2.state = BodyState.FLYING;
		rb1.Fh.set(0,0);
		rb1.Fn.set(0,0);
		rb1.Fr.set(0,0);
		rb1.Fres.set(0,0);
		rb1.a.set(0,-9.81);
		
		rb2.Fh.set(0,0);
		rb2.Fn.set(0,0);
		rb2.Fr.set(0,0);
		rb2.Fres.set(0,0);
		rb2.a.set(0,-9.81);
		if (showInfo) {
			System.out.println("Collision between:");
			System.out.println(rb1);
			System.out.println(rb2);
		}

		Vector2D p = ip.impactPoint;
		Vector2D collisionEdge = VectorMath.normalize(ip.impactEdge);

		// 1. Transformation in das Stoßkoordinatensystem
		Vector2D r1m = VectorMath.sub(rb1.r, p);
		Vector2D r2m = VectorMath.sub(rb2.r, p);

		double rot = VectorMath.angle(new Vector2D(0, 1), collisionEdge);
		if (collisionEdge.x < 0)
			rot = -rot;

		Vector2D r1mr = rotateVector2D(r1m, rot);
		Vector2D r2mr = rotateVector2D(r2m, rot);

		Vector2D v1r = rotateVector2D(rb1.v, rot);
		Vector2D v2r = rotateVector2D(rb2.v, rot);

		// Zustandsbestimmung nach Stoss
		if ((Math.abs(v1r.x) + (Math.abs(v2r.x)) < 0.001)) {
			rb1.state = BodyState.STOPPED;
			rb2.state = BodyState.STOPPED;
			if (this.showInfo)
				System.out.println("Stopped: " + "v1r.x=" + v1r.x + ", v2r.x=" + v2r.x);
		}

		// 2. Berechnung der neuen Größen im Stoßkoordinatensystem
		double a1 = -r1mr.y;
		double a2 = -r2mr.y;

		double Fx = impulseFx(v1r.x, v2r.x, a1, a2);

		Vector2D V1r = new Vector2D(v1r.x - (Fx / rb1.m), v1r.y);
		Vector2D V2r = new Vector2D(v2r.x + (Fx / rb2.m), v2r.y);

		double Omega1 = rb1.omega + ((a1 * Fx) / rb1.I);
		double Omega2 = rb2.omega - ((a2 * Fx) / rb2.I);

		// 3. Ruecktransformation ins Inertialsystem
		Vector2D V1r_ = rotateVector2D(V1r, -rot);
		Vector2D V2r_ = rotateVector2D(V2r, -rot);

		// 4. Setzen der neuen Werte

		// Stosskoordinatensystem
		// rb1.v.set(V1r);
		// rb2.v.set(V2r);
		//
		// rb1.omega = Omega1;
		// rb2.omega = Omega2;
		//
		// rb1.phi = phi1r;
		// rb2.phi = phi2r;
		//
		// rb1.r.set(r1mr);
		// rb2.r.set(r2mr);

		// Normale Koordinatensystem
		rb1.v.set(V1r_);
		rb2.v.set(V2r_);

		rb1.omega = Omega1;
		rb2.omega = Omega2;
		
		if(rb1.v.x > 0)
			rb1.direction = BodyDirection.RIGHT;
		else if(rb1.v.x < 0)
			rb1.direction = BodyDirection.LEFT;
		if(rb2.v.x > 0)
			rb2.direction = BodyDirection.RIGHT;
		else if(rb2.v.x < 0)
			rb2.direction = BodyDirection.LEFT;

		if ((Math.abs(v1r.x) + (Math.abs(v2r.x)) < 0.001)) {
			if (Circle.class.isAssignableFrom(rb1.shape.getClass())) {
				startRolling(rb1, collisionEdge);
			}
			if (Circle.class.isAssignableFrom(rb2.shape.getClass())) {
				startRolling(rb2, collisionEdge);
			}
		}

	}

	private void startRolling(RigidBody rb, Vector2D collisionEdge) {
		rb.state = BodyState.ROLLING;
		rb.r.y += 0.0001;

		//Plane
		if (collisionEdge.y == 0) {
			rb.Fr.x = -9.81 * rb.m * friction *(signum(rb.v.x));
			rb.v.y = 0;
			rb.a.set(rb.Fr.x / rb.m, 0);
			
		} else {
			//Incline
			rb.Fg.set(0, rb.m * -9.81);

			double angle = VectorMath.angle(collisionEdge, new Vector2D(1, 0));

			if (collisionEdge.x > 0 && collisionEdge.y > 0)
				rb.Fh.set(rotateVector2D(rb.Fg, toRadians(-90) + angle));
			else if (collisionEdge.x > 0 && collisionEdge.y < 0)
				rb.Fh.set(rotateVector2D(rb.Fg, toRadians(-90) - angle));

			Vector2D FhN = VectorMath.normalize(rb.Fh);
			double FgA = VectorMath.abs(rb.Fg);
			double FgAsin = FgA * sin(angle);
			rb.Fh.set(VectorMath.mult(FgAsin, FhN));
			if(collisionEdge.y<0)
				rb.Fh.mult(-1);

			rb.Fn.set(rotateVector2D(rb.Fg, -angle));
			Vector2D FnN = VectorMath.normalize(rb.Fn);
			double FgAcos = FgA * cos(angle);
			rb.Fn.set(VectorMath.mult(FgAcos, FnN));

			double FnA = VectorMath.abs(rb.Fn);
			double Frx = FnA * cos(angle) * friction * (-signum(rb.Fh.x));
			double Fry = FnA * sin(angle) * friction * (-signum(rb.Fh.y));
			
			rb.Fr.set(Frx, Fry);
			
			rb.Fres.set(VectorMath.sub(rb.Fh, rb.Fr));

			rb.a.x = rb.Fh.x / rb.m;
			rb.a.y = rb.Fh.y / rb.m;
			
			rb.alpha = rb.a.abs() / rb.shape.getRadius();
			if (rb.v.x > 0)
				rb.alpha = -rb.alpha;
		}
		
	}

	private Vector2D rotateVector2D(Vector2D r, double rot) {
		return new Vector2D(r.x * cos(rot) - r.y * sin(rot), r.x * sin(rot) + r.y * cos(rot));
	}

	private double impulseFx(double v1x, double v2x, double a1, double a2) {
		double a1omega1 = a1 * rb1.omega;
		double a2omega2 = a2 * rb2.omega;
		double zaehler = v1x - v2x - a1omega1 + a2omega2;
		double nenner = (1 / rb1.m) + (1 / rb2.m) + (a1 * a1 / rb1.I) + (a2 * a2 / rb2.I);
		double Fx = (1 + k) * (zaehler / nenner);
		return Fx;
	}
}
