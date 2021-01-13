package mechanics.rb2d;

import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import de.physolator.usr.*;
import de.physolator.usr.components.Vector2D;
import de.physolator.usr.components.VectorMath;
import de.physolator.usr.util.Color;
import geometry.Polygon2D;
import mechanics.rb2d.misc.BodyDirection;
import mechanics.rb2d.misc.BodyState;
import mechanics.rb2d.shapes.AbstractShape;
import mechanics.rb2d.shapes.Circle;
import mechanics.rb2d.shapes.Polygon;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

public class RigidBody {

	@Ignore
	private static int count;
	@Ignore
	public int uid;

	@V(unit = "kg")
	public double m;
	@V(unit = "m", derivative = "v")
	public Vector2D r;
	@V(unit = "m/s", derivative = "a")
	public Vector2D v;
	@V(unit = "m/s^2")
	public Vector2D a;

	@V(unit = "kg m^2")
	public double I;
	@V(unit = "rad", derivative = "omega")
	public double phi;
	@V(unit = "rad / s", derivative = "alpha")
	public double omega;
	@V(unit = "rad / s^2")
	public double alpha;

	@V(unit = "m/s^2")
	public double g = -9.81;

	@V(unit = "kg*m/s")
	public double E_kin;
	@V(unit = "kg*m/s")
	public double E_rot;
	@V(unit = "kg*m/s")
	public double E_rb;

	@V(unit = "N")
	public Vector2D Fg = new Vector2D();
	@V(unit = "N")
	public Vector2D Fh = new Vector2D();
	@V(unit = "N")
	public Vector2D Fr = new Vector2D();
	@V(unit = "N")
	public Vector2D Fn = new Vector2D();
	@V(unit = "N")
	public Vector2D Fres = new Vector2D();
	@V(unit = "N")
	public Vector2D Fhr = new Vector2D();

	@Ignore
	public int color = Color.make(0.5, 0.5, 0.5);

	public AbstractShape shape;

	public BodyState state;
	@Ignore
	public BodyDirection direction;
	@Ignore
	public boolean visible = true;
	@Ignore
	public double t_before = 0;
	@Ignore
	public RigidBody impactPartner_before;
	@Ignore
	public boolean dynamic;
	@Ignore
	public Line2D.Double lastImpactEdge;
	@Ignore
	public Vector2D slidingEdge;
	@Ignore
	public Line2D.Double groundSlidingEdge;

	public RigidBody(double m, Vector2D r, Vector2D v, Vector2D a, double I, double phi, double omega, double alpha,
			AbstractShape shape) {
		this(m, r, v, a, I, phi, omega, alpha, true, shape);
	}

	public RigidBody(double m, Vector2D r, Vector2D v, Vector2D a, double I, double phi, double omega, double alpha,
			boolean dynamic, AbstractShape shape) {
		count++;
		this.uid = count;
		this.m = m;
		this.r = r;
		this.v = v;
		this.a = a;
		this.I = I;
		this.phi = phi;
		this.omega = omega;
		this.alpha = alpha;
		this.shape = shape;
		this.state = BodyState.FLYING;
		this.dynamic = dynamic;
		this.lastImpactEdge = new Line2D.Double(Double.MIN_VALUE, Double.MIN_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
	}

	public RigidBody(AbstractShape shape, double m, Vector2D r, Vector2D v, Vector2D a, double phi, double omega,
			double alpha) {
		this(m, r, v, a, shape.getI(m), phi, omega, alpha, true, shape);
	}

	public RigidBody(AbstractShape shape, Vector2D r, double phi) {
		this.shape = shape;
		this.r = r;
		this.phi = phi;
	}

	public void f(double t, double dt) {
		E_kin = 0.5 * m * v.abs() * v.abs();
		E_rot = 0.5 * I * omega * omega;
		E_rb = E_kin + E_rot;
		if (!dynamic) {
			a.set(0, 0);
		}

		if (state == BodyState.STOPPED) {
			v.set(0, 0);
			a.set(0, 0);
			omega = 0;
			alpha = 0;
			direction = BodyDirection.NONE;
		}

		if (state == BodyState.SLIDING)
			omega = 0;
		if (state == BodyState.ROLLING || state == BodyState.SLIDING) {
			Fr.x = Math.abs(Fr.x) * signum(-v.x);
			Fr.y = Math.abs(Fr.y) * signum(-v.y);
//			v.y = 0;

			if (direction == BodyDirection.LEFT && state == BodyState.ROLLING) {
				omega = v.abs() / shape.getRadius();
			} else if (direction == BodyDirection.RIGHT && state == BodyState.ROLLING) {
				omega = -v.abs() / shape.getRadius();
			}

			if (lastImpactEdge.y1 == lastImpactEdge.y2)
				v.y = 0;

			// STOP PLANE
			if (direction == BodyDirection.LEFT && v.x > 0 || direction == BodyDirection.RIGHT && v.x < 0) {
				if (Fh.abs() == 0)
					state = BodyState.STOPPED;
			}
			if (v.x > 0)
				direction = BodyDirection.RIGHT;
			else if (v.x < 0)
				direction = BodyDirection.LEFT;
		}
	}

	public void collisionWithRigidBodyCheck(AfterEventDescription aed, RigidBody r2, double t,
			RigidBody[] rigidBodies) {

		if (this.in(r2)) {
			Runnable handler = new RigidBodyCollisionHandler(impactpoint(r2));
			aed.reportEvent(handler, "collision of rigidbodies: ", this.r.toString(), r2.r.toString());
		} else if (this.state == BodyState.ROLLING || this.state == BodyState.SLIDING)
			startFalling(this, impactpoint(r2), aed);
		else if (r2.state == BodyState.ROLLING || r2.state == BodyState.SLIDING)
			startFalling(r2, impactpoint(this), aed);
	}

	public void startFalling(RigidBody rb, Impactpoint ip, AfterEventDescription aed) {
		System.out.println("Ground sliding " + rb.lastImpactEdge.x1 + " " + rb.lastImpactEdge.y1 + " "
				+ rb.lastImpactEdge.x2 + " " + rb.lastImpactEdge.y2);
		double delta = 0;
		if (Polygon.class.isAssignableFrom(this.shape.getClass()))
			delta = 0.5;
		else if (Polygon.class.isAssignableFrom(this.shape.getClass()))
			delta = 0.5;
		if (rb.lastImpactEdge.x1 != rb.lastImpactEdge.x2) {
			if (rb.r.x - delta > rb.lastImpactEdge.x1 && rb.r.x - delta > rb.lastImpactEdge.x2) {
				Runnable handler = new RigidBodyStartsFallingHandler(rb, ip.impactEdgeLine);
				aed.reportEvent(handler, "Circle Starts falling ", rb.r.toString());
			} else if (rb.r.x + delta < rb.lastImpactEdge.x1 && rb.r.x + delta < rb.lastImpactEdge.x2) {
				Runnable handler = new RigidBodyStartsFallingHandler(rb, ip.impactEdgeLine);
				aed.reportEvent(handler, "Circle Starts falling ", rb.r.toString());
			}
		}

	}

	public Impactpoint impactpoint(RigidBody r2) {
		if (Polygon.class.isAssignableFrom(this.shape.getClass())
				&& Polygon.class.isAssignableFrom(r2.shape.getClass())) {
			RigidBody r1 = this;

			Polygon polygonShape_r1 = (Polygon) this.shape;
			Polygon polygonShape_r2 = (Polygon) r2.shape;

			Point2D.Double[] vertices1 = verticesToInertialSystem(polygonShape_r1.vertices, r1.phi, r1.r);
			Point2D.Double[] vertices2 = verticesToInertialSystem(polygonShape_r2.vertices, r2.phi, r2.r);

			Line2D.Double[] edges1 = getEdges(vertices1);
			Line2D.Double[] edges2 = getEdges(vertices2);

			Point2D.Double impactpoint = vertices1[0];
			Line2D.Double impactedge = edges2[0];

			double smallestDistance = Double.MAX_VALUE;
			for (int i = 0; i < edges1.length; i++) {
				for (int j = 0; j < vertices2.length; j++) {
					double distance = edges1[i].ptSegDist(vertices2[j]);
					if (distance < smallestDistance) {
						smallestDistance = distance;
						impactpoint = vertices2[j];
						impactedge = edges1[i];
					}
				}
			}

			double lastSmallestDistance = smallestDistance;

			for (int i = 0; i < edges2.length; i++) {
				for (int j = 0; j < vertices1.length; j++) {
					double distance = edges2[i].ptSegDist(vertices1[j]);
					if (distance < smallestDistance) {
						smallestDistance = distance;
						impactpoint = vertices1[j];
						impactedge = edges2[i];

					}
				}
			}

			Vector2D p = new Vector2D(impactpoint.x, impactpoint.y);
			Vector2D e = new Vector2D(impactedge.x2 - impactedge.x1, impactedge.y2 - impactedge.y1);
			Line2D.Double e_line = new Line2D.Double(impactedge.x1, impactedge.y1, impactedge.x2, impactedge.y2);

			if (smallestDistance != lastSmallestDistance)
				return new Impactpoint(p, e, this, r2, e_line);
			else
				return new Impactpoint(p, e, r2, this, e_line);

		} else if (Circle.class.isAssignableFrom(this.shape.getClass())
				&& Circle.class.isAssignableFrom(r2.shape.getClass())) {
			Circle circleShape_r1 = (Circle) this.shape;
			Vector2D r1_r2 = VectorMath.sub(r2.r, this.r);
			Vector2D impactEdge = VectorMath.perpendicular(this.r, r1_r2);
			if (this.r.x == r2.r.x)
				impactEdge.set(1, 0);
			if (this.r.y == r2.r.y)
				impactEdge.set(0, 1);

			impactEdge.normalize();
			Vector2D impactPoint = VectorMath.add(this.r,
					(VectorMath.mult(circleShape_r1.radius, VectorMath.normalize(r1_r2))));
			return new Impactpoint(impactPoint, impactEdge, this, r2);
		} else if (Circle.class.isAssignableFrom(this.shape.getClass())
				&& Polygon.class.isAssignableFrom(r2.shape.getClass())) {
			Polygon polygonShape_r2 = (Polygon) r2.shape;

			Point2D.Double[] vertices = verticesToInertialSystem(polygonShape_r2.vertices, r2.phi, r2.r);
			Line2D.Double[] edges = getEdges(vertices);

			double smallestDistanceToLine = Double.MAX_VALUE;
			Line2D.Double impactEdge = edges[0];
			for (Line2D.Double edge : edges) {
				if (edge.ptSegDist(this.r.x, this.r.y) <= smallestDistanceToLine) {
					smallestDistanceToLine = edge.ptSegDist(this.r.x, this.r.y);
					impactEdge = edge;

				}
			}
			for (Point2D.Double vertex : vertices) {
				if (vertex.distance(this.r.x, this.r.y) <= smallestDistanceToLine) {
					// Case: hit vertex
					Vector2D impactPoint = new Vector2D(vertex.x, vertex.y);
					Vector2D impactPoint_circle = VectorMath.sub(this.r, impactPoint);
					Vector2D trueImpactEdge = VectorMath.perpendicular(impactPoint, impactPoint_circle);
					if (this.r.x == r2.r.x)
						trueImpactEdge.set(1, 0);
					if (this.r.y == r2.r.y)
						trueImpactEdge.set(0, 1);
					return new Impactpoint(impactPoint, trueImpactEdge, r2, this, impactEdge);
				}
			}
			// Case: hit edge
			Vector2D trueImpactEdge = new Vector2D(impactEdge.x2 - impactEdge.x1, impactEdge.y2 - impactEdge.y1);
			Vector2D impactPoint = VectorMath.footOfPerpendicular(this.r, new Vector2D(impactEdge.x1, impactEdge.y1),
					trueImpactEdge);
			;
			return new Impactpoint(impactPoint, trueImpactEdge, this, r2, impactEdge);

		} else if (Polygon.class.isAssignableFrom(this.shape.getClass())
				&& Circle.class.isAssignableFrom(r2.shape.getClass())) {
			Polygon polygonShape_r1 = (Polygon) this.shape;

			Point2D.Double[] vertices = verticesToInertialSystem(polygonShape_r1.vertices, this.phi, this.r);
			Line2D.Double[] edges = getEdges(vertices);

			double smallestDistanceToLine = Double.MAX_VALUE;
			Line2D.Double impactEdge = edges[0];
			for (Line2D.Double edge : edges) {
				if (edge.ptSegDist(r2.r.x, r2.r.y) <= smallestDistanceToLine) {
					smallestDistanceToLine = edge.ptSegDist(r2.r.x, r2.r.y);
					impactEdge = edge;
				}
			}
			for (Point2D.Double vertex : vertices) {
				if (vertex.distance(r2.r.x, r2.r.y) <= smallestDistanceToLine) {
					// Case: hit vertex
					Vector2D impactPoint = new Vector2D(vertex.x, vertex.y);
					Vector2D impactPoint_circle = VectorMath.sub(r2.r, impactPoint);
					Vector2D trueImpactEdge = VectorMath.perpendicular(impactPoint, impactPoint_circle);
					if (r2.r.x == this.r.x)
						trueImpactEdge.set(1, 0);
					if (r2.r.y == this.r.y)
						trueImpactEdge.set(0, 1);
					return new Impactpoint(impactPoint, trueImpactEdge, this, r2, impactEdge);
				}
			}
			// Case: hit edge
			Vector2D trueImpactEdge = new Vector2D(impactEdge.x2 - impactEdge.x1, impactEdge.y2 - impactEdge.y1);
			Vector2D impactPoint = VectorMath.footOfPerpendicular(r2.r, new Vector2D(impactEdge.x1, impactEdge.y1),
					trueImpactEdge);
			;
			return new Impactpoint(impactPoint, trueImpactEdge, r2, this, impactEdge);
		}
		return null;
	}

	public boolean in(RigidBody r2) {
		if (Polygon.class.isAssignableFrom(this.shape.getClass())
				&& Polygon.class.isAssignableFrom(r2.shape.getClass())) {
			Polygon polygonShape_r1 = (Polygon) this.shape;
			Polygon polygonShape_r2 = (Polygon) r2.shape;

			Point2D.Double[] vertices1 = verticesToInertialSystem(polygonShape_r1.vertices, this.phi, this.r);
			Point2D.Double[] vertices2 = verticesToInertialSystem(polygonShape_r2.vertices, r2.phi, r2.r);

			Line2D.Double[] edges1 = getEdges(vertices1);
			Line2D.Double[] edges2 = getEdges(vertices2);

			for (Line2D.Double a : edges1)
				for (Line2D.Double b : edges2)
					if (a.intersectsLine(b))
						return true;
			return false;

		} else if (Circle.class.isAssignableFrom(this.shape.getClass())
				&& Circle.class.isAssignableFrom(r2.shape.getClass())) {
			Circle circleShape_r1 = (Circle) this.shape;
			Circle circleShape_r2 = (Circle) r2.shape;
			double distance = VectorMath.sub(this.r, r2.r).abs();
			if ((circleShape_r1.radius + circleShape_r2.radius) >= distance)
				return true;
			else
				return false;

		} else if (Circle.class.isAssignableFrom(this.shape.getClass())
				&& Polygon.class.isAssignableFrom(r2.shape.getClass())) {
			Circle circleShape_r1 = (Circle) this.shape;
			Polygon polygonShape_r2 = (Polygon) r2.shape;

			Point2D.Double[] vertices = verticesToInertialSystem(polygonShape_r2.vertices, r2.phi, r2.r);
			Line2D.Double[] edges = getEdges(vertices);

			double r = circleShape_r1.radius;

			for (Line2D.Double edge : edges)
				if (edge.ptSegDist(this.r.x, this.r.y) <= r)
					return true;
			return false;

		} else if (Polygon.class.isAssignableFrom(this.shape.getClass())
				&& Circle.class.isAssignableFrom(r2.shape.getClass())) {
			Polygon polygonShape_r1 = (Polygon) this.shape;
			Circle circleShape_r2 = (Circle) r2.shape;

			Point2D.Double[] vertices = verticesToInertialSystem(polygonShape_r1.vertices, this.phi, this.r);
			Line2D.Double[] edges = getEdges(vertices);

			double r = circleShape_r2.radius;

			for (Line2D.Double edge : edges)
				if (edge.ptSegDist(r2.r.x, r2.r.y) <= r)
					return true;
			return false;
		}
		return false;
	}

	public boolean extendedIn(RigidBody rb) {
		if (this.in(rb) == true) {
			return true;
		}
		boolean in = false;
		Polygon p1 = (Polygon) this.shape;
		Polygon p2 = (Polygon) rb.shape;
		Point2D.Double[] vertices1 = verticesToInertialSystem(p1.vertices, this.phi, this.r);
		Point2D.Double[] vertices2 = verticesToInertialSystem(p2.vertices, rb.phi, rb.r);
		double[] xPoints1 = new double[vertices1.length];
		double[] yPoints1 = new double[vertices1.length];
		for (int i = 0; i < vertices1.length; i++) {
			xPoints1[i] = vertices1[i].x;
			yPoints1[i] = vertices1[i].y;
		}
		double[] xPoints2 = new double[vertices2.length];
		double[] yPoints2 = new double[vertices2.length];
		for (int i = 0; i < vertices2.length; i++) {
			xPoints2[i] = vertices2[i].x;
			yPoints2[i] = vertices2[i].y;
		}
		Polygon2D rb1 = new Polygon2D(xPoints1, yPoints1, xPoints1.length);
		Polygon2D rb2 = new Polygon2D(xPoints2, yPoints2, xPoints2.length);
		for (Point2D.Double p : vertices1) {
			if (rb2.contains(p))
				return true;
		}
		for (Point2D.Double p : vertices2) {
			if (rb1.contains(p))
				return true;
		}

		return in;
	}

	private Line2D.Double[] getEdges(Point2D.Double[] points) {
		Line2D.Double[] edges = new Line2D.Double[points.length];
		edges[0] = new Line2D.Double(points[points.length - 1], points[0]);
		for (int i = 1; i < points.length; i++) {
			edges[i] = new Line2D.Double(points[i], points[i - 1]);
		}
		return edges;
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

	@Override
	public String toString() {
		String rbText = "(";
		rbText += ("[uid=" + this.uid + "], ");
		rbText += ("[m=" + m + "], ");
		rbText += ("[r=" + r + "], ");
		rbText += ("[v=" + v + "], ");
		rbText += ("[a=" + a + "], ");
		rbText += ("[I=" + m + "], ");
		rbText += ("[phi=" + phi + "], ");
		rbText += ("[omega=" + omega + "], ");
		rbText += ("[alpha=" + alpha + "]");
		return rbText + ")";
	}
}
