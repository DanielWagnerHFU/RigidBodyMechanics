package mechanics.rb2d;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import de.physolator.usr.*;
import de.physolator.usr.components.Vector2D;
import de.physolator.usr.components.VectorMath;
import de.physolator.usr.util.Color;
import mechanics.rb2d.shapes.AbstractShape;
import mechanics.rb2d.shapes.Circle;
import mechanics.rb2d.shapes.Polygon;
import sun.security.util.ArrayUtil;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

public class RigidBody {

	@V(unit = "kg")
	public double m;
	@V(unit = "m", derivative = "v")
	public Vector2D r = new Vector2D();
	@V(unit = "m/s", derivative = "a")
	public Vector2D v = new Vector2D();
	@V(unit = "m/s^2")
	public Vector2D a = new Vector2D();

	@V(unit = "kg m^2")
	public double I;
	@V(unit = "rad", derivative = "omega")
	public double phi;
	@V(unit = "rad / s", derivative = "alpha")
	public double omega;
	@V(unit = "rad / s^2")
	public double alpha;

	@V(unit = "kg*m/s")
	public double E_kin;
	@V(unit = "kg*m/s")
	public double E_rot;
	@V(unit = "kg*m/s")
	public double E_rb;

	@Ignore
	public int color = Color.make(0.5, 0.5, 0.5);
	@Ignore
	public AbstractShape shape;
	@Ignore
	public BodyState state;
	@Ignore
	public boolean visible = true;
	@Ignore
	public double t_before = 0;
	@Ignore
	public RigidBody impactPartner_before;

	public RigidBody(double m, Vector2D r, Vector2D v, Vector2D a, double I, double phi, double omega, double alpha,
			AbstractShape shape) {
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
	}

	public void f(double t, double dt) {
		E_kin = 0.5 * m * v.abs() * v.abs();
		E_rot = 0.5 * I * omega * omega;
		E_rb = E_kin + E_rot;
	}

	public void collisionWithRigidBodyCheck(AfterEventDescription aed, RigidBody r2, double t,
			RigidBody[] rigidBodies) {
		double r1_t_before = Math.round(this.t_before * 10e3) * 10e-3;
		double r2_t_before = Math.round(r2.t_before * 10e3) * 10e-3;
		double t_rounded = Math.round(t * 10e3) * 10e-3;

		if (this == r2.impactPartner_before && r2 == this.impactPartner_before) {
			if ((r1_t_before == 0 || r2_t_before == 0) || (t_rounded != r1_t_before && t_rounded != r2_t_before)) {
//				System.out.println("hier");
				if (this.in(r2)) {
					this.impactPartner_before = r2;
					r2.impactPartner_before = this;
					this.t_before = t;
					r2.t_before = t;
					Runnable handler = new RigidBodyCollisionHandler(this, r2, impactpoint(r2));
					aed.reportEvent(handler, "collision of rigidbodies: ", this.toString(), r2.toString());
				}
			}
		}else if (this.in(r2)) {
			this.impactPartner_before = r2;
			r2.impactPartner_before = this;
			this.t_before = t;
			r2.t_before = t;
			Runnable handler = new RigidBodyCollisionHandler(this, r2, impactpoint(r2));
			aed.reportEvent(handler, "collision of rigidbodies: ", this.toString(), r2.toString());
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
			return new Impactpoint(p, e);
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
			return new Impactpoint(impactPoint, impactEdge);
		} else if (Circle.class.isAssignableFrom(this.shape.getClass())
				&& Polygon.class.isAssignableFrom(r2.shape.getClass())) {
			Circle circleShape_r1 = (Circle) this.shape;
			Polygon polygonShape_r2 = (Polygon) r2.shape;

			Point2D.Double[] vertices = verticesToInertialSystem(polygonShape_r2.vertices, r2.phi, r2.r);
			Line2D.Double[] edges = getEdges(vertices);

			double r = circleShape_r1.radius;

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
					return new Impactpoint(impactPoint, trueImpactEdge);
				}
			}
			// Case: hit edge
			Vector2D trueImpactEdge = new Vector2D(impactEdge.x2 - impactEdge.x1, impactEdge.y2 - impactEdge.y1);
			Vector2D impactPoint = VectorMath.footOfPerpendicular(this.r, new Vector2D(impactEdge.x1, impactEdge.y1),
					trueImpactEdge);
			;
			return new Impactpoint(impactPoint, trueImpactEdge);

		} else if (Polygon.class.isAssignableFrom(this.shape.getClass())
				&& Circle.class.isAssignableFrom(r2.shape.getClass())) {
			Polygon polygonShape_r1 = (Polygon) this.shape;
			Circle circleShape_r2 = (Circle) r2.shape;

			Point2D.Double[] vertices = verticesToInertialSystem(polygonShape_r1.vertices, this.phi, this.r);
			Line2D.Double[] edges = getEdges(vertices);

			double r = circleShape_r2.radius;

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
					return new Impactpoint(impactPoint, trueImpactEdge);
				}
			}
			// Case: hit edge
			Vector2D trueImpactEdge = new Vector2D(impactEdge.x2 - impactEdge.x1, impactEdge.y2 - impactEdge.y1);
			Vector2D impactPoint = VectorMath.footOfPerpendicular(r2.r, new Vector2D(impactEdge.x1, impactEdge.y1),
					trueImpactEdge);
			;
			return new Impactpoint(impactPoint, trueImpactEdge);
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
}
