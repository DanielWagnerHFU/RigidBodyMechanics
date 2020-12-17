package mechanics.tvg;

import static java.lang.Math.*;

import java.util.ArrayList;

import mechanics.rb2d.RigidBody;
import de.physolator.usr.*;
import de.physolator.usr.components.*;
import de.physolator.usr.tvg.*;
import de.physolator.usr.util.*;
import de.physolator.usr.util.parameter.*;
import static de.physolator.usr.components.VectorMath.*;

public class MechanicsTVG extends TVG {

	public PhysicalSystem physicalSystem;
	public Recorder recorder;
	public Structure structure;
	public ArrayList<PM2D> pms = new ArrayList<PM2D>();
	public ArrayList<SP2D> sps = new ArrayList<SP2D>();
	private ArrayList<DrawableComponent2D> dc2dl = new ArrayList<DrawableComponent2D>();

	public MechanicsTVG(PhysicalSystem ps, Structure structure, Recorder recorder) {
		this.physicalSystem = ps;
		this.recorder = recorder;
		this.structure = structure;
		scalesStyle.visible = true;
		geometry.setRim(40, 40, 40, 40);
		geometry.setUserArea(-2, 2, -1.5, 1.5);
		geometry.useFixedXYRatio(1);
		geometry.useExtendedUserArea = true;
		geometry.adoptFontSizeScalingFactorFromOS = true;
		scalesStyle.x.showGridLines = true;
		scalesStyle.y.showGridLines = true;
		scalesStyle.x.showMinorGridLines = true;
		scalesStyle.y.showMinorGridLines = true;
		for (StructureElement s : structure.getSubstructures()) {
			initPM2D(s);
			initSP2D(s);
		}
		for (StructureElement s : structure.getSubstructures())
			if (s.getType().isAssignableFrom(RigidBody.class)) {
				dc2dl.add(new RigidBodyDC2D(this, s.getName(), (RigidBody) s.getObject()));
			}
	}

	private StructureElement sub(StructureElement se, String suffix) {
		String s = se.getTotalName();
		if (s.length() > 0)
			s = s + ".";
		return structure.getStructureElement(s + suffix);
	}

	private double subDoubleValue(StructureElement se, String suffix, double defaultValue) {
		StructureElement s = sub(se, suffix);
		double x = defaultValue;
		try {
			x = s.getField().getDouble(s.getOuterObject());
		} catch (Exception e) {
		}
		return x;
	}

	public void initPM2D(StructureElement se) {
		if (se.isArray())
			return;
		StructureElement r = sub(se, "r");
		StructureElement v = sub(se, "v");
		StructureElement a = sub(se, "a");
		StructureElement Fh = sub(se, "Fh");
		StructureElement Fr = sub(se, "Fr");
		if (r == null || v == null || a == null || Fh == null || Fr == null)
			return;
		double m = subDoubleValue(se, "m", 0);
		double radius = subDoubleValue(se, "radius", 0);
		pms.add(new PM2D(se.getObject(), sub(r, "x"), sub(r, "y"), sub(v, "x"), sub(v, "y"), sub(a, "x"), sub(a, "y"),
				sub(Fh, "x"), sub(Fh, "y"), sub(Fr, "x"), sub(Fr, "y"), m, radius, se.getTotalName()));
	}

	public void initSP2D(StructureElement se) {
		if (se.isArray())
			return;
		CrossrefStructure crs[] = se.getCrossrefs();
		StructureElement r1 = null;
		StructureElement r2 = null;
		for (CrossrefStructure cr : crs) {
			if (cr.localName.equals("r1"))
				r1 = cr.structureElement;
			if (cr.localName.equals("r2"))
				r2 = cr.structureElement;
		}
		StructureElement F = sub(se, "F");
		StructureElement broken = sub(se, "broken");
		if (r1 == null || r2 == null || F == null)
			return;
		sps.add(new SP2D(sub(r1, "x"), sub(r1, "y"), sub(r2, "x"), sub(r2, "y"), sub(F, "x"), sub(F, "y"), broken,
				se.getTotalName()));
	}

	public void addPointMass(String x, String y, String vx, String vy, String ax, String ay, String name) {
		pms.add(new PM2D(structure, structure.getStructureElement(x), structure.getStructureElement(y),
				structure.getStructureElement(vx), structure.getStructureElement(vy), structure.getStructureElement(ax),
				structure.getStructureElement(ay), 0, 0, name));
	}

	public void addPointMass(String x, String y, String vx, String vy, String ax, String ay) {
		addPointMass(x, y, vx, vy, ax, ay, "");
	}

	// Parameters

	@Parameter(path = "mechanics")
	public boolean showLabels = true;

	@Parameter(path = "mechanics")
	public boolean showPath = false;

	@Parameter(path = "mechanics.translational movement")
	@Slider(min = 1, max = 3000, step = 1)
	public int maximalPathLength = 1000;

	public boolean enabledPathLength() {
		return showPath;
	}

	@Parameter(path = "mechanics.translational movement")
	public boolean showVelocity = true;

	@Parameter(path = "mechanics.translational movement")
	@Slider(min = 1, max = 30, step = 0.2)
	public double velocityScaling = 0.2;

	public boolean enabledVelocityScaling() {
		return showVelocity;
	}

	@Parameter(path = "mechanics.translational movement")
	public boolean showAcceleration = true;

	@Parameter(path = "mechanics.translational movement")
	@Slider(min = 1, max = 30, step = 0.2)
	public double accelerationScaling = 0.2;

	public boolean enabledAccelerationScaling() {
		return showAcceleration;
	}

	@Parameter(path = "mechanics.translational movement")
	public boolean showFh = true;

	@Parameter(path = "mechanics.translational movement")
	@Slider(min = 1, max = 30, step = 0.2)
	public double FhScaling = 0.2;

	public boolean enabledFhScaling() {
		return showFh;
	}

	@Parameter(path = "mechanics.translational movement")
	public boolean showFr = true;

	@Parameter(path = "mechanics.translational movement")
	@Slider(min = 1, max = 30, step = 0.2)
	public double FrScaling = 0.2;

	public boolean enabledFrScaling() {
		return showFr;
	}

	@Parameter(path = "mechanics.translational movement")
	@Slider(min = 20, max = 30)
	public double minArrowLength = 25;

	@Parameter(path = "mechanics.translational movement")
	@Slider(min = 20, max = 400)
	public double maxArrowLength = 150;

	@Parameter(path = "mechanics.rotational movement")
	public boolean showAngularDisplacement = true;

	@Parameter(path = "mechanics.rotational movement")
	public boolean showAngularVelocity = true;

	@Parameter(path = "mechanics.rotational movement")
	public double angularVelocityScaling = 1;

	@Parameter(path = "mechanics.rotational movement")
	public boolean showAngularAcceleration = true;

	@Parameter(path = "mechanics.rotational movement")
	public double angularAccelerationScaling = 1;

	@Parameter(path = "mechanics.rotational movement")
	@Slider(min = 20, max = 70)
	public double rotationDisplayRadius = 35;

	// Painting

	public void beginTransformationToPointPX(Vector2D p) {
		double f = geometry.scaleX;
		beginTransformation((x, y) -> new Vector2D(p.x + x * f, p.y + y * f));
	}

	public void drawSpring(double x1, double y1, double x2, double y2) {
		pushStyle();
		style.strokeColor = Colors.grey50;
		drawLine(x1, y1, x2, y2);
		pullStyle();
	}

	public void drawSpringEndPoint(double x, double y, int c) {
		pushStyle();
		style.strokeColor = Colors.black;
		style.fillColor = c;
		style.antialiasing = true;
		double d = 3.5 * geometry.scaleX;
		drawEllipse(x, y, d, d, Shape.POLYGON_LINE_LOOP);
		pullStyle();
	}

	public void drawPointMass(double x, double y, double radius, Object o) {
		style.strokeColor = standardStrokeColor;
		style.strokeWidth = 2;
		style.fillColor = Color.mixColors(style.strokeColor, Colors.white, 0.4);
		drawCircle(x, y, radius, Shape.POLYGON_LINE_LOOP);
	}

	private void drawArrowHead(Vector2D from, Vector2D to, ArrowHead ah) {
		double arrowWidth = 10;
		double arrowLength = 15;
		double arrowFootshift = 4;
		double doubleArrowWidth = 14;
		double doubleArrowLength = 7;
		double doubleArrowDist = 5;
		double aw2 = 0.5 * arrowWidth;
		Vector2D shift = VectorMath.sub(to, from);
		Vector2D direction = normalize(shift);
		double angle = Math.atan2(direction.y, direction.x);
		direction.normalize();
		if (ah == ArrowHead.TRIANGLE) {
			beginTransformation(1, angle, to);
			beginShape(Shape.POLYGON_LINE_LOOP);
			vertex(0, 0);
			vertex(-arrowLength, aw2);
			vertex(-arrowLength + arrowFootshift, 0);
			vertex(-arrowLength, -aw2);
			endShape();
			endTransformation();
		}
		if (ah == ArrowHead.DOUBLE) {
			for (double offsetx : new double[] { 0, -doubleArrowDist }) {
				beginTransformation(1, angle, to);
				beginShape(Shape.LINE_STRIP);
				vertex(offsetx - doubleArrowLength, 0.5 * doubleArrowWidth);
				vertex(offsetx, 0);
				vertex(offsetx - doubleArrowLength, -0.5 * doubleArrowWidth);
				endShape();
				endTransformation();
			}
		}
	}

	public void drawArrow(Vector2D r, Vector2D v, double f, int color, ArrowHead ah) {
		style.strokeColor = standardStrokeColor;
		final Vector2D zero2D = new Vector2D();
		beginTransformationToPointPX(r);
		double length = v.abs() * f;
		double drawingLength = max(min(length, maxArrowLength), minArrowLength);
		Vector2D direction = normalize(v);
		if (length > minArrowLength)
			drawLine(zero2D, mult(drawingLength, direction));
		drawArrowHead(zero2D, mult(drawingLength, direction), ah);
		endTransformation();
	}

	public void drawRotationBaseLine(Vector2D r, double radius) {
		beginTransformationToPointPX(r);
		drawLine(0, 0, 0, radius);
		endTransformation();
	}

	private static final double TWOPI = 2 * PI;

	private static double normalizeAngle(double angle) {
		if (angle < 0) {
			int n = (int) (-angle / (TWOPI));
			return angle + (n + 1) * TWOPI;
		} else {
			int n = (int) (angle / (TWOPI));
			return angle - n * (TWOPI);
		}
	}

	public void drawPie(Vector2D r, double angle, double radius) {
		beginTransformationToPointPX(r);
		double angleN = normalizeAngle(angle);
		for (double d : new double[] { 0, 1 }) {
			beginShape(Shape.POLYGON);
			vertexesFromCurve(alpha -> -radius * sin(alpha), alpha -> radius * cos(alpha), 0 + d * 0.5 * angleN,
					0.5 * angleN + d * 0.5 * angleN);
			vertex(0, 0);
			endShape();
		}
		beginShape(Shape.LINE_LOOP);
		vertexesFromCurve(alpha -> -radius * sin(alpha), alpha -> radius * cos(alpha), 0, angleN);
		vertex(0, 0);
		endShape();
		endTransformation();
	}

	public void drawArrowCurved(Vector2D r, double angle, double radius, int color, ArrowHead ah) {
		beginTransformationToPointPX(r);
		drawCurve(alpha -> -radius * sin(alpha), alpha -> radius * cos(alpha), 0, angle);
		Vector2D endPoint = new Vector2D(-radius * sin(angle), radius * cos(angle));
		Vector2D v0 = angle > 0 ? new Vector2D(endPoint.y, -endPoint.x) : new Vector2D(-endPoint.y, endPoint.x);
		drawArrowHead(add(mult(0.7, endPoint), v0), endPoint, ah);
		endTransformation();
	}

	public void drawAngularDisplcementVelocityAcceleration(Vector2D r, double phi, double omega, double alpha) {
		if (showAngularDisplacement) {
			style.strokeColor = standardStrokeColor;
			style.fillColor = standardStrokeColor;
			drawPie(r, phi, 0.25 * rotationDisplayRadius);
		}
		int n = -1;
		final double factor[] = { 0.7, 1 };
		if (showAngularVelocity) {
			n++;
			style.fillColor = standardStrokeColor;
			drawArrowCurved(r, omega * angularVelocityScaling, factor[n] * rotationDisplayRadius, standardStrokeColor,
					ArrowHead.TRIANGLE);
		}
		if (showAngularAcceleration) {
			n++;
			drawArrowCurved(r, alpha * angularAccelerationScaling, factor[n] * rotationDisplayRadius,
					standardStrokeColor, ArrowHead.DOUBLE);
		}
		if (n >= 0) {
			style.strokeWidth = 4;
			drawRotationBaseLine(r, factor[n] * rotationDisplayRadius);
			style.strokeWidth = 2;
		}
	}

	protected double radiusFromMass(double m) {
		return 5;
	}

	public int pathColor = Colors.darkGrey;
	public int standardStrokeColor = Color.mixColors(Colors.royalBlue, Colors.darkGrey, 0.8);
	public int standardFillColor = Color.mixColors(standardStrokeColor, Colors.white, 0.4);
	public int rigidBodyFillColor = 0xffb471ff;
	public int rigidBodyStrokeColor = 0xBB7748ff;

	@Override
	public void paint() {
		beginClipping();
		style.useUCS = true;
		style.font = geometry.getNormalSizeFont();
		int c = recorder.getCurrentPosition();
		int n = min(c, maximalPathLength);
		for (DrawableComponent2D dc : dc2dl) {
			dc.paint();
		}
		for (SP2D sp : sps)
			if (sp.broken == null || !recorder.getBoolean(sp.broken, c)) {
				double r1x = recorder.getCurrentDouble(sp.r1x);
				double r1y = recorder.getCurrentDouble(sp.r1y);
				double r2x = recorder.getCurrentDouble(sp.r2x);
				double r2y = recorder.getCurrentDouble(sp.r2y);
				drawSpring(r1x, r1y, r2x, r2y);
				drawSpringEndPoint(r1x, r1y, Colors.black);
				drawSpringEndPoint(r2x, r2y, Colors.black);
			}
		for (PM2D p : pms) {
			if (showPath) {
				style.setStrokeColor(pathColor);
				beginShape(Shape.LINE_STRIP);
				for (int i = c - n; i <= c; i++)
					vertex(recorder.getDouble(p.rx, i), recorder.getDouble(p.ry, i));
				endShape();
			}
			double radius = p.radius != 0 ? p.radius : radiusFromMass(p.m) * geometry.scaleX;
			Vector2D r = new Vector2D(recorder.getDouble(p.rx, c), recorder.getDouble(p.ry, c));
			double x = recorder.getDouble(p.rx, c);
			double y = recorder.getDouble(p.ry, c);
			style.fillColor = standardStrokeColor;
			if (showVelocity) {
				Vector2D v = new Vector2D(recorder.getDouble(p.vx, c), recorder.getDouble(p.vy, c));
				drawArrow(r, v, velocityScaling, standardStrokeColor, ArrowHead.TRIANGLE);
			}
			if (showAcceleration) {
				Vector2D a = new Vector2D(recorder.getDouble(p.ax, c), recorder.getDouble(p.ay, c));
				drawArrow(r, a, accelerationScaling, standardStrokeColor, ArrowHead.DOUBLE);
			}
			if (showFh) {
				Vector2D a = new Vector2D(recorder.getDouble(p.Fhx, c), recorder.getDouble(p.Fhy, c));
				drawArrow(r, a, FhScaling, standardStrokeColor, ArrowHead.DOUBLE);
			}
			if (showFr) {
				Vector2D a = new Vector2D(recorder.getDouble(p.Frx, c), recorder.getDouble(p.Fry, c));
				drawArrow(r, a, FrScaling, standardStrokeColor, ArrowHead.DOUBLE);
			}

			style.fillColor = standardFillColor;
			drawPointMass(x, y, radius, p.object);
			if (showLabels)
				drawText(x + radius, y + radius, p.name);
		}
		endClipping();
	}

}