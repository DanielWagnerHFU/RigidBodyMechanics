package mechanics.rb2d;

import mechanics.rb2d.misc.RandomPolygonBuilder;
import mechanics.rb2d.shapes.AbstractShape;
import mechanics.rb2d.shapes.Circle;
import mechanics.rb2d.shapes.Hexagon;
import mechanics.rb2d.shapes.Pentagon;
import mechanics.rb2d.shapes.Polygon;
import mechanics.rb2d.shapes.Rectangle;
import mechanics.rb2d.shapes.Triangle;
import mechanics.tvg.MechanicsTVG;

import java.util.ArrayList;
import java.util.List;
import Mathematics.MyRandom;
import de.physolator.usr.*;
import de.physolator.usr.components.Vector2D;

public class RigidBodiesPS extends PhysicalSystem {

	public RigidBody[] rigidBodies;

	@V(unit = "kg*m/s")
	public double E_gesamt;

	public RigidBodiesPS() {
		test_sliding_plane();
	}

	private void test_sliding_plane() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-6, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(new Pentagon(1), 1.5, new Vector2D(4,0), new Vector2D(-3, 0),
				new Vector2D(0, -9.81), Math.PI/4, 1, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
		
	}

	private void test_sliding_left() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0.1, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-6, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(new Pentagon(1), 1.5, new Vector2D(4,0), new Vector2D(0, 0),
				new Vector2D(0, -9.81), Math.PI/4, 1, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
	}

	private void test_sliding_right() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, -0.1, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(6, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(new Triangle(1), 1.5, new Vector2D(-5, 1), new Vector2D(2, 0),
				new Vector2D(0, -9.81), Math.PI/4, 1, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
		
	}

	private void test_rolling_incline_right() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, -0.1, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(6, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(new Circle(1), 1.5, new Vector2D(-4, 1), new Vector2D(-1, 0),
				new Vector2D(0, -9.81), 0.6, 0, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
	}
	
	private void test_rolling_incline_left() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0.1, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-6, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(new Circle(1), 1.5, new Vector2D(4, 1), new Vector2D(-1, 0),
				new Vector2D(0, -9.81), 0.6, 0, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
	}
	
	private void test_rolling_plane() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(6, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2, 0, 0, false, new Rectangle(10.5, 1)));
//		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-6, -2), new Vector2D(0, 0), new Vector2D(0, 0),
//				Double.MAX_VALUE, Math.PI / 2, 0, 0, false, new Rectangle(10.5, 1)));
		
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 2.5), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, -0.1, 0, 0, false, new Rectangle(4, 0.2)));
//		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(2.5, 2), new Vector2D(0, 0), new Vector2D(0, 0),
//				Double.MAX_VALUE, -0.2, 0, 0, false, new Rectangle(0.2, 3)));
//		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-4.3, 3), new Vector2D(0, 0), new Vector2D(0, 0),
//				Double.MAX_VALUE, 0.1, 0, 0, false, new Rectangle(2, 0.25)));

		rigidBodies.add(new RigidBody(new Circle(1), 1.5, new Vector2D(-4, 1), new Vector2D(2, 0),
				new Vector2D(0, -9.81), 0.6, 0, 0));
		rigidBodies.add(new RigidBody(new Circle(0.5), 1, new Vector2D(0, 0), new Vector2D(5, 2),
				new Vector2D(0, -9.81), 0.6, 0, 0));
		rigidBodies.add(new RigidBody(new Circle(0.7), 1, new Vector2D(4, 3), new Vector2D(5, 0),
				new Vector2D(0, -9.81), 0.6, 0, 0));
////		
		rigidBodies.add(new RigidBody(new Circle(0.7), 1, new Vector2D(-1, 4), new Vector2D(-1, 0),
				new Vector2D(0, -9.81), 0.6, 0, 0));
//		
//		rigidBodies.add(new RigidBody(new Circle(0.2), 0.2, new Vector2D(-3.6, 4), new Vector2D(0, 0),
//				new Vector2D(0, -9.81), 0.6, 0, 0));
//		

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
	}
	
	private void test_rolling_plane02() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0, 0, 0, false, new Rectangle(10.5, 1)));
		
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 2.5), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0.1, 0, 0, false, new Rectangle(4, 0.2)));
		
		rigidBodies.add(new RigidBody(new Circle(0.7), 1, new Vector2D(-2, 4), new Vector2D(0, 0),
				new Vector2D(0, -9.81), -0.6, 0, 0));
		rigidBodies.add(new RigidBody(new Circle(0.7), 1, new Vector2D(0, 4), new Vector2D(0, 0),
				new Vector2D(0, -9.81), -0.6, 0, 0));
		
		rigidBodies.add(new RigidBody(new Circle(0.7), 1, new Vector2D(7, 0), new Vector2D(-3.6, 0),
				new Vector2D(0, -9.81), -0.6, 0, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
	}

	private void test_tri_penta_hexa() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0.1, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(new Triangle(1), 1, new Vector2D(-4, 0), new Vector2D(), new Vector2D(0, -9.81),
				1, 0, 0));

		rigidBodies.add(new RigidBody(new Pentagon(2), 1, new Vector2D(0, 1), new Vector2D(), new Vector2D(0, -9.81),
				-0.1, 0, 0));
		rigidBodies.add(new RigidBody(new Hexagon(2), 2.5, new Vector2D(4, 2), new Vector2D(), new Vector2D(0, -9.81),
				0, 0, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);

	}

	private void test_random() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -5), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, -0.1, 0, 0, false, new Rectangle(10.5, 1)));
		MyRandom random = new MyRandom();
		while (rigidBodies.size() < 2) {
			AbstractShape shape = RandomPolygonBuilder.getPolygon((int) random.getNextHalfNormalDistribution(3, 4));
			RigidBody rb = new RigidBody(shape, 1, new Vector2D(0, 0), new Vector2D(0, 0), new Vector2D(0, -9), 0, 0,
					0);

			while (rbIntersects(rb, rigidBodies)) {
				rb.r.set(random.nextGaussian(2.2), random.nextGaussian(1.5));
			}
			rigidBodies.add(rb);
		}

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);

	}

	private boolean rbIntersects(RigidBody rb, List<RigidBody> rbList) {
		for (RigidBody testRB : rbList) {
			if (testRB.extendedIn(rb))
				return true;
		}
		return false;
	}

	private void test11() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, -0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 - 0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 + 0.01, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 8, 0, 0, false, new Rectangle(2, 1)));

		rigidBodies.add(new RigidBody(new Circle(1.23), 1.3, new Vector2D(-0.4399518902302824, -3.083141317330461),
				new Vector2D(-1.4412752755064793, 2.546927958603605), new Vector2D(0, 0), 17.949999999817752, 1, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);

	}

	private void test10() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, -0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 - 0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 + 0.01, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 8, 0, 0, false, new Rectangle(2, 1)));

		rigidBodies.add(new RigidBody(new Circle(1.23), 1.3, new Vector2D(-3, 0.5), new Vector2D(1, 0.5),
				new Vector2D(0, 0), 0, 1, 0));
		rigidBodies.add(new RigidBody(new Circle(0.5), 10, new Vector2D(3, 0.5), new Vector2D(1, 0.5),
				new Vector2D(0, 0), 0, 1, 0));
		rigidBodies.add(new RigidBody(new Circle(1), 1.3, new Vector2D(3, -2.5), new Vector2D(1, 4), new Vector2D(0, 0),
				0, 1, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);

	}

	private void test9() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, -0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 - 0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 + 0.01, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 8, 0, 0, false, new Rectangle(2, 1)));

		Polygon rectangle = new Rectangle();
		rigidBodies.add(new RigidBody(rectangle, 1.5, new Vector2D(-3.5, 0.5), new Vector2D(0, -0.5),
				new Vector2D(0, 0), Math.PI / 2, -0.4, 0));

		Polygon complexStructure = new Polygon(new Vector2D[] { new Vector2D(-1.5, 1.5), new Vector2D(1.5, 1.5),
				new Vector2D(1.5, -1.5), new Vector2D(-1.5, -1.5), new Vector2D(-1, -1), new Vector2D(1, -1),
				new Vector2D(-1, 1), new Vector2D(1, 1) });
		rigidBodies.add(new RigidBody(complexStructure, 5, new Vector2D(3, 3), new Vector2D(-0.5, 0),
				new Vector2D(0, 0), 0, -0.1, 0));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);

	}

	private void test8() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();

		rigidBodies.add(new RigidBody(2.7, new Vector2D(-2.3092533854673296, 2.317296134400402),
				new Vector2D(-0.40556197276118344, -0.7548117969613616), new Vector2D(0, 0), 10, -6.568889443058639,
				-0.5371540750645915, 0,
				new Polygon(new Vector2D[] { new Vector2D(-1.5, 1.5), new Vector2D(1.5, 1.5), new Vector2D(1.5, -1.5),
						new Vector2D(-1.5, -1.5), new Vector2D(-1, -1), new Vector2D(1, -1), new Vector2D(1, 1),
						new Vector2D(-1, 1) })));
		rigidBodies.add(new RigidBody(0.7, new Vector2D(1.048279777283462, 2.8125373703172523),
				new Vector2D(-3.484229851346371, -0.2719497647193772), new Vector2D(0, 0), 0.3, 125.84038262508483, 1,
				0, new Circle(0.5)));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
	}

	private void test7() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, -0.01, 0, 0, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0.01, 0, 0, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 - 0.01, 0, 0, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 + 0.01, 0, 0, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 8, 0, 0, new Rectangle(2, 1)));

		rigidBodies.add(new RigidBody(1, new Vector2D(-3.5, 0.5), new Vector2D(1, -0.5), new Vector2D(0, 0), 0.4,
				Math.PI / 2, -0.4, 0, new Rectangle()));

		rigidBodies.add(new RigidBody(1.5, new Vector2D(2.5, -1), new Vector2D(-1, 0), new Vector2D(0, 0), 0.6, 10, 0.5,
				0, new Circle(1)));
		rigidBodies.add(new RigidBody(1.5, new Vector2D(-2.3, -1), new Vector2D(-1, -4), new Vector2D(0, 0), 0.6, -10,
				-2, 0, new Circle(1)));
		rigidBodies.add(new RigidBody(0.7, new Vector2D(3, 3), new Vector2D(-0.4, -0.1), new Vector2D(0, 0), 0.3, 10, 1,
				0, new Circle(0.5)));

		rigidBodies
				.add(new RigidBody(2.7, new Vector2D(3, 3), new Vector2D(-0.5, 0), new Vector2D(0, 0), 10, 0, -0.1, 0,
						new Polygon(new Vector2D[] { new Vector2D(-1.5, 1.5), new Vector2D(1.5, 1.5),
								new Vector2D(1.5, -1.5), new Vector2D(-1.5, -1.5), new Vector2D(-1, -1),
								new Vector2D(1, -1), new Vector2D(1, 1), new Vector2D(-1, 1) })));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
	}

	private void test7_1() {
		List<RigidBody> rigidBodies = new ArrayList<RigidBody>();

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, -6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, -0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 6), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(-6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 - 0.01, 0, 0, false, new Rectangle(10.5, 1)));
		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(6, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 2 + 0.01, 0, 0, false, new Rectangle(10.5, 1)));

		rigidBodies.add(new RigidBody(Double.MAX_VALUE, new Vector2D(0, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, Math.PI / 8, 0, 0, false, new Rectangle(2, 1)));

		rigidBodies.add(new RigidBody(1, new Vector2D(-3.5, 0.5), new Vector2D(1, -0.5), new Vector2D(0, 0), 0.4,
				Math.PI / 2, -0.4, 0, new Rectangle()));

		rigidBodies.add(new RigidBody(1.5, new Vector2D(2.5, -1), new Vector2D(-1, 0), new Vector2D(0, 0), 0.6, 10, 0.5,
				0, new Circle(1)));
		rigidBodies.add(new RigidBody(1.5, new Vector2D(-2.3, -1), new Vector2D(-1, -4), new Vector2D(0, 0), 0.6, -10,
				-2, 0, new Circle(1)));
		rigidBodies.add(new RigidBody(0.7, new Vector2D(3, 3), new Vector2D(-0.4, -0.1), new Vector2D(0, 0), 0.3, 10, 1,
				0, new Circle(0.5)));

		rigidBodies
				.add(new RigidBody(2.7, new Vector2D(3, 3), new Vector2D(-0.5, 0), new Vector2D(0, 0), 10, 0, -0.1, 0,
						new Polygon(new Vector2D[] { new Vector2D(-1.5, 1.5), new Vector2D(1.5, 1.5),
								new Vector2D(1.5, -1.5), new Vector2D(-1.5, -1.5), new Vector2D(-1, -1),
								new Vector2D(1, -1), new Vector2D(1, 1), new Vector2D(-1, 1) })));

		this.rigidBodies = new RigidBody[rigidBodies.size()];
		this.rigidBodies = rigidBodies.toArray(this.rigidBodies);
	}

	private void test6() {
		rigidBodies = new RigidBody[2];
		rigidBodies[0] = new RigidBody(1, new Vector2D(-1, 0), new Vector2D(0, 0), new Vector2D(0, 0), 0.5, -2, 0, 0,
				new Rectangle());
		rigidBodies[1] = new RigidBody(1, new Vector2D(2.5, -1), new Vector2D(-1, 0), new Vector2D(0, 0), 0.5, 10, 0, 0,
				new Circle(1));
	}

	private void test5() {
		rigidBodies = new RigidBody[6];
		rigidBodies[0] = new RigidBody(Double.MAX_VALUE, new Vector2D(-1.1, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 5, 0, 0, new Rectangle());
		rigidBodies[1] = new RigidBody(Double.MAX_VALUE, new Vector2D(1.1, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0, 0, 0, new Rectangle());
		rigidBodies[2] = new RigidBody(Double.MAX_VALUE, new Vector2D(0, 1.1), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0, 0, 0, new Rectangle());
		rigidBodies[3] = new RigidBody(Double.MAX_VALUE, new Vector2D(0, -1.1), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0, 0, 0, new Rectangle());
		rigidBodies[4] = new RigidBody(1, new Vector2D(0.4, -0.2), new Vector2D(-0.5, 0), new Vector2D(0, 0), 0.25, 0,
				0, 0, new Polygon(new Vector2D[] { new Vector2D(-0.1, 0.1), new Vector2D(0.1, 0.1),
						new Vector2D(0.1, -0.1), new Vector2D(-0.1, -0.1) }));
		rigidBodies[5] = new RigidBody(1, new Vector2D(-0.4, 0.2), new Vector2D(-0.5, 0), new Vector2D(0, 0), 0.25, 0,
				0, 0, new Polygon(new Vector2D[] { new Vector2D(-0.1, 0.1), new Vector2D(0.1, 0.1),
						new Vector2D(0.1, -0.1), new Vector2D(-0.1, -0.1) }));

	}

	private void test4() {
		rigidBodies = new RigidBody[6];
		rigidBodies[0] = new RigidBody(Double.MAX_VALUE, new Vector2D(-1.1, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 5, 0, 0, new Rectangle());
		rigidBodies[1] = new RigidBody(Double.MAX_VALUE, new Vector2D(1.1, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0, 0, 0, new Rectangle());
		rigidBodies[2] = new RigidBody(Double.MAX_VALUE, new Vector2D(0, 1.1), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0, 0, 0, new Rectangle());
		rigidBodies[3] = new RigidBody(Double.MAX_VALUE, new Vector2D(0, -1.1), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 0, 0, 0, new Rectangle());
		rigidBodies[4] = new RigidBody(1, new Vector2D(0.4, -0.2), new Vector2D(-0.5, 0), new Vector2D(0, 0), 0.25, 0,
				0, 0, new Polygon(
						new Vector2D[] { new Vector2D(0.1, -0.1), new Vector2D(-0.1, -0.1), new Vector2D(0, 0.2) }));
		rigidBodies[5] = new RigidBody(1, new Vector2D(-0.4, 0.2), new Vector2D(-0.5, 0), new Vector2D(0, 0), 0.25, 0,
				0, 0,
				new Polygon(new Vector2D[] { new Vector2D(-0.1, 0.1), new Vector2D(0.1, 0.1), new Vector2D(0.1, -0.1),
						new Vector2D(-0.1, -0.1), new Vector2D(-0.2, -0.2), new Vector2D(-0.3, -0.3) }));

	}

	public void test1() {
		rigidBodies = new RigidBody[2];
		rigidBodies[0] = new RigidBody(1, new Vector2D(-1, 0), new Vector2D(1, 0), new Vector2D(0, 0), 0.5, 10, 0, 0,
				new Rectangle());
		rigidBodies[1] = new RigidBody(1, new Vector2D(1, 0), new Vector2D(-0.5, 0), new Vector2D(0, 0), 0.5, 0, 0, 0,
				new Rectangle());
	}

	public void test2() {
		rigidBodies = new RigidBody[2];
		rigidBodies[0] = new RigidBody(1, new Vector2D(-1, 0), new Vector2D(1, 0), new Vector2D(0, 0), 0.5, 10, 0, 0,
				new Circle(2));
		rigidBodies[1] = new RigidBody(1, new Vector2D(2.5, 0), new Vector2D(-1, 0), new Vector2D(0, 0), 0.5, 10, 0, 0,
				new Circle(1));

	}

	public void test3() {
		rigidBodies = new RigidBody[5];
		rigidBodies[0] = new RigidBody(Double.MAX_VALUE, new Vector2D(-2, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 10, 0, 0, new Circle(1.3));
		rigidBodies[1] = new RigidBody(Double.MAX_VALUE, new Vector2D(2, 0), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 10, 1, 0, new Circle(1.3));
		rigidBodies[2] = new RigidBody(Double.MAX_VALUE, new Vector2D(0, 2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 10, 0, 0, new Circle(1.3));
		rigidBodies[3] = new RigidBody(Double.MAX_VALUE, new Vector2D(0, -2), new Vector2D(0, 0), new Vector2D(0, 0),
				Double.MAX_VALUE, 10, 0, 0, new Circle(1.3));
		rigidBodies[4] = new RigidBody(1, new Vector2D(0, 0), new Vector2D(1, 1.1), new Vector2D(0, 0), 0.5, 10, 0, 0,
				new Circle(0.1));
	}

	@Override
	public void f(double t, double dt) {
		double E_gesamt_temp = 0;
		for (RigidBody rb : rigidBodies) {
			rb.f(t, dt);
			E_gesamt_temp += rb.E_rb;
		}
		E_gesamt = E_gesamt_temp;
	}

	@Override
	public void g(double t, AfterEventDescription aed) {
		for (int i = 0; i < rigidBodies.length - 1; i = i + 1) {
			for (int j = rigidBodies.length - 1; j > i; j = j - 1) {
				rigidBodies[i].collisionWithRigidBodyCheck(aed, rigidBodies[j], t, rigidBodies);
			}
		}
	}

	@Override
	public void initSimulationParameters(SimulationParameters s) {
		s.fastMotionFactor = 1;
	}

	@Override
	public void initGraphicsComponents(GraphicsComponents g, Structure s, Recorder r, SimulationParameters sp) {
		MechanicsTVG mTVG = new MechanicsTVG(this, s, r);
		mTVG.geometry.setUserArea(-6, 6, -3, 5);
		mTVG.velocityScaling = 100;
		mTVG.accelerationScaling = 10;
		mTVG.angularVelocityScaling = 0.2;
		mTVG.angularAccelerationScaling = 0.2;
		mTVG.FhScaling = 30;
		mTVG.showAngularAcceleration = false;
		mTVG.showAngularDisplacement = false;
		mTVG.showAngularVelocity = false;
		mTVG.showLabels = false;
		mTVG.showPath = false;
		mTVG.showVelocity = true;
		mTVG.showAcceleration = false;
		mTVG.showFh = false;
		mTVG.showFr = true;
		g.addTVG(mTVG);
	}

	public static void main(String args[]) {
		start();
	}
}
