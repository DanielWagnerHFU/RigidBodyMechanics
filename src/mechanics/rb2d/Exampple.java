package mechanics.rb2d;

import de.physolator.usr.components.Vector2D;
import mechanics.rb2d.shapes.AbstractShape;
import mechanics.rb2d.shapes.Circle;

public class Exampple extends RigidBody {

	public Exampple(AbstractShape shape, double m, Vector2D r, Vector2D v, Vector2D a, double phi, double omega,
			double alpha) {
		super(shape, m, r, v, a, phi, omega, alpha);
		// TODO Auto-generated constructor stub
	}

	@Override
	public void f(double t, double dt) {
		// TODO Auto-generated method stub
		super.f(t, dt);
	}

	

}
