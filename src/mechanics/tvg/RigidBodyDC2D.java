package mechanics.tvg;

import de.physolator.usr.util.Color;
import mechanics.rb2d.RigidBody;

public class RigidBodyDC2D extends DrawableComponent2D {

	public RigidBody rb;

	public RigidBodyDC2D(MechanicsTVG mTVG, String name, RigidBody rb) {
		super(mTVG, name);
		this.rb = rb;
	}

	@Override
	public void paint() {
		//mTVG.style.strokeColor = Color.make(0, 0, 0);
		//mTVG.style.fillColor = Color.make(0, 0, 0, 1); 
		//rb.shape.paint(mTVG, rb.r, rb.phi);
		//mTVG.drawAngularDisplcementVelocityAcceleration(rb.r, rb.phi, rb.omega, rb.alpha);
		//mTVG.drawPointMass(rb.r.x, rb.r.y, 0.05, mTVG);
	}
}
