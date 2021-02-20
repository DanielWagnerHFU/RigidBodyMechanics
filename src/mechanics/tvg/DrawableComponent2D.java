package mechanics.tvg;

import de.physolator.usr.components.Vector2D;
import de.physolator.usr.util.Color;

public class DrawableComponent2D {
	protected MechanicsTVG mTVG;
	protected String name;
	public IDrawable drawable;

	public DrawableComponent2D(MechanicsTVG mTVG, String name, IDrawable drawable) {
		this.mTVG = mTVG;
		this.name = name;
		this.drawable = drawable;
	}

	public void paint() {
		drawable.paint(mTVG, new Vector2D(), 0);
		
	}
}
