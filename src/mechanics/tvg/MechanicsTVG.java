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
	private ArrayList<DrawableComponent2D> drawableComponentList = new ArrayList<DrawableComponent2D>();

	public MechanicsTVG(PhysicalSystem ps, Structure structure, Recorder recorder) {
		this.physicalSystem = ps;
		this.recorder = recorder;
		this.structure = structure;
		scalesStyle.visible = true;
		geometry.setRim(40, 40, 40, 40);
		geometry.setUserArea(-1, 1, -1, 1);
		geometry.useFixedXYRatio(1);
		geometry.useExtendedUserArea = true;
		geometry.adoptFontSizeScalingFactorFromOS = true;
		scalesStyle.x.showGridLines = true;
		scalesStyle.y.showGridLines = true;
		scalesStyle.x.showMinorGridLines = true;
		scalesStyle.y.showMinorGridLines = true;
		
		for (StructureElement structureElement : structure.getSubstructures()) {
			System.out.println(IDrawable.class.isAssignableFrom(structureElement.getType()));
			if (IDrawable.class.isAssignableFrom(structureElement.getType())) {
				drawableComponentList.add(new DrawableComponent2D(this, structureElement.getName(), (IDrawable) structureElement.getObject()));
			}
		}
	}
	
	@Override
	public void paint() {
		beginClipping();
		style.useUCS = true;
		for (DrawableComponent2D dc : drawableComponentList) {
			dc.paint();
		}
		endClipping();
	}
}