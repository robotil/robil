package elements;

import java.awt.Graphics2D;

public class View {

	public interface ChangesListener {
		public void onViewChange();
	}

	public Vec loc = new Vec(0, 0);
	public double zoom = 1;
	public Graphics2D graphics = null;

	@Override
	public View clone() {
		View v = new View();
		v.loc = new Vec(this.loc);
		v.zoom = this.zoom;
		v.graphics = this.graphics;
		return v;
	}
}
