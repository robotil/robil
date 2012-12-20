package elements;

import java.awt.Graphics2D;

public class View {
	
	public interface ChangesListener{
		public void onViewChange();
	}

	public Vec loc = new Vec(0,0);
	public double zoom = 1;
	public Graphics2D graphics = null;
	

}
