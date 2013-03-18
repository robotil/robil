package document.ui;

import java.awt.Image;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;

public class Utilities {
	
	public static ImageIcon loadIcon(String iconName) {
		try{
			return new ImageIcon(Utilities.class.getClassLoader().getResource("icons/" + iconName));
		} catch(Exception e) {
			return null;
		}
	}
	
	public static Image loadImage(String imageName) {
		try{
			return ImageIO.read(Utilities.class.getClassLoader().getResource("icons/" + imageName));
		} catch(Exception e) {
			return null;
		}
	}
	
}
