package document.ui;

import javax.swing.ImageIcon;

public class Utilities {
	public static ImageIcon loadIcon(String iconName){
		try{
			return new ImageIcon(Utilities.class.getClassLoader().getResource("icons/" + iconName));
		}catch(Exception e){
			return null;
		}
	}
}
