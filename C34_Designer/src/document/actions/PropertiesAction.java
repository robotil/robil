package document.actions;

import document.PropertiesEditor;
import document.PropertiesXmlHandler;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JDialog;
import javax.swing.JOptionPane;
import javax.swing.JTable;

import org.xml.sax.SAXException;

public class PropertiesAction implements ActionListener {

	public PropertiesAction() {}

	public void actionPerformed(ActionEvent a) {
		PropertiesEditor.show(PropertiesXmlHandler.lastPath);
	}	
}