package document.listeners;

import document.PropertiesEditor;
import document.PropertiesXmlHandler;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JTable;
import javax.swing.table.AbstractTableModel;

import org.xml.sax.SAXException;

public class PropertiesEditorAction implements ActionListener {

	private JTable table;
	private JFrame frame;
	
	public PropertiesEditorAction(JTable table, JFrame frame) {
		this.table = table;
		this.frame = frame;
	}

	public void actionPerformed(ActionEvent a) {

		switch (a.getActionCommand()) {
		case "open_properties_dialog":
			PropertiesEditor.show(PropertiesXmlHandler.lastPath);
			break;
		case "load_properties":
			try {
				String path = Dialogs.openFile("Load Properties File", "xml", ".");
				
				if (path != null) {
					PropertiesXmlHandler.lastPath = path;
				}
				PropertiesXmlHandler.loadAndSetProperties(PropertiesXmlHandler.lastPath);
				
				updateTable();
				
			} catch (Exception ex) {
				JOptionPane.showMessageDialog(null, "Error loading file", "Load properties",
						JOptionPane.ERROR_MESSAGE);
			}
			break;
		case "save_properties":
			try {
				String path = Dialogs.saveFile("Save Properties", "xml");
				
				if (path == null) {
					return;
				}

				PropertiesXmlHandler.setParametersFromTable(table);
				PropertiesXmlHandler.saveToFile(path);
				
				JOptionPane.showMessageDialog(null, "Properties file saved");
			} catch (Exception ex) {
				JOptionPane.showMessageDialog(null, "Error saving file", "Save properties",
						JOptionPane.ERROR_MESSAGE);
			}

			break;
		case "cancel_properties":
			frame.dispose();
			break;
		}
	}
	
	private void updateTable() {
//		table.getModel()
		((AbstractTableModel)table.getModel()).fireTableDataChanged();
	}
}