package document.actions;

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

		if (a.getActionCommand().equals("open_properties_dialog")) {
			PropertiesEditor.show(PropertiesXmlHandler.lastPath);
		} else if (a.getActionCommand().equals("load_properties")) {
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
		} else if (a.getActionCommand().equals("save_properties")) {
			try {
				String path = Dialogs.saveFile("Save Properties", "xml");
				
				if (path == null || path.equals("")) {
					return;
				}

				// updateTable();
				
				PropertiesXmlHandler.setParametersFromTable(table);
				PropertiesXmlHandler.saveToFile(path);
				
				JOptionPane.showMessageDialog(null, "Properties file saved");
			} catch (Exception ex) {
				JOptionPane.showMessageDialog(null, "Error saving file", "Save properties",
						JOptionPane.ERROR_MESSAGE);
			}

		} else if (a.getActionCommand().equals("cancel_properties")) {
			frame.dispose();
		}
	}
	
	private void updateTable() {
//		table.getModel()
		((AbstractTableModel)table.getModel()).fireTableDataChanged();
	}
}