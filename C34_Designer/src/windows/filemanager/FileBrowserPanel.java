package windows.filemanager;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.TableCellRenderer;

import windows.filemanager.filelistproviders.FileDescription;
import windows.filemanager.filelistproviders.FileListProvider;
import windows.filemanager.filelistproviders.LocalFileListProvider;

public class FileBrowserPanel extends JPanel {
	private static final long serialVersionUID = 1L;
	
	private FileListProvider _fileListProvider;
	private String _currentDirectory;
	
	private JTable _filesTable;
	private JLabel _lblCurrentDirectory;
	
	public FileBrowserPanel(String title, FileListProvider fileListProvider, String initialDirectory) {
		this._fileListProvider = fileListProvider;
		createGUIComponents(title);
		setDirectory(initialDirectory);
	}
	
	public void createGUIComponents(String title) {
		GridBagConstraints gridBagContraint;
		setLayout(new GridBagLayout());

		
		gridBagContraint = new GridBagConstraints();
		gridBagContraint.fill = GridBagConstraints.HORIZONTAL;
		gridBagContraint.weightx = 1.0;
		gridBagContraint.weighty = 0.0;
		gridBagContraint.gridx = 0;
		gridBagContraint.gridy = 0;
		add(createDirectoryBrowserPanel(), gridBagContraint);
		
		
		gridBagContraint = new GridBagConstraints();
		gridBagContraint.fill = GridBagConstraints.BOTH;
		gridBagContraint.weightx = 1.0;
		gridBagContraint.weighty = 1.0;
		gridBagContraint.gridx = 0;
		gridBagContraint.gridy = 1;
		add(createFileBrowserTable(), gridBagContraint);
		
		
		gridBagContraint = new GridBagConstraints();
		gridBagContraint.fill = GridBagConstraints.HORIZONTAL;
		gridBagContraint.weightx = 1.0;
		gridBagContraint.weighty = 0.0;
		gridBagContraint.gridx = 0;
		gridBagContraint.gridy = 2;
		add(createAdditionalControlsPanel(), gridBagContraint);
	}
	
	public JComponent createDirectoryBrowserPanel() {
		JPanel panel = new JPanel();
		
		_lblCurrentDirectory = new JLabel();
		panel.add(_lblCurrentDirectory);
		
		return panel;
	}
	
	public JComponent createFileBrowserTable() {
		_filesTable = new JTable(new FileBrowserTableModel()) {
			private static final long serialVersionUID = 1L;

			@Override
			public Component prepareRenderer(TableCellRenderer renderer,
					int row, int column) {
				Component comp = super.prepareRenderer(renderer, row, column);
				if (JComponent.class.isInstance(comp)){
		            ((JComponent)comp).setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
		        }
				return comp;
			}
		};
		_filesTable.setRowHeight(28);
		
		_filesTable.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
		_filesTable.getColumnModel().getColumn(0).setPreferredWidth(24);
		
		return new JScrollPane(_filesTable);
	}
	
	public JComponent createAdditionalControlsPanel() {
		JPanel panel = new JPanel();
		panel.setLayout(new FlowLayout(FlowLayout.LEFT));
		
		panel.add(new JButton("Upload"));
		
		return panel;
	}
	
	public void setDirectory(String path) {
		_currentDirectory = path;
		_filesTable.setModel(new FileBrowserTableModel(_fileListProvider.getFiles(path)));		
		_lblCurrentDirectory.setText(path);
	}
	
	public static void main(String[] args) {
		JFrame frame = new JFrame("Test file browser");
		frame.setLayout(new BorderLayout());
		frame.add(new FileBrowserPanel("Local files", new LocalFileListProvider(), "/"));
		frame.setSize(800, 600);
		frame.setLocation(100, 100);
		frame.setVisible(true);
	}
	
}
