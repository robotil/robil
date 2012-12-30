package document;

//ColorPane.java
//A simple extension of JTextPane that allows the user to easily append
//colored text to the document.
//

import javax.swing.*;
import javax.swing.text.*;

import document.actions.Dialogs;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;

public class LogConsoleWindow extends JTextPane {

	private String logFile;

	public void appendNaive(Color c, String s) { // naive implementation
		// bad: instiantiates a new AttributeSet object on each call
		SimpleAttributeSet aset = new SimpleAttributeSet();
		StyleConstants.setForeground(aset, c);

		int len = getText().length();
		setCaretPosition(len); // place caret at the end (with no selection)
		setCharacterAttributes(aset, false);
		replaceSelection(s); // there is no selection, so inserts at caret
	}

	private void append(Color c, String s, Object style) { // better implementation--uses
											// StyleContext
		StyleContext sc = StyleContext.getDefaultStyleContext();
		AttributeSet aset = sc.addAttribute(SimpleAttributeSet.EMPTY,
				style, c);

		int len = getDocument().getLength(); // same value as
												// getText().length();
		setCaretPosition(len); // place caret at the end (with no selection)
		setCharacterAttributes(aset, false);
		replaceSelection(s); // there is no selection, so inserts at caret
	}

	public void refreshData() {
		this.setText("");

		processData();
	}

	private void processData() {
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new InputStreamReader(
					new FileInputStream(logFile)));
			
			String line = null;
			while ((line = reader.readLine()) != null) {
				
			}
			
		} catch (FileNotFoundException ex) {
			JOptionPane.showMessageDialog(null, "Log file " + logFile + " cannot be found",
					"Open Log Console", JOptionPane.ERROR_MESSAGE);
		} catch (IOException ex) {
			System.err.println("Error in reading logfile " + ex.getMessage());
		} finally {
			try {
				if (reader == null) {
					return;
				}
				
				reader.close();
			} catch (IOException ex) {
				System.err.println(ex.getMessage());
			}
		}
	}

	private void writeError(String txt) {
		append(Color.RED, txt+"\n", StyleConstants.Foreground);
	}
	
	private void writeInformation(String txt) {
		append(Color.GREEN, txt+"\n", StyleConstants.Foreground);
	}
	
	private void writeNormal(String txt) {
		append(Color.BLACK, txt+"\n", StyleConstants.Foreground);
	}
	
	private void writePlanRun(String txt) {
		append(Color.YELLOW, txt+"\n", StyleConstants.Background);
	}
	
	public LogConsoleWindow(String logFile) {
		this.logFile = logFile;
	}

	public static void show(String logFile) {
		LogConsoleWindow pane = new LogConsoleWindow(logFile);

		JFrame f = new JFrame("Log Console");
		f.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		f.setContentPane(new JScrollPane(pane));
		f.setSize(600, 400);
		f.setVisible(true);
	}
}