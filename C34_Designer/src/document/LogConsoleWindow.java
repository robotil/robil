package document;

//ColorPane.java
//A simple extension of JTextPane that allows the user to easily append
//colored text to the document.
//

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;

import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.swing.JTextPane;
import javax.swing.text.AttributeSet;
import javax.swing.text.SimpleAttributeSet;
import javax.swing.text.StyleConstants;
import javax.swing.text.StyleContext;

import logger.LogManager;

public class LogConsoleWindow extends JTextPane {

	private static final long serialVersionUID = 7243421902696150132L;

	public static void show(String logFile) {
		// logFile = LogManager.getOutputFileName();

		LogConsoleWindow pane = new LogConsoleWindow();
		JScrollPane scroll = new JScrollPane(pane);
		pane.setScrollPane(scroll);

		JFrame f = new JFrame("Log Console");
		f.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		f.setContentPane(scroll);
		f.setSize(600, 400);
		f.setVisible(true);
	}

	private String logFile;

	private JScrollPane scroll;

	public LogConsoleWindow() {
		new Thread(new Runnable() {

			@Override
			public void run() {

				while (true) {
					String outputContent = LogManager.getOutputContent();

					if (!outputContent.equals(getText())) {
						setText(LogManager.getOutputContent());

						// Scroll down
						// scrollRectToVisible(new
						// Rectangle(0,getDocument().getLength(),1,1));
						setCaretPosition(getDocument().getLength());
					}

					try {
						Thread.sleep(500);
					} catch (InterruptedException e) {
					}
				}
			}
		}).start();
	}

	private void append(Color c, String s, Object style) { // better
															// implementation--uses
		// StyleContext
		StyleContext sc = StyleContext.getDefaultStyleContext();
		AttributeSet aset = sc.addAttribute(SimpleAttributeSet.EMPTY, style, c);

		int len = getDocument().getLength(); // same value as
												// getText().length();
		setCaretPosition(len); // place caret at the end (with no selection)
		setCharacterAttributes(aset, false);
		replaceSelection(s); // there is no selection, so inserts at caret
	}

	public void appendNaive(Color c, String s) { // naive implementation
		// bad: instiantiates a new AttributeSet object on each call
		SimpleAttributeSet aset = new SimpleAttributeSet();
		StyleConstants.setForeground(aset, c);

		int len = getText().length();
		setCaretPosition(len); // place caret at the end (with no selection)
		setCharacterAttributes(aset, false);
		replaceSelection(s); // there is no selection, so inserts at caret
	}

	private void processData() {
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new InputStreamReader(
					new FileInputStream(this.logFile)));

			String line = null;
			while ((line = reader.readLine()) != null) {

			}

		} catch (FileNotFoundException ex) {
			JOptionPane.showMessageDialog(null, "Log file " + this.logFile
					+ " cannot be found", "Open Log Console",
					JOptionPane.ERROR_MESSAGE);
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

	public void refreshData() {
		this.setText("");

		processData();
	}

	public void setScrollPane(JScrollPane scrollPane) {
		this.scroll = scrollPane;
	}

	private void writeError(String txt) {
		append(Color.RED, txt + "\n", StyleConstants.Foreground);
	}

	private void writeInformation(String txt) {
		append(Color.GREEN, txt + "\n", StyleConstants.Foreground);
	}

	private void writeNormal(String txt) {
		append(Color.BLACK, txt + "\n", StyleConstants.Foreground);
	}

	private void writePlanRun(String txt) {
		append(Color.YELLOW, txt + "\n", StyleConstants.Background);
	}

}