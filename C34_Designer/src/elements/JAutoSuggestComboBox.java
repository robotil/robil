package elements;

import java.awt.EventQueue;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.Collections;
import java.util.List;
import java.util.Vector;

import javax.swing.DefaultComboBoxModel;
import javax.swing.JComboBox;
import javax.swing.JTextField;

public class JAutoSuggestComboBox extends JComboBox<String> {
	private static final long serialVersionUID = 3397382953165041968L;

	private static DefaultComboBoxModel<String> getSuggestedModel(
			List<String> list, String text) {
		final DefaultComboBoxModel<String> model = new DefaultComboBoxModel<String>();

		for (final String s : list)
			if (s.toLowerCase().startsWith(text.toLowerCase()))
				model.addElement(s);

		return model;
	}

	//
	// public static void main(String[] args) {
	// final Vector<String> items = new Vector<String>();
	// items.add("Good");
	// items.add("Goody");
	//
	// final JFrame frame = new JFrame();
	// frame.add(new JAutoSuggestComboBox(items));
	// frame.setVisible(true);
	// }
	private final JTextField _textField;
	private JComboBox<String> _comboBox = new JComboBox<String>();
	private Vector<String> _itemsList = new Vector<String>();

	private boolean _hideFlag = false;

	public JAutoSuggestComboBox(Vector<String> items) {
		this._comboBox = this;
		this._comboBox.setEditable(true);
		this._textField = (JTextField) this._comboBox.getEditor()
				.getEditorComponent();
		this._textField.addKeyListener(new KeyAdapter() {
			@Override
			public void keyPressed(KeyEvent e) {
				final String text = JAutoSuggestComboBox.this._textField
						.getText();
				final int code = e.getKeyCode();
				if (code == KeyEvent.VK_ENTER) {
					if (!JAutoSuggestComboBox.this._itemsList.contains(text)) {
						JAutoSuggestComboBox.this._itemsList.addElement(text);
						Collections.sort(JAutoSuggestComboBox.this._itemsList);
						setModel(
								getSuggestedModel(
										JAutoSuggestComboBox.this._itemsList,
										text), text);
					}
					JAutoSuggestComboBox.this._hideFlag = true;
				} else if (code == KeyEvent.VK_ESCAPE) {
					JAutoSuggestComboBox.this._hideFlag = true;
				} else if (code == KeyEvent.VK_RIGHT) {
					for (int i = 0; i < JAutoSuggestComboBox.this._itemsList
							.size(); i++) {
						final String str = JAutoSuggestComboBox.this._itemsList
								.elementAt(i);
						if (str.toLowerCase().startsWith(text.toLowerCase())) {
							JAutoSuggestComboBox.this._comboBox
									.setSelectedIndex(-1);
							JAutoSuggestComboBox.this._textField.setText(str);
							return;
						}
					}
				}
			}

			@Override
			public void keyTyped(KeyEvent e) {
				EventQueue.invokeLater(new Runnable() {
					@Override
					public void run() {
						final String text = JAutoSuggestComboBox.this._textField
								.getText();
						if (text.length() == 0) {
							JAutoSuggestComboBox.this._comboBox.hidePopup();
							setModel(new DefaultComboBoxModel<String>(
									JAutoSuggestComboBox.this._itemsList), "");
						} else {
							final DefaultComboBoxModel<String> model = getSuggestedModel(
									JAutoSuggestComboBox.this._itemsList, text);
							if (model.getSize() == 0
									|| JAutoSuggestComboBox.this._hideFlag) {
								JAutoSuggestComboBox.this._comboBox.hidePopup();
								JAutoSuggestComboBox.this._hideFlag = false;
							} else {
								setModel(model, text);
								JAutoSuggestComboBox.this._comboBox.showPopup();
							}
						}
					}
				});
			}
		});

		this._itemsList = items;
		setModel(new DefaultComboBoxModel<String>(this._itemsList), "");
	}

	public String getText() {
		return ((JTextField) this._comboBox.getEditor().getEditorComponent())
				.getText();
	}

	private void setModel(DefaultComboBoxModel<String> mdl, String str) {
		this._comboBox.setModel(mdl);
		this._comboBox.setSelectedIndex(-1);
		this._textField.setText(str);
	}

	public void setText(String text) {
		((JTextField) this._comboBox.getEditor().getEditorComponent())
				.setText(text);
	}
}