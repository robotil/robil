package document.history;

import java.util.ArrayList;
import java.util.Date;

import document.Document;
import elements.GElement;

/**
 * Stores a copy of all elements & arrows in specified document
 * @author blackpc
 *
 */
class Snapshot {
	
	private Date _date;
	private Document _document;
	private int _sequenceNumber;
	private ArrayList<GElement> _elements;
	private ArrayList<GElement> _arrows;
	
	private Snapshot(Document document, ArrayList<GElement> elements, ArrayList<GElement> arrows, int sequenceNumber) {
		this._date 		= new Date();
		this._document 	= document;
		this._elements 	= elements;
		this._arrows 	= arrows;
		this._sequenceNumber = sequenceNumber;
	}
	
	/**
	 * Restores saved state for related document
	 */
	public void activate() {
		// ***************************************************************************
		// *** Clones elements & arrows from snapshot, and replace document's elements
		// *** Note: snapshot's elements & arrows must remain immutable
		// ***************************************************************************
		
		// Cloned elements & arrows
		ArrayList<GElement> elements = new ArrayList<GElement>();
		ArrayList<GElement> arrows   = new ArrayList<GElement>();
		
		// Set elements & arrows to allow cloning
		_document.elements = _elements;
		_document.arrays  	= _arrows;
		
		// Clone elements & arrows
		_document.cloneElements(elements, arrows);
		
		// Replace elements & arrows by clones
		_document.elements = elements;
		_document.arrays  	= arrows;		

		// ***************************************************************************
		// *** Set the right view
		// ***************************************************************************
		for (GElement element : elements) 
			element.setView(_document.view);
		
		for (GElement element : arrows) 
			element.setView(_document.view);

		_document.repaint();
	}
	
	/**
	 * Creates snapshot from specified document
	 * @param document Target document
	 * @param sequenceNumber Sequence number of new snapshot
	 * @return Snapshot
	 */
	public static Snapshot create(Document document, int sequenceNumber) {
		ArrayList<GElement> elements = new ArrayList<GElement>();
		ArrayList<GElement> arrows   = new ArrayList<GElement>();
		document.cloneElements(elements, arrows);
		return new Snapshot(document, elements, arrows, sequenceNumber);
	}
	
	/**
	 * The date & time the snapshot was taken
	 * @return
	 */
	public Date getDate() {
		return _date;
	}
	
	@Override
	public String toString() {
		return Integer.toString(_sequenceNumber) + "X" + _date.getTime();
	}
}
