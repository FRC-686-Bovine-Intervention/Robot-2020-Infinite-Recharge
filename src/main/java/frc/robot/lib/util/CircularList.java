package frc.robot.lib.util;

import java.util.ArrayList;

/* Careful using this
 * For it to work the list size shouldn't change, and only positive indices should be used
 */

@SuppressWarnings("serial")
public class CircularList<E> extends ArrayList<E> 
{
	private int idx = 0;
	
	public E getNext() 
	{
		return get(idx + 1);
	}
	
    @Override
	public E get(int index) 
	{
		idx = index % size();
		return super.get(idx);
	}
}