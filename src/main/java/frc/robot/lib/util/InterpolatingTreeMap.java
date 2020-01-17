package frc.robot.lib.util;

import java.util.TreeMap;
import java.util.Map;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined
 * by making a guess from points that are defined. This uses linear
 * interpolation.
 * 
 * @param <K>
 *            The type of the key (must implement InverseInterpolable)
 * @param <V>
 *            The type of the value (must implement Interpolable)
 */
public class InterpolatingTreeMap<K extends InverseInterpolable<K> & Comparable<K>, V extends Interpolable<V>> extends TreeMap<K, V> 
{
    private static final long serialVersionUID = 8347275262778054124L;

    int maxSize;

    public InterpolatingTreeMap(int _maxSize) 
    {
        maxSize = _maxSize;
    }

    public InterpolatingTreeMap() 
    {
        this(0);
    }

    /**
     * Inserts a key value pair, and trims the tree if a max size is specified
     * 
     * @param _key
     *            Key for inserted data
     * @param _value
     *            Value for inserted data
     * @return the value
     */
    @Override
    public V put(K _key, V _value) 
    {
        if ((maxSize > 0) && (size() >= maxSize)) 
        {
            // "Prune" the tree when it overflows
            remove( firstKey() );
        }

        super.put(_key, _value);

        return _value;
    }

    @Override
    public void putAll(Map<? extends K, ? extends V> _map) 
    {
        System.out.println("Unimplemented Method");
    }

    /**
     *
     * @param _key
     *            Lookup for a value (does not have to exist)
     * @return V or null; V if it is Interpolable or exists, null if it is at a
     *         bound and cannot average
     */
    public V getInterpolated(K _key) 
    {
        V retVal = get(_key);
        
        if (retVal == null) 
        {
            /** Get surrounding keys for interpolation */
            K topBound = ceilingKey(_key);
            K bottomBound = floorKey(_key);

           /**
             * If attempting interpolation at ends of tree, return the nearest
             * data point
             */
            if (topBound == null && bottomBound == null) 
            {
            	retVal = null;
            } 
            else if (topBound == null) 
            {
            	retVal = get(bottomBound);
            } 
            else if (bottomBound == null) 
            {
            	retVal = get(topBound);
            }
            else
            {
	            /** Get surrounding values for interpolation */
	            V topElem = get(topBound);
	            V bottomElem = get(bottomBound);
	            retVal = bottomElem.interpolate(topElem, bottomBound.inverseInterpolate(topBound, _key));
            }            
        }
        
        return retVal;
    }
}