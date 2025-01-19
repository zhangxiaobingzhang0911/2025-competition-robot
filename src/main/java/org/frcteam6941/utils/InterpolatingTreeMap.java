package org.frcteam6941.utils;

import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolatable;

// A TreeMap that automatically trims its size to a specified maximum size upon insertion of new elements.
// The class is designed to work with keys that extend Number and values that implement Interpolatable.
public class InterpolatingTreeMap<K extends Number, V extends Interpolatable<V>> extends TreeMap<K, V> {
    final int max_;

    // Constructor that initializes the maximum size of the TreeMap.
    // If the maximum size is reached, the TreeMap will remove the first (smallest) key-value pair.
    public InterpolatingTreeMap(int maximumSize) {
        max_ = maximumSize;
    }

    // Default constructor that initializes the maximum size to 0, meaning no automatic trimming.
    public InterpolatingTreeMap() {
        this(0);
    }

    // Inserts a key-value pair into the TreeMap and trims the tree if the maximum size is exceeded.
    // If the maximum size is specified and the current size of the TreeMap is equal to or greater than this size,
    // the smallest key-value pair will be removed to make space for the new pair.
    @Override
    public V put(K key, V value) {
        if (max_ > 0 && max_ <= size()) {
            // Remove the first key-value pair to maintain the maximum size.
            K first = firstKey();
            remove(first);
        }

        super.put(key, value);

        return value;
    }

    // This method is not implemented and currently prints a message indicating so.
    @Override
    public void putAll(Map<? extends K, ? extends V> map) {
        System.out.println("Unimplemented Method");
    }

    // Returns an interpolated value for the given key and interpolant.
    // If the key exists, its associated value is returned.
    // If the key does not exist, the method attempts to interpolate between the nearest key-value pairs.
    // If interpolation is not possible (e.g., the key is at a boundary and there are no surrounding keys),
    // the nearest key-value pair is returned, or null if no such pair exists.
    public V getInterpolated(K key, double interpolant) {
        Optional<V> gotval = Optional.ofNullable(get(key));
        return gotval.orElseGet(() -> {
            // Determine the keys just above and below the given key for interpolation.
            K topBound = ceilingKey(key);
            K bottomBound = floorKey(key);

            // Handle cases where the key is outside the bounds of the existing keys.
            if (topBound == null && bottomBound == null) {
                return null;
            } else if (topBound == null) {
                return get(bottomBound);
            } else if (bottomBound == null) {
                return get(topBound);
            }

            // Retrieve the values associated with the surrounding keys.
            V topElem = get(topBound);
            V bottomElem = get(bottomBound);
            // Perform interpolation between the bottom and top elements using the provided interpolant.
            return bottomElem.interpolate(topElem, interpolant);
        });
    }
}