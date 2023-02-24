// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Lookup Table
 * 
 *  Performs linear interpolation between values in table
 */
public class LookupTable
{
  private final String[] columns;

  /** Original or interpolated data point.
   *
   *  Provides one or more values for a "position".
   */
  public class Entry
  {
    final public double position;
    final public double[] values;

    Entry(final double position, final double[] values)
    {
      this.position = position;
      this.values = values;
    }

    /** @return First or maybe only value for the position */
    public double getValue()
    {
      return values[0];
    }

    @Override
    public String toString()
    {
      StringBuilder buf = new StringBuilder();
      buf.append(columns[0]).append(" ").append(position).append(": ");
      for (int i=0; i<values.length; ++i)
      {
        if (i > 0)
          buf.append(", ");
        buf.append(columns[i+1]).append(" ").append(values[i]);
      }
      return buf.toString();
    }
  }

  /** Table of data points */
  private final List<Entry> table = new ArrayList<>();

  /** Create lookup table
   *  @param columns Column names "Position", "Value1", "Value2", ...
   *  @param values Data for position, value1, value2, ...
   */
  public LookupTable(final String[] columns, final double... values)
  {
    if (columns.length < 2)
      throw new IllegalArgumentException("Need at least two columns, got " + Arrays.toString(columns));
    this.columns = columns;
    if (values.length % columns.length != 0)
      throw new IllegalArgumentException("Need list of " + Arrays.toString(columns) + ", i.e., multiple of " + columns.length + " values");
    for (int i = 0;  i < values.length;  i += columns.length)
    {
      double position = values[i];
      double[] entry_values = new double[columns.length-1];
      System.arraycopy(values, i+1, entry_values, 0, columns.length-1);
      table.add(new Entry(position, entry_values));
    }
    // Table must be sorted by first value
    table.sort((a, b) -> Double.compare(a.position, b.position));
  }
  
  /**  @param pos Position
    *  @return Values for that position
    */
  public Entry lookup(final double pos)
  {
    final int n = table.size();
    // Is position outside of table's position range?
    if (pos <= table.get(0).position)
      return table.get(0);
    if (pos >= table.get(n-1).position)
      return table.get(n-1);
    // Binary search starting with left, right set to complete table
    // https://en.wikipedia.org/wiki/Binary_search_algorithm#Procedure_for_finding_the_leftmost_element
    int l = 0, r = n;
    while (l < r)
    {   // Binary search: Find middle index, rounding down(!)
      final int m = (l + r) / 2;
      if (table.get(m).position < pos)
        l = m+1;   // pos must be in upper half
      else
        r = m;     // pos must be in lower half (or exact match)
    }
    // For an exact match, [l] is that element
    if (table.get(l).position == pos)
      return table.get(l);
    // Otherwise l points to the next larger element,
    // so pos is between element [l-1] and [l].
    // Interpolate each value between those two points
    final double[] values = new double[columns.length-1];
    for (int i=0; i<columns.length-1; ++i)
    {
      final double slope = (table.get(l).values[i] - table.get(l-1).values[i])   /
                           (table.get(l).position  - table.get(l-1).position);
      values[i] = table.get(l-1).values[i] + (pos - table.get(l-1).position) * slope;
    }
    return new Entry(pos, values);
  }

  @Override
  public String toString()
  {
    StringBuilder buf = new StringBuilder();
    for (Entry entry : table)
        buf.append(entry).append("\n");
    return buf.toString();
  }

  // Test/demo
  public static void main(String[] args)
  {
    // Example for lookup of spinner speeds for distance
    final LookupTable speeds = new LookupTable(
      new String[] { "Position", "Speed", "Hood", "Deviation" },
                             30,      65,      0,      0,
                             20,      60,      0,      0,
                              0,      55,      0,      0,
                            -20,      60,      0,      0,
                            -30,      75,      0,      0);

    for (double d : new double[] { 40, 30, 25, 20, 10, 5, 0, -5, -10, -20, -30, -40})
      System.out.println(d + " -> " + speeds.lookup(d));
  }
}