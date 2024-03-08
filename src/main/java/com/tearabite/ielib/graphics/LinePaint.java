package com.tearabite.ielib.graphics;

import android.graphics.Color;
import android.graphics.Paint;

/**
 * A class to represent a paint object for drawing lines
 */
public class LinePaint extends Paint
{
    public LinePaint(int color)
    {
        setColor(color);
        setAntiAlias(true);
        setStrokeCap(Paint.Cap.ROUND);
    }

    public static LinePaint RED = new LinePaint(Color.RED);
    public static LinePaint BLUE = new LinePaint(Color.BLUE);
    public static LinePaint BLACK = new LinePaint(Color.BLACK);
    public static LinePaint WHITE = new LinePaint(Color.WHITE);
}