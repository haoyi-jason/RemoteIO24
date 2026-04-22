using Microsoft.Maui.Graphics;

namespace RfRxV02Host.Maui;

public sealed class FastLineChartDrawable : IDrawable
{
    private readonly Queue<float> _seriesA;
    private readonly Queue<float> _seriesB;
    private readonly object _sync = new();
    private readonly int _capacity;

    public FastLineChartDrawable(int capacity = 120)
    {
        _capacity = Math.Max(8, capacity);
        _seriesA = new Queue<float>(_capacity);
        _seriesB = new Queue<float>(_capacity);
    }

    public int Count
    {
        get
        {
            lock (_sync)
            {
                return Math.Min(_seriesA.Count, _seriesB.Count);
            }
        }
    }

    public void AddPoints(float valueA, float valueB)
    {
        lock (_sync)
        {
            Enqueue(_seriesA, valueA);
            Enqueue(_seriesB, valueB);
        }
    }

    public void Clear()
    {
        lock (_sync)
        {
            _seriesA.Clear();
            _seriesB.Clear();
        }
    }

    public void Draw(ICanvas canvas, RectF dirtyRect)
    {
        canvas.SaveState();

        canvas.StrokeColor = Color.FromArgb("#2A3442");
        canvas.StrokeSize = 1;
        float midY = dirtyRect.Top + (dirtyRect.Height / 2f);
        canvas.DrawLine(dirtyRect.Left, midY, dirtyRect.Right, midY);

        float[] valuesA;
        float[] valuesB;
        lock (_sync)
        {
            valuesA = _seriesA.ToArray();
            valuesB = _seriesB.ToArray();
        }

        int pointCount = Math.Min(valuesA.Length, valuesB.Length);
        if (pointCount < 2)
        {
            canvas.RestoreState();
            return;
        }

        float[] combined = valuesA.Take(pointCount).Concat(valuesB.Take(pointCount)).ToArray();
        float min = combined.Min();
        float max = combined.Max();
        if (Math.Abs(max - min) < 0.001f)
        {
            max = min + 1f;
        }

        float plotLeft = dirtyRect.Left + 4;
        float plotTop = dirtyRect.Top + 4;
        float plotWidth = Math.Max(1, dirtyRect.Width - 8);
        float plotHeight = Math.Max(1, dirtyRect.Height - 8);

        DrawSeries(canvas, valuesA, pointCount, min, max, plotLeft, plotTop, plotWidth, plotHeight, Color.FromArgb("#4DD0E1"));
        DrawSeries(canvas, valuesB, pointCount, min, max, plotLeft, plotTop, plotWidth, plotHeight, Color.FromArgb("#FFB74D"));

        canvas.RestoreState();
    }

    private void Enqueue(Queue<float> series, float value)
    {
        if (series.Count >= _capacity)
        {
            series.Dequeue();
        }

        series.Enqueue(value);
    }

    private static void DrawSeries(
        ICanvas canvas,
        float[] values,
        int pointCount,
        float min,
        float max,
        float plotLeft,
        float plotTop,
        float plotWidth,
        float plotHeight,
        Color color)
    {
        canvas.StrokeColor = color;
        canvas.StrokeSize = 2;

        float stepX = plotWidth / (pointCount - 1);
        float x1 = plotLeft;
        float y1 = MapY(values[0], min, max, plotTop, plotHeight);

        for (int i = 1; i < pointCount; i++)
        {
            float x2 = plotLeft + (i * stepX);
            float y2 = MapY(values[i], min, max, plotTop, plotHeight);
            canvas.DrawLine(x1, y1, x2, y2);
            x1 = x2;
            y1 = y2;
        }
    }

    private static float MapY(float value, float min, float max, float top, float height)
    {
        float normalized = (value - min) / (max - min);
        return top + (height * (1f - normalized));
    }
}
