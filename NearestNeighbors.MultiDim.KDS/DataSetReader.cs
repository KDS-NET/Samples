using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

#nullable enable

namespace NearestNeighbors.MultiDim.KDS
{
    public class Data
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double T { get; set; }
    }

    public class Point
    {
        public Data[] Data { get; set; } = global::System.Array.Empty<global::NearestNeighbors.MultiDim.KDS.Data>();
        public string File { get; set; } = "";
    }

    public static class DataSetReader
    {
        private static IEnumerable<(T, T)> GetPairs<T>(this IEnumerable<T> Elements)
        {
            List<(T, T)> result = new();
            for (int i = 0; i < Elements.Count(); i++)
            {
                for (int j = i; j < Elements.Count(); j++)
                {
                    result.Add((Elements.ElementAt(i), Elements.ElementAt(j)));
                }
            }
            return result;
        }

        public static Point[] ReadDataSet(IEnumerable<string> files)
        {
            List<Point> points = new();
            foreach (string? file in files)
            {
                Point pt = new();
                pt.File = file;
                List<Data> ds = new();

                foreach (string? line in File.ReadAllLines(file))
                {
                    IEnumerable<string>? elements = line.Replace(".", NumberFormatInfo.CurrentInfo.NumberDecimalSeparator).Replace("\t", " ").Split(' ').Where(x => !string.IsNullOrWhiteSpace(x));
                    ds.Add(new Data() { T = double.Parse(elements.ElementAt(0)), X = double.Parse(elements.ElementAt(1)), Y = double.Parse(elements.ElementAt(2)) });
                }
                pt.Data = ds.ToArray();
                points.Add(pt);
            }
            return points.ToArray();
        }

        public static double GetMaxSpeed(Point[] points)
        {
            List<double> speeds = new();
            foreach (Point? data in points)
            {
                for (int i = 0; i < data.Data.Length - 1; i++)
                {
                    Data pos1 = data.Data[i];
                    Data pos2 = data.Data[i + 1];
                    double dist = Math.Abs(Math.Sqrt(Math.Pow(pos1.X, 2) + Math.Pow(pos1.Y, 2)) - Math.Sqrt(Math.Pow(pos2.X, 2) + Math.Pow(pos2.Y, 2)));
                    double t = Math.Abs(pos1.T - pos2.T);
                    speeds.Add(dist / t);
                }
            }
            return speeds.Max();
        }
    }
}
