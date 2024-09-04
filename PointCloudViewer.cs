using System;
using System.Collections.Generic;
using System.IO;
using NXOpen;

namespace WednesdayTaskNX
{
    public class PointCloudViewer
    {
        public static void Main()
        {
            try
            {
                // Get the NX session
                Session session = Session.GetSession();
                if (session == null)
                {
                    Console.WriteLine("Failed to get NX session.");
                    return;
                }

                // Get the active part
                Part workPart = session.Parts.Work;
                if (workPart == null)
                {
                    session.ListingWindow.WriteLine("No active part found. Please open a part.");
                    return;
                }

                // Load the point cloud data
                string filePath = @"C:\Users\diksh\Desktop\open3d_data\download\BunnyMesh\BunnyMesh.ply";
                List<Point3d> points = LoadPointCloud(filePath);

                // Check if points are loaded
                if (points.Count == 0)
                {
                    session.ListingWindow.WriteLine("No points loaded from file.");
                    return;
                }

                session.ListingWindow.WriteLine("Loaded " + points.Count + " points from file.");

                // Visualize the point cloud
                VisualizePointCloud(workPart, points);

                // Calculate and visualize bounding box
                BoundingBox boundingBox = CalculateBoundingBox(points);
                VisualizeBoundingBox(workPart, boundingBox);

                // Cluster points
                List<List<Point3d>> clusters = ClusterPointCloud(points, boundingBox, 5);
                session.ListingWindow.WriteLine("Generated " + clusters.Count + " clusters.");

                // Keep NX Open running
                session.ListingWindow.Open();
                session.ListingWindow.WriteLine("Point cloud processing complete. Press any key to exit.");
                Console.ReadKey();
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error: " + ex.Message);
            }
        }

        static List<Point3d> LoadPointCloud(string filePath)
        {
            List<Point3d> points = new List<Point3d>();
            bool headerParsed = false;
            int vertexCount = 0;

            try
            {
                using (StreamReader reader = new StreamReader(filePath))
                {
                    string line;
                    while ((line = reader.ReadLine()) != null)
                    {
                        if (line.StartsWith("element vertex"))
                        {
                            // Extract vertex count from header
                            string[] parts = line.Split(' ');
                            if (parts.Length == 3)
                            {
                                vertexCount = int.Parse(parts[2]);
                            }
                        }
                        else if (line.StartsWith("end_header"))
                        {
                            headerParsed = true;
                            continue;
                        }

                        if (headerParsed)
                        {
                            // Parse point data
                            string[] parts = line.Split(' ');
                            if (parts.Length >= 3 && vertexCount > 0)
                            {
                                double x = double.Parse(parts[0]);
                                double y = double.Parse(parts[1]);
                                double z = double.Parse(parts[2]);
                                points.Add(new Point3d(x, y, z));
                                vertexCount--;
                                if (vertexCount <= 0) break;
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error reading point cloud file: " + ex.Message);
            }

            return points;
        }

        static void VisualizePointCloud(Part workPart, List<Point3d> points)
        {
            foreach (Point3d point in points)
            {
                try
                {
                    // Create a point in the NX model
                    Point nxPoint = workPart.Points.CreatePoint(point);
                    nxPoint.SetVisibility(SmartObject.VisibilityOption.Visible);
                }
                catch (Exception ex)
                {
                    // Log errors creating points
                    Session.GetSession().ListingWindow.WriteLine("Error creating point: " + ex.Message);
                }
            }
        }

        static BoundingBox CalculateBoundingBox(List<Point3d> points)
        {
            double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;

            foreach (Point3d point in points)
            {
                if (point.X < minX) minX = point.X;
                if (point.Y < minY) minY = point.Y;
                if (point.Z < minZ) minZ = point.Z;

                if (point.X > maxX) maxX = point.X;
                if (point.Y > maxY) maxY = point.Y;
                if (point.Z > maxZ) maxZ = point.Z;
            }

            return new BoundingBox(new Point3d(minX, minY, minZ), new Point3d(maxX, maxY, maxZ));
        }

        static void VisualizeBoundingBox(Part workPart, BoundingBox boundingBox)
        {
            // Create corner points of the bounding box
            List<Point3d> boxCorners = new List<Point3d>
            {
                boundingBox.Min,
                new Point3d(boundingBox.Max.X, boundingBox.Min.Y, boundingBox.Min.Z),
                new Point3d(boundingBox.Max.X, boundingBox.Max.Y, boundingBox.Min.Z),
                new Point3d(boundingBox.Min.X, boundingBox.Max.Y, boundingBox.Min.Z),
                new Point3d(boundingBox.Min.X, boundingBox.Min.Y, boundingBox.Max.Z),
                new Point3d(boundingBox.Max.X, boundingBox.Min.Y, boundingBox.Max.Z),
                new Point3d(boundingBox.Max.X, boundingBox.Max.Y, boundingBox.Max.Z),
                new Point3d(boundingBox.Min.X, boundingBox.Max.Y, boundingBox.Max.Z)
            };

            // Visualize edges of the bounding box
            for (int i = 0; i < 4; i++)
            {
                CreateLine(workPart, boxCorners[i], boxCorners[(i + 1) % 4]); // Bottom edges
                CreateLine(workPart, boxCorners[i + 4], boxCorners[(i + 1) % 4 + 4]); // Top edges
                CreateLine(workPart, boxCorners[i], boxCorners[i + 4]); // Vertical edges
            }
        }

        static void CreateLine(Part workPart, Point3d start, Point3d end)
        {
            Line line = workPart.Curves.CreateLine(start, end);
            line.SetVisibility(SmartObject.VisibilityOption.Visible);
        }

        static List<List<Point3d>> ClusterPointCloud(List<Point3d> points, BoundingBox boundingBox, int gridSize)
        {
            // Simple grid-based clustering within the bounding box
            double cellSizeX = (boundingBox.Max.X - boundingBox.Min.X) / gridSize;
            double cellSizeY = (boundingBox.Max.Y - boundingBox.Min.Y) / gridSize;
            double cellSizeZ = (boundingBox.Max.Z - boundingBox.Min.Z) / gridSize;

            Dictionary<int, Dictionary<int, Dictionary<int, List<Point3d>>>> clusters = new Dictionary<int, Dictionary<int, Dictionary<int, List<Point3d>>>>();

            foreach (Point3d point in points)
            {
                int xIndex = (int)((point.X - boundingBox.Min.X) / cellSizeX);
                int yIndex = (int)((point.Y - boundingBox.Min.Y) / cellSizeY);
                int zIndex = (int)((point.Z - boundingBox.Min.Z) / cellSizeZ);

                if (!clusters.ContainsKey(xIndex))
                {
                    clusters[xIndex] = new Dictionary<int, Dictionary<int, List<Point3d>>>();
                }

                if (!clusters[xIndex].ContainsKey(yIndex))
                {
                    clusters[xIndex][yIndex] = new Dictionary<int, List<Point3d>>();
                }

                if (!clusters[xIndex][yIndex].ContainsKey(zIndex))
                {
                    clusters[xIndex][yIndex][zIndex] = new List<Point3d>();
                }

                clusters[xIndex][yIndex][zIndex].Add(point);
            }

            List<List<Point3d>> clusterList = new List<List<Point3d>>();
            foreach (var xDict in clusters.Values)
            {
                foreach (var yDict in xDict.Values)
                {
                    foreach (var zDict in yDict.Values)
                    {
                        clusterList.Add(zDict); // Directly add the List<Point3d>
                    }
                }
            }

            return clusterList;
        }

    }

    public class BoundingBox
    {
        public Point3d Min { get; private set; }
        public Point3d Max { get; private set; }

        public BoundingBox(Point3d min, Point3d max)
        {
            Min = min;
            Max = max;
        }
    }
}
