
using System.Windows.Media.Media3D;
using DotNetMatrix;
using MultiKinect.KDataModels.KSkeletons;
namespace MultiKinect.Calibration.CalibrationAlgorithms.AbsoluteOrientationQuaternionAlgorithm
{
    public class AbsoluteOrientationQuaternion : TwoCameraCalibration
    {
        /// <summary>
        /// the number of calibrated joints
        /// </summary>
        private const int POINTNUMBER = 10;

        public GeneralMatrix GetCalibrationMatrix(SkeletonWithTimestamp skeletonKid, SkeletonWithTimestamp skeletonMaster)
        {
            //1. mean of 10 points
            Point3D[] points0 = GetTenPointFromSkeletn(skeletonKid);
            Point3D[] points1 = GetTenPointFromSkeletn(skeletonMaster);

            Point3D mean0 = MeanOfSkeleton(points0);
            Point3D mean1 = MeanOfSkeleton(points1);

            //2. remove the centroid
            Point3D[] A0 = RemoveCentroid(points0, mean0);
            Point3D[] A1 = RemoveCentroid(points1, mean1);

            //3. compute the quaternions
            GeneralMatrix M = ComputerQuaternion(A0, A1);

            //4. eigenvalues
            EigenvalueDecomposition eigen = new EigenvalueDecomposition(M);

            double[] eigenvalues = eigen.RealEigenvalues;

            double[] eigenvector = eigen.GetV().Transpose().Array[3];

            //5. 4X4 rotation matrix
            GeneralMatrix RM = EigenvalueRotationMatrix(eigenvector);

            // 3X3 rotation matrix
            GeneralMatrix rm = RM.GetMatrix(1, 3, 1, 3);

            //6. compute the scale factor
            double scale = GetScaleFactor(A0, A1, rm);
            //scale = 1.0;
            //7. compute the final transformation
            Point3D T = GetTranslation(mean1, scale, rm, mean0);

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    rm.Array[i][j] = scale * rm.Array[i][j];
                }
            }

            //the final transformation matirx
            double[][] m = new double[4][];
            m[0] = new double[4] { rm.Array[0][0], rm.Array[0][1], rm.Array[0][2], T.X };
            m[1] = new double[4] { rm.Array[1][0], rm.Array[1][1], rm.Array[1][2], T.Y };
            m[2] = new double[4] { rm.Array[2][0], rm.Array[2][1], rm.Array[2][2], T.Z };
            m[3] = new double[4] { 0, 0, 0, 1 };
            GeneralMatrix Mx = new GeneralMatrix(m);

            return Mx;
        }

        /// <summary>
        /// Get 10 Joints from a skeleton
        /// </summary>
        /// <param name="skeleton">the calibrated skeleton</param>
        /// <returns>10 Position array</returns>
        private Point3D[] GetTenPointFromSkeletn(SkeletonWithTimestamp skeleton)
        {
            Point3D[] result = new Point3D[POINTNUMBER];

            //LeftElbow, RightElbow, LeftShoulder, RightShoulder, ShoulderCentre, HipCentre, LeftHip, RightHip, LeftKnee and RightKnee
            int j = 0;
            for (int i = 2; i < KSkeletonWithInfo.JOINTNUMBER - 4; i++)
            {
                if (i != 4 && i != 9 && i != 10 && i != 8)
                {
                    result[j] = new Point3D();
                    result[j].X = skeleton.Joints[i].x;
                    result[j].Y = skeleton.Joints[i].y;
                    result[j].Z = skeleton.Joints[i].z;
                    j++;
                }
            }
            return result;
        }

        /// <summary>
        /// the mean of a skeleton
        /// </summary>
        /// <param name="joints">some joints array</param>
        /// <returns>mean</returns>
        private Point3D MeanOfSkeleton(Point3D[] joints)
        {
            double x = 0;
            double y = 0;
            double z = 0;

            for (int i = 0; i < POINTNUMBER; i++)
            {
                x += joints[i].X;
                y += joints[i].Y;
                z += joints[i].Z;
            }

            return new Point3D(x / POINTNUMBER, y / POINTNUMBER, z / POINTNUMBER);
        }

        /// <summary>
        /// minus mean from each joint
        /// </summary>
        /// <param name="joints">the joints array</param>
        /// <param name="mean">the mean of the joints array</param>
        /// <returns></returns>
        private Point3D[] RemoveCentroid(Point3D[] joints, Point3D mean)
        {
            Point3D[] result = new Point3D[POINTNUMBER];
            for (int i = 0; i < POINTNUMBER; i++)
            {
                result[i] = new Point3D();
                result[i].X = joints[i].X - mean.X;
                result[i].Y = joints[i].Y - mean.Y;
                result[i].Z = joints[i].Z - mean.Z;
            }
            return result;
        }

        /// <summary>
        /// quaternion generated from two different coordiante systems
        /// </summary>
        /// <param name="a">positions from one coordinate system</param>
        /// <param name="b">positions from another coordinate system</param>
        /// <returns>quaternion matrix</returns>
        private GeneralMatrix ComputerQuaternion(Point3D[] a, Point3D[] b)
        {
            double[][] m = new double[4][];
            m[0] = new double[4] { 0, 0, 0, 0 };
            m[1] = new double[4] { 0, 0, 0, 0 };
            m[2] = new double[4] { 0, 0, 0, 0 };
            m[3] = new double[4] { 0, 0, 0, 0 };
            GeneralMatrix result = new GeneralMatrix(m);
            GeneralMatrix Ma;
            GeneralMatrix Mb;

            for (int i = 0; i < POINTNUMBER; i++)
            {
                double[][] ma = new double[4][];
                ma[0] = new double[4] { 0, -a[i].X, -a[i].Y, -a[i].Z };
                ma[1] = new double[4] { a[i].X, 0, a[i].Z, -a[i].Y };
                ma[2] = new double[4] { a[i].Y, -a[i].Z, 0, a[i].X };
                ma[3] = new double[4] { a[i].Z, a[i].Y, -a[i].X, 0 };
                Ma = new GeneralMatrix(ma);
                Ma = Ma.Transpose();


                double[][] mb = new double[4][];
                mb[0] = new double[4] { 0, -b[i].X, -b[i].Y, -b[i].Z };
                mb[1] = new double[4] { b[i].X, 0, -b[i].Z, b[i].Y };
                mb[2] = new double[4] { b[i].Y, b[i].Z, 0, -b[i].X };
                mb[3] = new double[4] { b[i].Z, -b[i].Y, b[i].X, 0 };
                Mb = new GeneralMatrix(mb);

                GeneralMatrix mx = Ma * Mb;
                result += mx;
            }

            return result;
        }

        private double[] CreateDoubleArr(double a1, double a2, double a3, double a4)
        {
            return new double[4] { a1, a2, a3, a4 };
        }

        /// <summary>
        /// Eigenvalue of a Rotation Matrix
        /// </summary>
        /// <param name="e">eigen vector</param>
        /// <returns></returns>
        private GeneralMatrix EigenvalueRotationMatrix(double[] e)
        {
            double[][] m1 = new double[4][];
            m1[0] = new double[4] { e[0], -e[1], -e[2], -e[3] };
            m1[1] = new double[4] { e[1], e[0], e[3], -e[2] };
            m1[2] = new double[4] { e[2], -e[3], e[0], e[1] };
            m1[3] = new double[4] { e[3], e[2], -e[1], e[0] };
            GeneralMatrix M1 = new GeneralMatrix(m1);
            M1 = M1.Transpose();

            double[][] m2 = new double[4][];
            m2[0] = new double[4] { e[0], -e[1], -e[2], -e[3] };
            m2[1] = new double[4] { e[1], e[0], -e[3], e[2] };
            m2[2] = new double[4] { e[2], e[3], e[0], -e[1] };
            m2[3] = new double[4] { e[3], -e[2], e[1], e[0] };
            GeneralMatrix M2 = new GeneralMatrix(m2);

            return M1 * M2;
        }

        /// <summary>
        /// row vector * rotation matirx * column vector
        /// </summary>
        /// <param name="b"></param>
        /// <param name="R"></param>
        /// <param name="a"></param>
        /// <returns></returns>
        private double BRB(Point3D b, GeneralMatrix R, Point3D a)
        {
            double x = b.X * R.Array[0][0] + b.Y * R.Array[1][0] + b.Z * R.Array[2][0];
            double y = b.X * R.Array[0][1] + b.Y * R.Array[1][1] + b.Z * R.Array[2][1];
            double z = b.X * R.Array[0][2] + b.Y * R.Array[1][2] + b.Z * R.Array[2][2];

            return x * a.X + y * a.Y + z * a.Z;
        }

        /// <summary>
        /// product of a vector itself
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        private double BB(Point3D b)
        {
            return b.X * b.X + b.Y * b.Y + b.Z * b.Z;
        }

        /// <summary>
        /// Get Scale Factor of two coordinate system
        /// </summary>
        /// <param name="A0">position of one coordinate system</param>
        /// <param name="A1">position of another coordinate system</param>
        /// <param name="R">rotation matrix</param>
        /// <returns></returns>
        private double GetScaleFactor(Point3D[] A0, Point3D[] A1, GeneralMatrix R)
        {
            double a = 0;
            double b = 0;

            for (int i = 0; i < POINTNUMBER; i++)
            {
                a += BRB(A1[i], R, A0[i]);
                b += BB(A1[i]);
            }

            return b / a;
        }

        /// <summary>
        /// Get Translation vector of two coordinate system
        /// </summary>
        /// <param name="cb">  --> mean vector 1  </param>
        /// <param name="s"> scale</param>
        /// <param name="R"> rotation matrix</param>
        /// <param name="ca">      mean vector 0 <-- </param>
        /// <returns></returns>
        private Point3D GetTranslation(Point3D cb, double s, GeneralMatrix R, Point3D ca)
        {
            double[,] r = {{0,0,0},
                            {0,0,0},
                            {0,0,0}
                            };
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    r[i, j] = s * R.Array[i][j];
                }
            }

            double x = cb.X - (r[0, 0] * ca.X + r[0, 1] * ca.Y + r[0, 2] * ca.Z);
            double y = cb.Y - (r[1, 0] * ca.X + r[1, 1] * ca.Y + r[1, 2] * ca.Z);
            double z = cb.Z - (r[2, 0] * ca.X + r[2, 1] * ca.Y + r[2, 2] * ca.Z);

            return new Point3D(x, y, z);
        }
    }
}
