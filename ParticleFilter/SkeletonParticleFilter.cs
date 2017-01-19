using System.Windows;
using System.Windows.Media.Media3D;
using DotNetMatrix;
using MultiKinect.KDataModels.Factory;
using MultiKinect.KDataModels.KSkeletons;
using MultiKinect.KDataModels.MatrixFactories;
namespace MultiKinect.Fusion.ParticleFilter
{
    public class SkeletonParticleFilter : ISkeletonParticleFilter
    {
        public const int DIMENSION = 6;
        private IJointParticleFilter[] jointParticleFilters;
        private SkeletonCreater skeletonCreater;
        private MatrixCreater matrixCreater;

        public SkeletonParticleFilter(IJointParticleFilter[] jointParticleFilters, MatrixCreater matrixCreater, SkeletonCreater skeletonCreater)
        {
            this.jointParticleFilters = jointParticleFilters;
            this.skeletonCreater = skeletonCreater;
            this.matrixCreater = matrixCreater;
        }

        public void Initialize(SkeletonWithTimestamp skeleton)
        {
            double[][] xr = Get2DdoubleArray(0.06,0.06,0.06,0,0,0); // deviation
            GeneralMatrix X_R = matrixCreater.CreatAGeneralMatrixFrom(xr);
            for(int i=0; i<KSkeletonWithInfo.JOINTNUMBER; ++i)
            {
                jointParticleFilters[i].InitializeParticles(skeleton.Joints[i], X_R, skeleton.timestamp);
            }
        }

        public SkeletonWithTimestamp FilteASkeleton(SkeletonWithTimestamp skeleton)
        {
            SkeletonWithTimestamp filteredSkeleton = skeletonCreater.CreateAnEmptySkeleton();

            //AmplifySensorErrorIfBoneLengthsOutOfRange(skeleton);

            for (int i = 0; i < KSkeletonWithInfo.JOINTNUMBER; ++i)
            {
                filteredSkeleton.Joints[i] = jointParticleFilters[i].GetAParticleFilteredJointFrom(skeleton.Joints[i], skeleton.timestamp);
            }

            filteredSkeleton.timestamp = skeleton.timestamp;
            return filteredSkeleton;
        }

        public XmlOneJoint[] GetJointSamples(int j)
        {
            GeneralMatrix[] smaples = jointParticleFilters[j].GetSamplePoints();
            return converToPoints(smaples);
        }

        private XmlOneJoint[] converToPoints(GeneralMatrix[] smaples)
        {
            int size = smaples.Length;
            XmlOneJoint[] points = new XmlOneJoint[size];
            for (int i = 0; i < size; ++i)
            {
                points[i] = new XmlOneJoint();
                points[i].x = smaples[i].GetElement(0, 0);
                points[i].y = smaples[i].GetElement(1, 0);
                points[i].z = smaples[i].GetElement(2, 0);
            }
            return points;
        }

        private double[][] Get2DdoubleArray(double d1, double d2, double d3, double d4, double d5, double d6)
        {
            double[][] XNdoubleValues = new double[DIMENSION][] { 
                                                                        new double []{ d1 },
                                                                        new double []{ d2 },
                                                                        new double []{ d3 },
                                                                        new double []{ d4 },
                                                                        new double []{ d5 },
                                                                        new double []{ d6 }
                                                                        };
            return XNdoubleValues;
        }
    }
}
