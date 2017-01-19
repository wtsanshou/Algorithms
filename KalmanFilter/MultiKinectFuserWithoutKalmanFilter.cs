using System;
using System.Windows.Media.Media3D;
using DotNetMatrix;
using MultiKinect.Fusion.KalmanFilter;
using MultiKinect.Fusion.SkeletonDirection;
using MultiKinect.Fusion.SkeletonSwitch;
using MultiKinect.KDataModels.Factory;
using MultiKinect.KDataModels.KSkeletons;
using MultiKinect.KDataModels.MatrixFactories;

namespace MultiKinect.Fusion.MultiKinectFusion
{
    public class MultiKinectFuserWithoutFilter : IMultiKinectFuser
    {
        private SkeletonWithTimestamp[] _oriSkeletonArray;

        private SkeletonWithTimestamp[] _calibratedSkeletonArray;

        private SkeletonWithTimestamp oldSkeleton;

        private bool _isFirst = true;

        /// <summary>
        /// the correct percentage of the 4 skeletons
        /// </summary>
        private int[] skeletonsConfidences;

        /// <summary>
        /// the 4 kinect's percentage of facing
        /// </summary>
        private double[] FrontSkeletonsConfidences;

        private GeneralMatrix[] transforMatrics;

        /// <summary>
        /// 3
        /// </summary>
        private const int TRANSFORMATRICSNUMBER = 3;

        /// <summary>
        /// 4
        /// </summary>
        private const int SKELETONNUMBER = 4;
        /// <summary>
        /// 2
        /// </summary>
        private int WEIGHT = 2;
        /// <summary>
        /// -20
        /// </summary>
        private static int FRONTWEIGHT = -20;
        /// <summary>
        /// -0.75
        /// </summary>
        private const double MAXROTATION = -0.75;

        private const int RIGHT_SHOULDER = 3;
        private const int LEFT_SHOULDER = 6;

        private SkeletonSourseState preSkeletonState;
        private SkeletonSourseState skeletonState0;
        private SkeletonSourseState skeletonState1;
        private SkeletonSourseState skeletonState2;
        private SkeletonSourseState skeletonState3;

        private SkeletonCopierAndCreater skeletonManager;
        private IFrontSkeletonConfidencesCollector frontSkeletonConfidencesCollector;
        private ISkeletonKalmanFilter skeletonKalmanFilter;
        private JointCreater jointCreater;
        private MatrixCreater matrixCreater;
        private SkeletonStateCreater skeletonSateCreater;

        public MultiKinectFuserWithoutFilter(SkeletonCopierAndCreater skeletonManager,
                                                      JointCreater jointCreater,
                                                      IFrontSkeletonConfidencesCollector frontSkeletonConfidencesCollector,
                                                      ISkeletonKalmanFilter skeletonKalmanFilter,
                                                      MatrixCreater matrixCreater,
                                                      SkeletonStateCreater skeletonSateCreater)
        {
            this.skeletonManager = skeletonManager;
            this.jointCreater = jointCreater;
            this.frontSkeletonConfidencesCollector = frontSkeletonConfidencesCollector;
            this.skeletonKalmanFilter = skeletonKalmanFilter;
            this.matrixCreater = matrixCreater;
            this.skeletonSateCreater = skeletonSateCreater;

            this.transforMatrics = matrixCreater.CreateMultipleNullGeneralMatrix(TRANSFORMATRICSNUMBER);
        }

        public void SetTransferMatrices(GeneralMatrix[] calibratedMatrix)
        {
            this.transforMatrics = calibratedMatrix;
        }

        public SkeletonWithTimestamp Fusion(SkeletonWithTimestamp[] skeletonArray)
        {
            InitializeOriSkeletonAndCalibratedSkeleton(skeletonArray);

            if (_isFirst)
            {
                InitializeSkeletonStates();
                InitializeTheFirstOldSkeletonData();

                return null;
            }
            TransferSkeletonsToMainKinectSystem();


            CollectSkeletonsConfidences();

            CollectFrontSkeletonsConfidences();

            ReduceBackSkeletonConfidences();

            return FuseSkeletons();
        }
        private void InitializeOriSkeletonAndCalibratedSkeleton(SkeletonWithTimestamp[] skeletonArray)
        {
            this._oriSkeletonArray = skeletonManager.CreateMultipleEmptySkeleton(SKELETONNUMBER);

            for (int i = 0; i < SKELETONNUMBER; ++i)
            {
                this._oriSkeletonArray[i] = (IsEmptySkeleton(skeletonArray[i]))
                                            ? skeletonManager.CreateAnEmptySkeleton()
                                            : skeletonArray[i];
            }

            this._calibratedSkeletonArray = skeletonManager.CreateMultipleEmptySkeleton(SKELETONNUMBER);
            this._calibratedSkeletonArray[KSkeletonWithInfo.SKELETONID0] = this.skeletonManager.CopySkeleton(this._oriSkeletonArray[KSkeletonWithInfo.SKELETONID0]);
        }
        private bool IsEmptySkeleton(SkeletonWithTimestamp oneSkeleton)
        {
            return oneSkeleton == null || oneSkeleton.Joints == null;
        }

        private void InitializeSkeletonStates()
        {
            skeletonState0 = skeletonSateCreater.GetSkeletonState(0);
            skeletonState1 = skeletonSateCreater.GetSkeletonState(1);
            skeletonState2 = skeletonSateCreater.GetSkeletonState(2);
            skeletonState3 = skeletonSateCreater.GetSkeletonState(3);
            preSkeletonState = skeletonState0;
        }

        private void InitializeTheFirstOldSkeletonData()
        {
            this.oldSkeleton = this.skeletonManager.CopySkeleton(this._oriSkeletonArray[KSkeletonWithInfo.SKELETONID0]);
            FusionIsRunning();
        }

        private void FusionIsRunning()
        {
            _isFirst = false;
        }

        public void FusionIsFinished()
        {
            _isFirst = true;
        }

        private void TransferSkeletonsToMainKinectSystem()
        {
            for (int j = 1; j < SKELETONNUMBER; j++)
            {
                this._calibratedSkeletonArray[j] = this.skeletonManager.CreateAnEmptySkeleton();
            }

            TransferFromSkeleton(KSkeletonWithInfo.SKELETONID1);
            TransferFromSkeleton(KSkeletonWithInfo.SKELETONID2);
            TransferFromSkeleton(KSkeletonWithInfo.SKELETONID3);
        }

        private void TransferFromSkeleton(int KinectB)
        {
            for (int i = 0; i < KSkeletonWithInfo.JOINTNUMBER; i++)
            {
                double[][] s1 = new double[1][];
                s1[0] = new double[] 
                            { 
                                this._oriSkeletonArray[KinectB].Joints[i].x,
                                this._oriSkeletonArray[KinectB].Joints[i].y,    
                                this._oriSkeletonArray[KinectB].Joints[i].z, 
                                1 
                            };
                GeneralMatrix S1 = matrixCreater.CreatAGeneralMatrixFrom(s1);
                S1 = S1.Transpose();

                S1 = this.transforMatrics[KinectB - 1] * S1;

                this._calibratedSkeletonArray[KinectB].Joints[i] = jointCreater.CreateAJoint
                                                                        (
                                                                            S1.Array[0][0],
                                                                            S1.Array[1][0],
                                                                            S1.Array[2][0],
                                                                            this._oriSkeletonArray[KinectB].Joints[i].t
                                                                        );
            }
        }

        

        private void CollectSkeletonsConfidences()
        {
            this.skeletonsConfidences = new int[SKELETONNUMBER];
            this.skeletonsConfidences = KSkeletonWithInfo.GetSkeletonConfidence(this._oriSkeletonArray);
        }

        private void CollectFrontSkeletonsConfidences()
        {
            this.FrontSkeletonsConfidences = this.frontSkeletonConfidencesCollector.GetFrontBackConfidences(this.skeletonsConfidences, this._oriSkeletonArray);
        }

        private void ReduceBackSkeletonConfidences()
        {
            if (FrontBack(2, 0))
                this.skeletonsConfidences[KSkeletonWithInfo.SKELETONID0] -= WEIGHT;
            if (FrontBack(3, 1))
                this.skeletonsConfidences[KSkeletonWithInfo.SKELETONID1] -= WEIGHT;
            if (FrontBack(0, 2))
                this.skeletonsConfidences[KSkeletonWithInfo.SKELETONID2] -= WEIGHT;
            if (FrontBack(1, 3))
                this.skeletonsConfidences[KSkeletonWithInfo.SKELETONID3] -= WEIGHT;
        }

        protected bool FrontBack(int front, int back)
        {
            bool backSkeletonRotationAngleIsTooLarge = GetRotationAngle(back) < MAXROTATION;
            bool frontIsTrue = this.FrontSkeletonsConfidences[front] - this.FrontSkeletonsConfidences[back] >= FRONTWEIGHT;
            return frontIsTrue && backSkeletonRotationAngleIsTooLarge;
        }

        private double GetRotationAngle(int back)
        {
            //prevent front back switch problem
            Point3D newShouler = GetVectorFromTwoJoints(_calibratedSkeletonArray[back].Joints[RIGHT_SHOULDER], _calibratedSkeletonArray[back].Joints[LEFT_SHOULDER]);
            Point3D oldShouler = GetVectorFromTwoJoints(this.oldSkeleton.Joints[RIGHT_SHOULDER], this.oldSkeleton.Joints[LEFT_SHOULDER]);

            double ab = ProductOfTwoVectors(newShouler, oldShouler);
            double a = Math.Sqrt(ProductOfTwoVectors(newShouler, newShouler));
            double b = Math.Sqrt(ProductOfTwoVectors(oldShouler, oldShouler));

            return ab / (a * b);
        }
        private Point3D GetVectorFromTwoJoints(XmlOneJoint joint1, XmlOneJoint joint2)
        {
            return new Point3D(joint1.x - joint2.x, joint1.y - joint2.y, joint1.z - joint2.z);
        }
        private double ProductOfTwoVectors(Point3D v1, Point3D v2)
        {
            return v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z;
        }

        private SkeletonWithTimestamp FuseSkeletons()
        {
            preSkeletonState.InitializeSourse(this.skeletonsConfidences, this.FrontSkeletonsConfidences, this._calibratedSkeletonArray, oldSkeleton);

            SkeletonWithTimestamp fusedSkeleton = GetFusedSkeleton();

            this.oldSkeleton = fusedSkeleton;

            return fusedSkeleton;
        }

        private SkeletonWithTimestamp GetFusedSkeleton()
        {
            SkeletonWithTimestamp fusedSkeleton;
            if (TheBestSkeletonIs(KSkeletonWithInfo.SKELETONID0))
            {
                fusedSkeleton = preSkeletonState.SwitchToKinect0();
                preSkeletonState = skeletonState0;
            }
            else if (TheBestSkeletonIs(KSkeletonWithInfo.SKELETONID1))
            {
                fusedSkeleton = preSkeletonState.SwitchToKinect1();
                preSkeletonState = skeletonState1;
            }
            else if (TheBestSkeletonIs(KSkeletonWithInfo.SKELETONID2))
            {
                fusedSkeleton = preSkeletonState.SwitchToKinect2();
                preSkeletonState = skeletonState2;
            }
            else
            {
                fusedSkeleton = preSkeletonState.SwitchToKinect3();
                preSkeletonState = skeletonState3;
            }

            fusedSkeleton.timestamp = _oriSkeletonArray[0].timestamp;

            return fusedSkeleton;
        }

        private bool TheBestSkeletonIs(int target)
        {
            for (int i = 0; i < SKELETONNUMBER; ++i)
                if (this.skeletonsConfidences[target] < this.skeletonsConfidences[i])
                    return false;
            return true;
        }

        public Point3D[] GetJointSamples(int j)
        {
            throw new NotImplementedException();
        }


        XmlOneJoint[] IMultiKinectFuser.GetJointSamples(int j)
        {
            return null;
        }
    }
}
