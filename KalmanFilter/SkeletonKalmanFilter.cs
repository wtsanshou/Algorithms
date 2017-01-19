using System;
using DotNetMatrix;
using MultiKinect.KDataModels.Factory;
using MultiKinect.KDataModels.KBones;
using MultiKinect.KDataModels.KSkeletons;
using MultiKinect.KDataModels.MatrixFactories;
namespace MultiKinect.Fusion.KalmanFilter
{
    public class SkeletonKalmanFilter : ISkeletonKalmanFilter
    {
        /// <summary>
        /// 6
        /// </summary>
        private const int DIMENSION = 6;

        private IJointKalmanFilter[] jointKalmanFilter;
        private MatrixCreater matrixCreater;
        private SkeletonCreater skeletonCreater;
        private KBonesCalculator kBonesCalculator;

        private double[] DefaultBoneLengths = 
        {
            0.065197214555122715,0.28829892337958035,0.22744449659398278,0.1701250876888423,0.22499135601400139,
            0.14972724243645125,0.28184359960643429,0.25872943402497139,0.054913588215730072,0.23001668074982837,
            0.31302215690771418,0.081221301786389283,0.080422983883569679,0.35259948661810431,0.32889695943830055,
            0.30859522806928086,0.319119080796603,0.10681108714130279,0.10246732528889699
        };

        public SkeletonKalmanFilter(IJointKalmanFilter[] kalmanFilterJoints, MatrixCreater matrixCreater, SkeletonCreater skeletonCreater, KBonesCalculator kBonesCalculator)
        {
            this.jointKalmanFilter = kalmanFilterJoints;
            //SetMeasurementErrorCovForEachJoint(); //TODO: delete me, cannot reduce mean and sd
            this.matrixCreater = matrixCreater;
            this.skeletonCreater = skeletonCreater;
            this.kBonesCalculator = kBonesCalculator;
        }

        private void SetMeasurementErrorCovForEachJoint()
        {
            jointKalmanFilter[0].setMeasurementErrorCovXYZ(0.073, 0.024, 0.08);
            jointKalmanFilter[1].setMeasurementErrorCovXYZ(0.071, 0.027, 0.079);

            jointKalmanFilter[2].setMeasurementErrorCovXYZ(0.058, 0.026, 0.056);
            jointKalmanFilter[3].setMeasurementErrorCovXYZ(0.044, 0.021, 0.036);
            jointKalmanFilter[4].setMeasurementErrorCovXYZ(0.034, 0.033, 0.045);
            jointKalmanFilter[5].setMeasurementErrorCovXYZ(0.034, 0.029, 0.041);
            jointKalmanFilter[6].setMeasurementErrorCovXYZ(0.038, 0.023, 0.037);
            jointKalmanFilter[7].setMeasurementErrorCovXYZ(0.056, 0.026, 0.057);
            jointKalmanFilter[8].setMeasurementErrorCovXYZ(0.072, 0.028, 0.069);
            jointKalmanFilter[9].setMeasurementErrorCovXYZ(0.079, 0.019, 0.067);
            jointKalmanFilter[10].setMeasurementErrorCovXYZ(0.045, 0.014, 0.052);
            jointKalmanFilter[11].setMeasurementErrorCovXYZ(0.035, 0.023, 0.039);
            jointKalmanFilter[12].setMeasurementErrorCovXYZ(0.037, 0.019, 0.038);
            jointKalmanFilter[13].setMeasurementErrorCovXYZ(0.041, 0.020, 0.043);
            jointKalmanFilter[14].setMeasurementErrorCovXYZ(0.039, 0.024, 0.04);
            jointKalmanFilter[15].setMeasurementErrorCovXYZ(0.044, 0.025, 0.039);
            jointKalmanFilter[16].setMeasurementErrorCovXYZ(0.047, 0.020, 0.037);
            jointKalmanFilter[17].setMeasurementErrorCovXYZ(0.042, 0.027, 0.033);
            jointKalmanFilter[18].setMeasurementErrorCovXYZ(0.06, 0.032, 0.056);
            jointKalmanFilter[19].setMeasurementErrorCovXYZ(0.069, 031, 0.051);

        }

        public void InitializeBoneLengths(double[] boneLengths)
        {
            this.DefaultBoneLengths = boneLengths;
        }

        public SkeletonWithTimestamp FilteASkeleton(SkeletonWithTimestamp skeleton)
        {
            SkeletonWithTimestamp filteredSkeleton = skeletonCreater.CreateAnEmptySkeleton();

            AmplifySensorErrorIfBoneLengthsOutOfRange(skeleton);

            for (int i = 0; i < KSkeletonWithInfo.JOINTNUMBER; ++i)
            {
                filteredSkeleton.Joints[i] = jointKalmanFilter[i].FilterJoint(skeleton.Joints[i], skeleton.timestamp);
            }


            filteredSkeleton.timestamp = skeleton.timestamp;
            return filteredSkeleton;
        }

        private void AmplifySensorErrorIfBoneLengthsOutOfRange(SkeletonWithTimestamp skeleton)
        {
            double[] curentBoneLengths = kBonesCalculator.GetBoneLengthsFrom(skeleton);

            double times = BoneLengthOutRangeTimes(curentBoneLengths, 0);
            AmplifyTwoJointsNTimes(0, 1, times);

            times = BoneLengthOutRangeTimes(curentBoneLengths, 1);
            AmplifyTwoJointsNTimes(1, 2, times);

            times = BoneLengthOutRangeTimes(curentBoneLengths, 2);
            AmplifyTwoJointsNTimes(2, 3, times);

            times = BoneLengthOutRangeTimes(curentBoneLengths, 3);
            AmplifyTwoJointsNTimes(3, 5, times);

            times = BoneLengthOutRangeTimes(curentBoneLengths, 4);
            AmplifyTwoJointsNTimes(4, 5, times);

            times = BoneLengthOutRangeTimes(curentBoneLengths, 5);
            AmplifyTwoJointsNTimes(5, 6, times);

            times = BoneLengthOutRangeTimes(curentBoneLengths, 6);
            AmplifyTwoJointsNTimes(6, 7, times);

            times = BoneLengthOutRangeTimes(curentBoneLengths, 7);
            AmplifyTwoJointsNTimes(7, 8, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 8);
            AmplifyTwoJointsNTimes(8, 9, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 9);
            AmplifyTwoJointsNTimes(10, 5, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 10);
            AmplifyTwoJointsNTimes(10, 11, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 11);
            AmplifyTwoJointsNTimes(11, 12, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 12);
            AmplifyTwoJointsNTimes(11, 13, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 13);
            AmplifyTwoJointsNTimes(12, 14, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 14);
            AmplifyTwoJointsNTimes(13, 15, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 15);
            AmplifyTwoJointsNTimes(14, 6, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 16);
            AmplifyTwoJointsNTimes(15, 17, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 17);
            AmplifyTwoJointsNTimes(16, 18, times);
            times = BoneLengthOutRangeTimes(curentBoneLengths, 18);
            AmplifyTwoJointsNTimes(17, 19, times);
        }

        private void AmplifyTwoJointsNTimes(int joint1, int joint2, double times)
        {
            jointKalmanFilter[joint1].AmplifySensorError(times);
            jointKalmanFilter[joint2].AmplifySensorError(times);
        }

        private double BoneLengthOutRangeTimes(double[] curentBoneLengths, int boneID)
        {
            double boneLengthOutRangeValues = Math.Abs(curentBoneLengths[boneID] - DefaultBoneLengths[boneID]);
            double times = (boneLengthOutRangeValues / DefaultBoneLengths[boneID]) + 1;
            return times;
        }

        public void SetPreviousSkeleton(SkeletonWithTimestamp skeleton)
        {
            GeneralMatrix[] jointValues = GetJointMatricesFrom(skeleton);
            GeneralMatrix initialState = GetAnEmptyMatrix();
            
            for (int i = 0; i < jointKalmanFilter.Length; ++i)
            {
                jointKalmanFilter[i].SetPreviousValues(jointValues[i], initialState, skeleton.timestamp);
            }
        }

        public SkeletonWithTimestamp PredictSkeleton(double timestamp)
        {
            SkeletonWithTimestamp predictedSkeleton = skeletonCreater.CreateAnEmptySkeleton();
            for (int i = 0; i < KSkeletonWithInfo.JOINTNUMBER; ++i)
            {
                predictedSkeleton.Joints[i] = jointKalmanFilter[i].PredictValues(timestamp);
            }
            predictedSkeleton.timestamp = timestamp;
            return predictedSkeleton;
        }
        public SkeletonWithTimestamp EstimateSkeleton(SkeletonWithTimestamp skeleton)
        {
            AmplifySensorErrorIfBoneLengthsOutOfRange(skeleton);
            SkeletonWithTimestamp estimatedSkeleton = skeletonCreater.CreateAnEmptySkeleton();
            for (int i = 0; i < KSkeletonWithInfo.JOINTNUMBER; ++i)
            {
                estimatedSkeleton.Joints[i] = jointKalmanFilter[i].EstimateJoint(skeleton.Joints[i]);
            }
            estimatedSkeleton.timestamp = skeleton.timestamp;
            return estimatedSkeleton;
        }

        private GeneralMatrix[] GetJointMatricesFrom(SkeletonWithTimestamp skeleton)
        {
            GeneralMatrix[] jointsMatrices = matrixCreater.CreateMultipleNullGeneralMatrix(KSkeletonWithInfo.JOINTNUMBER);
            for (int i = 0; i < KSkeletonWithInfo.JOINTNUMBER; ++i)
            {
                jointsMatrices[i] = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, 1);
                jointsMatrices[i].SetElement(0, 0, skeleton.Joints[i].x);
                jointsMatrices[i].SetElement(1, 0, skeleton.Joints[i].y);
                jointsMatrices[i].SetElement(2, 0, skeleton.Joints[i].z);
            }
            return jointsMatrices;
        }
        private GeneralMatrix GetAnEmptyMatrix()
        {
            return matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, DIMENSION);
        }
    }
}
