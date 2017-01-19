using System;
using MultiKinect.KDataModels.KSkeletons;
namespace MultiKinect.Fusion.KalmanFilter
{
    public interface ISkeletonKalmanFilter
    {
        void InitializeBoneLengths(double[] boneLengths);

        SkeletonWithTimestamp FilteASkeleton(SkeletonWithTimestamp skeleton);

        SkeletonWithTimestamp EstimateSkeleton(SkeletonWithTimestamp skeleton);
        SkeletonWithTimestamp PredictSkeleton(double timestamp);

        void SetPreviousSkeleton(SkeletonWithTimestamp skeleton);
    }
}
