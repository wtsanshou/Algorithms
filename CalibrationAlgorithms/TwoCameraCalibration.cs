using DotNetMatrix;
using MultiKinect.KDataModels.KSkeletons;
namespace MultiKinect.Calibration.CalibrationAlgorithms
{
    public interface TwoCameraCalibration
    {
        GeneralMatrix GetCalibrationMatrix(SkeletonWithTimestamp skeletonKid, SkeletonWithTimestamp skeletonMaster);
    }
}
