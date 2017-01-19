
using DotNetMatrix;
using MultiKinect.KDataModels.KSkeletons;
namespace MultiKinect.Fusion.KalmanFilter
{
    public interface IJointKalmanFilter
    {
        void setMeasurementErrorCovXYZ(double Ex, double Ey, double Ez);

        XmlOneJoint FilterJoint(XmlOneJoint joint, double time_T);

        XmlOneJoint EstimateJoint(XmlOneJoint joint);
        XmlOneJoint PredictValues(double time_T);

        void SetPreviousValues(GeneralMatrix goal_X, GeneralMatrix state_P, double time_T);

        void AmplifySensorError(double times);
    }
}
