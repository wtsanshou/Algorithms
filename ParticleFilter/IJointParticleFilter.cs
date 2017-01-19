using DotNetMatrix;
using MultiKinect.KDataModels.KSkeletons;
namespace MultiKinect.Fusion.ParticleFilter
{
    public interface IJointParticleFilter
    {
        void InitializeParticles(XmlOneJoint firstJoint, GeneralMatrix X_R, double time_T);
        XmlOneJoint GetAParticleFilteredJointFrom(XmlOneJoint measuredJoint, double time_T);
        GeneralMatrix[] GetSamplePoints();
    }
}
