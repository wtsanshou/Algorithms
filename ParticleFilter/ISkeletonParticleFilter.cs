using DotNetMatrix;
using MultiKinect.KDataModels.KSkeletons;
namespace MultiKinect.Fusion.ParticleFilter
{
    public interface ISkeletonParticleFilter
    {
        void Initialize(SkeletonWithTimestamp skeleton);
        SkeletonWithTimestamp FilteASkeleton(SkeletonWithTimestamp skeleton);
        XmlOneJoint[] GetJointSamples(int j);
    }
}
