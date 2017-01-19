using System;
using DotNetMatrix;
using MultiKinect.KDataModels.Factory;
using MultiKinect.KDataModels.KSkeletons;
using MultiKinect.KDataModels.MatrixFactories;

namespace MultiKinect.Fusion.ParticleFilter
{
    public class JointParticleFilter : IJointParticleFilter
    {
        /// <summary>
        /// 6
        /// </summary>
        private const int DIMENSION = 6;
        /// <summary>
        /// 100
        /// </summary>
        private const int SAMPLE_SIZE = 100;
        private double q = 0.5;
        /// <summary>
        /// 0.05
        /// </summary>
        private const double DEFAULT_TIME_INTERVAL = 0.05; //seconds;
        /// <summary>
        /// 1000
        /// </summary>
        private const double ONE_SECOND = 1000; // one second = 1000 million seconds
        private double deltaT;
        private double curTime;
        private double previousTime;

        private Random rand;
        private GeneralMatrix[] X_P;
        private GeneralMatrix F;
        private GeneralMatrix X_N; // standard deviation, Noise of process
        private GeneralMatrix X_R; // standard deviation, Noise of measurement
        private GeneralMatrix[] X_P_Update;
        private GeneralMatrix[] Z_P_Update;
        private double[] P_W;
        private double[] joint;

        private JointCreater jointCreater;
        private MatrixCreater matrixCreater;

        public JointParticleFilter(JointCreater jointCreater, MatrixCreater matrixCreater)
        {
            rand = new Random(); //reuse this if you are generating many
            this.jointCreater = jointCreater;
            this.matrixCreater = matrixCreater;

            InitializeParticleParameters();
        }
        private void InitializeParticleParameters()
        {
            X_P = matrixCreater.CreateMultipleEmpty_MN_Matrices(SAMPLE_SIZE, DIMENSION, 1); //vector of particles
            X_P_Update = matrixCreater.CreateMultipleEmpty_MN_Matrices(SAMPLE_SIZE, DIMENSION, 1); //vector of updated predicted particles
            Z_P_Update = matrixCreater.CreateMultipleEmpty_MN_Matrices(SAMPLE_SIZE, DIMENSION, 1); //vector of updated measured particles
            F = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, DIMENSION); //predict matrix
            X_N = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, 1); //Noise of process
            X_R = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, 1);   //Noise of measurement
            P_W = new double[SAMPLE_SIZE];
        }

        public void InitializeParticles(XmlOneJoint firstJoint, GeneralMatrix X_R, double time_T)
        {
            SetPreviousTime(time_T);
            this.X_R = X_R;
            for (int i = 0; i < SAMPLE_SIZE; ++i)
            {
                X_P[i].SetElement(0, 0, NormalRandom(firstJoint.x, X_R.GetElement(0, 0)));
                X_P[i].SetElement(1, 0, NormalRandom(firstJoint.y, X_R.GetElement(1, 0)));
                X_P[i].SetElement(2, 0, NormalRandom(firstJoint.z, X_R.GetElement(2, 0)));
            }
        }

        public XmlOneJoint GetAParticleFilteredJointFrom(XmlOneJoint measuredJoint, double time_T)
        {
            SetCurrentTime(time_T);
            SetDeltaT(GetCurrentTime() - GetPreviousTime());

            SetTransitionMatrix_F();

            UpdateProcessError_X_N(GetDeltaT());

            joint = new double[] { measuredJoint.x, measuredJoint.y, measuredJoint.z };

            P_W = UpdatePredictWights_P_W();
            ResampleParticles_X_P();

            SetPreviousTime(time_T); // TODO: is X_R constant?
            Update_X_R(GetDeltaT());

            //get the joint
            return jointCreater.CreateAJoint(Mean(X_P, 0), Mean(X_P, 1), Mean(X_P, 2), KSkeletonWithInfo.PARTICLE_FILTERJOINT);
        }
        public GeneralMatrix[] GetSamplePoints()
        {
            return X_P;
        }

        private void SetDeltaT(double timeInterval)
        {
            this.deltaT = (timeInterval <= 0) ? DEFAULT_TIME_INTERVAL : timeInterval / ONE_SECOND;
        }

        private double GetDeltaT()
        {
            return this.deltaT;
        }
        private void SetTransitionMatrix_F()
        {
            for (int i = 0; i < DIMENSION; ++i)
                this.F.SetElement(i, i, 1);
            for (int row = 0; row < 3; ++row)
                this.F.SetElement(row, row + 3, GetDeltaT());
        }
        private GeneralMatrix GetTransitionMatrix_F()
        {
            return this.F;
        }
        
        private void SetCurrentTime(double time_T)
        {
            this.curTime = time_T;
        }
        private double GetCurrentTime()
        {
            return this.curTime;
        }
        private double GetPreviousTime()
        {
            return this.previousTime;
        }
        private void SetPreviousTime(double time_T)
        {
            this.previousTime = time_T;
        }
        

        private void Update_X_R(double T)
        {
            X_R.SetElement(3, 0, X_R.GetElement(0, 0) / (T * T));
            X_R.SetElement(4, 0, X_R.GetElement(1, 0) / (T * T));
            X_R.SetElement(5, 0, X_R.GetElement(2, 0) / (T * T));
        }

        private void UpdateProcessError_X_N(double T)
        {
            double qWeight = q * T;
            double pointWight = 1 + T * T;
            double speedWeight = 1;
            double[] ds = {qWeight * pointWight, qWeight * pointWight, qWeight * pointWight,
                            qWeight * speedWeight, qWeight * speedWeight, qWeight * speedWeight};
            double[][] XNdoubleValues = Get2DDoubleArray(ds);
            SetProcessErrorMatrix_X_N(XNdoubleValues);
        }

        private void SetProcessErrorMatrix_X_N(double[][] XNdoubleValues)
        {
            X_N = matrixCreater.CreatAGeneralMatrixFrom(XNdoubleValues);
        }

        private double[] UpdatePredictWights_P_W()
        {
            double pwSum = 0;
            double[] SdXYZ = { 0, 0, 0 };
            
            double[] pw = new double[SAMPLE_SIZE];
            for (int i = 0; i < SAMPLE_SIZE; ++i)
            {
                X_P_Update[i] = Update_X_P(X_P, i);
                Z_P_Update[i] = Update_Z_P(X_P_Update, i);
                for (int j = 0; j < SdXYZ.Length; ++j)
                {
                    SdXYZ[j] += Z_P_Update[i].GetElement(j, 0);
                }
            }

            GeneralMatrix SdMatrix = GetXYZCovMatrix(ref SdXYZ);

            for (int i = 0; i < SAMPLE_SIZE; ++i)
            {
                pw[i] = Calculate_P_W(SdMatrix, i);
                pwSum += pw[i];
            }

            for (int i = 0; i < SAMPLE_SIZE; ++i)
            {
                pw[i] /= pwSum;
            }
            return pw;
        }

        private GeneralMatrix GetXYZCovMatrix(ref double[] SdXYZ)
        {
            double[] MeanXYZ = { 0, 0, 0 };
            for (int j = 0; j < SdXYZ.Length; ++j)
            {
                MeanXYZ[j] = SdXYZ[j] / SAMPLE_SIZE;
            }

            SdXYZ = new double[] { 0, 0, 0 };
            for (int i = 0; i < SAMPLE_SIZE; ++i)
            {
                for (int j = 0; j < SdXYZ.Length; ++j)
                {
                    SdXYZ[j] += (Z_P_Update[i].GetElement(j, 0) - MeanXYZ[j]) * (Z_P_Update[i].GetElement(j, 0) - MeanXYZ[j]);
                }
            }

            for (int j = 0; j < SdXYZ.Length; ++j)
            {
                SdXYZ[j] = Math.Sqrt(SdXYZ[j] / SAMPLE_SIZE);
            }

            //double[][] SdMatrixDoubles = Get2DDoubleArray(SdXYZ);
            //GeneralMatrix SdArray = matrixCreater.CreatAGeneralMatrixFrom(SdMatrixDoubles);
            //GeneralMatrix SdMatrix = SdArray * SdArray.Transpose();
            double SDxx = SdXYZ[0] * SdXYZ[0];
            double SDyy = SdXYZ[1] * SdXYZ[1];
            double SDzz = SdXYZ[2] * SdXYZ[2];
            X_R.SetElement(0, 0, SDxx);
            X_R.SetElement(1, 0, SDyy);
            X_R.SetElement(2, 0, SDzz);
            double[][] sdDoubles = new double[][] { 
                                                    new double[] {0.0002, 0, 0},
                                                    new double[] {0, 0.0002, 0},
                                                    new double[] {0, 0, 0.0002},
                                                    };

            GeneralMatrix SdMatrix = matrixCreater.CreatAGeneralMatrixFrom(sdDoubles);
            return SdMatrix;
        }

        private double Calculate_P_W(GeneralMatrix SdMatrix, int i)
        {
            double pw = 0;
            pw = Get_P_W(SdMatrix, joint, Z_P_Update[i]);
            return pw;
        }
        private const double PI_TRISQURE = 8 * Math.PI * Math.PI * Math.PI;
        private double Get_P_W(GeneralMatrix SdMatrix, double[] joint, GeneralMatrix Z_P)
        {
            double[] ds = { Z_P.GetElement(0, 0) - joint[0], Z_P.GetElement(1, 0) - joint[1], Z_P.GetElement(2, 0) - joint[2] };
            double[][] ErrorDoubles = Get2DDoubleArray(ds);
            GeneralMatrix ErrorQueue = matrixCreater.CreatAGeneralMatrixFrom(ErrorDoubles);
            double sqrtPartValue = 1 / (PI_TRISQURE * Math.Sqrt(SdMatrix.Determinant()));
            GeneralMatrix right = ErrorQueue.Transpose() * SdMatrix.Inverse() * ErrorQueue;
            double expPartValue = Math.Exp(-0.5 * right.GetElement(0, 0));
            return sqrtPartValue * expPartValue;
        }

        private GeneralMatrix Update_X_P(GeneralMatrix[] xp, int i)
        {
            return GetTransitionMatrix_F() * xp[i] + GaussianRandMatrix(X_N);
        }
        private GeneralMatrix Update_Z_P(GeneralMatrix[] XPUpdate, int i)
        {
            return XPUpdate[i] + X_R;
        }
      

        private double Get_P_W_From(GeneralMatrix Z_P, GeneralMatrix X_R, int j)
        {
            double sqrtPartValue = 1 / (Math.Sqrt(2 * Math.PI) * X_R.GetElement(j, 0));
            double expPartValue = Math.Exp(-(joint[j] - Z_P.GetElement(j, 0)) * (joint[j] - Z_P.GetElement(j, 0)) / (2 * X_R.GetElement(j, 0) * X_R.GetElement(j, 0)));
            return sqrtPartValue * expPartValue;
        }

        private double Mean(GeneralMatrix[] X_P, int p)
        {
            double sum = 0;
            for (int i = 0; i < SAMPLE_SIZE; ++i)
            {
                sum += X_P[i].GetElement(p, 0);
            }
            return sum / SAMPLE_SIZE;
        }

        private void ResampleParticles_X_P()
        {
            for (int i = 0; i < SAMPLE_SIZE; ++i)
            {
                X_P[i] = ResampleAParticle(P_W, X_P_Update);
            }
        }

        private GeneralMatrix ResampleAParticle(double[] P_W, GeneralMatrix[] X_P_Update)
        {
            double randomNum = rand.NextDouble();
            double accumlate = 0;
            for (int i = 0; i < SAMPLE_SIZE; ++i)
            {
                accumlate += P_W[i];
                if (accumlate >= randomNum)
                {
                    return X_P_Update[i];
                }
            }
            return X_P_Update[0]; // TODO: should not be invoked!
        }

        private GeneralMatrix GaussianRandMatrix(GeneralMatrix X_N)
        {
            double[] ds = {X_N.GetElement(0, 0),
                            X_N.GetElement(1, 0),
                            X_N.GetElement(2, 0),
                            X_N.GetElement(3, 0),
                            X_N.GetElement(4, 0),
                            X_N.GetElement(5, 0)};
            double[][] gaussianRandDoubleValues = Get2DNormalRandomDoubleArray(ds);

            GeneralMatrix gaussianRandMatrix = matrixCreater.CreatAGeneralMatrixFrom(gaussianRandDoubleValues);

            return gaussianRandMatrix;
        }
        private double[][] Get2DNormalRandomDoubleArray(double[] ds)
        {
            double[][] TwoDNormalRandomdoubleValues = new double[ds.Length][];
            for (int i = 0; i < ds.Length; ++i)
            {
                TwoDNormalRandomdoubleValues[i] = new double[] { NormalRandom(0, ds[i]) };
            }

            return TwoDNormalRandomdoubleValues;
        }
        private double[][] Get2DDoubleArray(double[] ds)
        {
            double[][] TwoDdoubleValues = new double[ds.Length][];
            for (int i = 0; i < ds.Length; ++i)
            {
                TwoDdoubleValues[i] = new double[] { ds[i] };
            }

            return TwoDdoubleValues;
        }
        private double NormalRandom(double mean, double stdDev)
        {
            double u1 = rand.NextDouble(); //these are uniform(0,1) random doubles
            double u2 = rand.NextDouble();
            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) *
                         Math.Sin(2.0 * Math.PI * u2); //random normal(0,1)
            double randNormal =
                         mean + stdDev * randStdNormal; //random normal(mean,stdDev^2)
            return randNormal;
        }
    }
}
