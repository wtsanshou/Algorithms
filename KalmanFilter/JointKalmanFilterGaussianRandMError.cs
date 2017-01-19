using System;
using DotNetMatrix;
using MultiKinect.KDataModels.Factory;
using MultiKinect.KDataModels.KSkeletons;
using MultiKinect.KDataModels.MatrixFactories;
namespace MultiKinect.Fusion.KalmanFilter
{
    public class JointKalmanFilterGaussianRandMError : IJointKalmanFilter
    {
        /// <summary>
        /// 6
        /// </summary>
        private const int DIMENSION = 6;
        /// <summary>
        /// need to be modified based on movement
        /// </summary>
        private const double DEFAULT_Q = 2.0;
        private double qWEIGHT = 2.0; // process error weight, when we think the predicted value is very bad, increase the value

        private const double ERRORcovX = 0.05;
        private const double ERRORcovY = 0.0245;
        private const double ERRORcovZ = 0.05;

        private double[] MEASUREMENT_ERRORcovXYZ = { ERRORcovX, ERRORcovY, ERRORcovZ };
        /// <summary>
        /// 2
        /// </summary>
        private const double MEASURE_ERROR_AMPLIFICATION_TIMES = 2;
        /// <summary>
        /// 12.4
        /// </summary>
        private const double MAX_HUMAN_SPEED = 12.4; //  m/s
        /// <summary>
        /// 0.05
        /// </summary>
        private const double DEFAULT_TIME_INTERVAL = 0.05; //seconds;
        /// <summary>
        /// 1000
        /// </summary>
        private const double ONE_SECOND = 1000; // one second = 1000 million seconds
        private double previousTime;
        private double curTime;

        private GeneralMatrix previousX;
        private GeneralMatrix predictX;
        private GeneralMatrix previousP;
        private GeneralMatrix predictP;
        private double deltaT;

        private GeneralMatrix TransitionMatrix_A;
        private GeneralMatrix covMatrix_Q;
        private GeneralMatrix measurementError_R;
        private GeneralMatrix formatMatrix_H;
        private GeneralMatrix UnitMatrix_I;
        private GeneralMatrix kalmanGain_K;
        private GeneralMatrix measurementValues_Yt;

        private GeneralMatrix estimatedResult_Xt;
        private GeneralMatrix estimatedState_Pt;

        private Random rand;

        private JointCreater jointCreater;
        private MatrixCreater matrixCreater;

        public JointKalmanFilterGaussianRandMError(JointCreater jointCreater, MatrixCreater matrixCreater)
        {
            rand = new Random(); //reuse this if you are generating many
            this.jointCreater = jointCreater;
            this.matrixCreater = matrixCreater;

            InitializeKalmanParameters();
        }

        public void AmplifySensorError(double times)
        {
            MEASUREMENT_ERRORcovXYZ = new double[] { NormalRandom(0, ERRORcovX), NormalRandom(0, ERRORcovY), NormalRandom(0, ERRORcovZ) };
            //MEASUREMENT_ERRORcovXYZ = new double[] { -ERRORcovX, -ERRORcovY, -ERRORcovZ };

            qWEIGHT = DEFAULT_Q;
            //for (int i = 0; i < MEASUREMENT_ERRORcovXYZ.Length; ++i)
            //{
            //    MEASUREMENT_ERRORcovXYZ[i] *= times;
            //}
            //qWEIGHT /= times;
        }

        public void setMeasurementErrorCovXYZ(double Ex, double Ey, double Ez)
        {
            MEASUREMENT_ERRORcovXYZ[0] = NormalRandom(0, Ex) * 100;
            MEASUREMENT_ERRORcovXYZ[1] = NormalRandom(0, Ey) * 100;
            MEASUREMENT_ERRORcovXYZ[2] = NormalRandom(0, Ez) * 100;
        }

        private double NormalRandom(double mean, double stdDev)
        {
            double u1 = rand.NextDouble(); //these are uniform(0,1) random doubles
            double u2 = rand.NextDouble();
            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) *
                         Math.Sin(2.0 * Math.PI * u2); //random normal(0,1)
            double randNormal =
                         mean + stdDev * randStdNormal; //random normal(mean,stdDev^2)
            return Math.Abs(randNormal);
        }

        private void InitializeKalmanParameters()
        {
            this.TransitionMatrix_A = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, DIMENSION);
            this.covMatrix_Q = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, DIMENSION);
            this.measurementError_R = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, DIMENSION);
            this.formatMatrix_H = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, DIMENSION);
            this.UnitMatrix_I = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, DIMENSION);
            this.kalmanGain_K = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, DIMENSION);
            this.measurementValues_Yt = matrixCreater.CreatAnEmpty_MN_Matrix(DIMENSION, 1);
            SetFormatMatrix_H();
            SetUnitMatrix_I();
        }

        public XmlOneJoint FilterJoint(XmlOneJoint joint, double time_T)
        {
            PredictValues(time_T);

            return EstimateJoint(joint);
        }

        public XmlOneJoint PredictValues(double time_T)
        {
            SetCurrentTime(time_T);
            SetDeltaT(GetCurrentTime() - GetPreviousTime());

            SetTransitionMatrix_A();
            PredictPositionAndVelocity_Xtp();

            XmlOneJoint predictedJoint = jointCreater.CreateAJoint(predictX.GetElement(0, 0),
                                                                    predictX.GetElement(1, 0),
                                                                    predictX.GetElement(2, 0),
                                                                    KSkeletonWithInfo.KALMAN_FILTERJOINT
                                                                );
            return predictedJoint;
        }

        public XmlOneJoint EstimateJoint(XmlOneJoint joint)
        {
            SetProcessCovMatrix_Qt();
            PredictState_Ptp();

            double[] KinectMeasureErrorXYZ = EstimateMeasureError(joint);
            SetMeasurementError_R(KinectMeasureErrorXYZ); //TODO: find the right values based on the joint statements
            KalmanGain_Kt();

            SetMeasurementValues_Yt(joint);

            GeneralMatrix result = EstimatedResult_Xt();
            XmlOneJoint filteredJoint = jointCreater.CreateAJoint(result.GetElement(0, 0),
                                                                    result.GetElement(1, 0),
                                                                    result.GetElement(2, 0),
                                                                    KSkeletonWithInfo.KALMAN_FILTERJOINT
                                                                );
            UpdataState_Pk();

            return filteredJoint;
        }


        private void SetCurrentTime(double time_T)
        {
            this.curTime = time_T;
        }
        private double GetCurrentTime()
        {
            return this.curTime;
        }
        public void SetPreviousValues(GeneralMatrix goal_X, GeneralMatrix state_P, double time_T)
        {
            this.previousX = goal_X;
            this.previousP = state_P;
            this.previousTime = time_T;
        }

        public GeneralMatrix GetPreviousX()
        {
            return this.previousX;
        }
        public GeneralMatrix GetPreviousP()
        {
            return this.previousP;
        }
        public double GetPreviousTime()
        {
            return this.previousTime;
        }

        private void SetDeltaT(double timeInterval)
        {
            this.deltaT = (timeInterval <= 0) ? DEFAULT_TIME_INTERVAL : timeInterval / ONE_SECOND;
        }

        public double GetDeltaT()
        {
            return this.deltaT;
        }

        private void SetTransitionMatrix_A()
        {
            for (int i = 0; i < DIMENSION; ++i)
                this.TransitionMatrix_A.SetElement(i, i, 1);
            for (int row = 0; row < 3; ++row)
                this.TransitionMatrix_A.SetElement(row, row + 3, GetDeltaT());
        }

        public GeneralMatrix GetTransitionMatrix_A()
        {
            return this.TransitionMatrix_A;
        }

        private void PredictPositionAndVelocity_Xtp()
        {
            this.predictX = GetTransitionMatrix_A() * GetPreviousX();
        }

        public GeneralMatrix GetPredictX()
        {
            return this.predictX;
        }

        private void SetProcessCovMatrix_Qt()
        {
            double predictedMoveDistance = GetPredictedMoveDistance();
            double q = qWEIGHT;
            if (FasterThanHumanSpeed(predictedMoveDistance))
                q *= 10;
            double T = GetDeltaT();

            //double TT = 0.5 * T * T;
            //TODO: this model is not work, I may make some mistake about this model
            //double[][] w = { new double[] { TT }, new double[] { TT }, new double[] { TT },
            //                   new double[] { T }, new double[] { T }, new double[] { T } };
            //GeneralMatrix W = matrixCreater.CreatAGeneralMatrixFrom(w);

            //double[][] sd = { new double[] { q * T } };
            //GeneralMatrix SD = matrixCreater.CreatAGeneralMatrixFrom(sd);
            //covMatrix_Q = W * SD * W.Transpose();

            double weight = q * T;
            covMatrix_Q = GetTransitionMatrix_A() * GetTransitionMatrix_A().Transpose();
            for (int i = 0; i < DIMENSION; ++i)
            {
                for (int j = 0; j < DIMENSION; ++j)
                    this.covMatrix_Q.SetElement(i, j, covMatrix_Q.GetElement(i, j) * weight);
            }
        }

        private bool FasterThanHumanSpeed(double predictedMoveDistance)
        {
            return predictedMoveDistance > MAX_HUMAN_SPEED * GetDeltaT();
        }

        private double GetPredictedMoveDistance()
        {
            double x = MoveDistanceFromAxis(0);
            double y = MoveDistanceFromAxis(1);
            double z = MoveDistanceFromAxis(2);
            return Math.Sqrt(x * x + y * y + z * z);
        }

        private double MoveDistanceFromAxis(int row)
        {
            return GetPredictX().GetElement(row, 0) - GetPreviousX().GetElement(row, 0);
        }

        public GeneralMatrix GetProcesCovMatrix_Qt()
        {
            return this.covMatrix_Q;
        }

        private void PredictState_Ptp()
        {
            this.predictP = A_P_AT() + GetProcesCovMatrix_Qt();
        }

        private GeneralMatrix A_P_AT()
        {
            return GetTransitionMatrix_A() * GetPreviousP() * (GetTransitionMatrix_A().Transpose());
        }

        public GeneralMatrix GetPredictState_Ptp()
        {
            return this.predictP;
        }

        private double[] EstimateMeasureError(XmlOneJoint joint)
        {
            double MeasuredMovingDistance = GetMeasuredMovingDistance(joint);

            double PredictedMovingDistance = GetPredictedMovingDistance();

            double[] KinectMeasureErrorXYZ = new double[MEASUREMENT_ERRORcovXYZ.Length];
            for (int i = 0; i < KinectMeasureErrorXYZ.Length; ++i)
            {
                KinectMeasureErrorXYZ[i] = MEASUREMENT_ERRORcovXYZ[i];
            }

            //CheckIfNeedAmplifyMeasurementErrors(joint, MeasuredMovingDistance, PredictedMovingDistance, KinectMeasureErrorXYZ);

            return KinectMeasureErrorXYZ;
        }

        private void CheckIfNeedAmplifyMeasurementErrors(XmlOneJoint joint, double MeasuredMovingDistance, double PredictedMovingDistance, double[] KinectMeasureErrorXYZ)
        {
            if (JointIsPredictByKinect(joint) || FasterThanHumanSpeed(MeasuredMovingDistance))
            {
                AmplificationMeasureError(KinectMeasureErrorXYZ);
            }
            for (int i = 2; i <= 8; i += 2)
            {
                if (MeasuredMovingDistanceIsLargerThanPredict(MeasuredMovingDistance, PredictedMovingDistance, i))
                {
                    AmplificationMeasureError(KinectMeasureErrorXYZ);
                }
            }
        }

        private double GetPredictedMovingDistance()
        {
            double vx = GetDeltaT() * GetPreviousX().GetElement(3, 0);
            double vy = GetDeltaT() * GetPreviousX().GetElement(4, 0);
            double vz = GetDeltaT() * GetPreviousX().GetElement(5, 0);
            double PredictedMovingDistance = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            return PredictedMovingDistance;
        }

        private double GetMeasuredMovingDistance(XmlOneJoint joint)
        {
            double mx = joint.x - GetPreviousX().GetElement(0, 0);
            double my = joint.y - GetPreviousX().GetElement(1, 0);
            double mz = joint.z - GetPreviousX().GetElement(2, 0);
            double MeasuredMovingDistance = Math.Sqrt(mx * mx + my * my + mz * mz);
            return MeasuredMovingDistance;
        }
        private bool JointIsPredictByKinect(XmlOneJoint joint)
        {
            return joint.t != 2;
        }
        private static void AmplificationMeasureError(double[] KinectMeasureErrorXYZ)
        {
            for (int i = 0; i < KinectMeasureErrorXYZ.Length; ++i)
                KinectMeasureErrorXYZ[i] *= MEASURE_ERROR_AMPLIFICATION_TIMES;
        }
        private bool MeasuredMovingDistanceIsLargerThanPredict(double MeasuredMovingDistance, double PredictedMovingDistance, int times)
        {
            return PredictedMovingDistance > 0 && MeasuredMovingDistance > (PredictedMovingDistance * times);
        }

        private void SetMeasurementError_R(double[] maximum)
        {
            for (int i = 0; i < 3; ++i)
            {
                this.measurementError_R.SetElement(i, i, maximum[i]);
                maximum[i] /= (GetDeltaT() * GetDeltaT() * 10);  // TODO: refactor it. It means the error covairance of axis speed
            }
            for (int i = 3; i < DIMENSION; ++i)
                this.measurementError_R.SetElement(i, i, maximum[i - 3]);
        }

        public GeneralMatrix GetMeasurementError_R()
        {
            return this.measurementError_R;
        }

        private void SetFormatMatrix_H()
        {
            formatMatrix_H = GetAUnitMatrix(DIMENSION);
        }
        private void SetUnitMatrix_I()
        {
            UnitMatrix_I = GetAUnitMatrix(DIMENSION);
        }

        public GeneralMatrix GetAUnitMatrix(int n)
        {
            GeneralMatrix unitMatrix = matrixCreater.CreatAnEmpty_MN_Matrix(n, n);
            for (int i = 0; i < n; ++i)
                unitMatrix.SetElement(i, i, 1);
            return unitMatrix;
        }

        private void KalmanGain_Kt()
        {
            GeneralMatrix numerator = Ptp_HT();
            GeneralMatrix denominator = H_Ptp_HT() + GetMeasurementError_R();
            this.kalmanGain_K = numerator * denominator.Inverse();

        }

        private GeneralMatrix Ptp_HT()
        {
            return GetPredictState_Ptp() * formatMatrix_H.Transpose();
        }
        private GeneralMatrix H_Ptp_HT()
        {
            return formatMatrix_H * GetPredictState_Ptp() * formatMatrix_H.Transpose();
        }

        public GeneralMatrix GetKalmanGain_K()
        {
            return this.kalmanGain_K;
        }

        private void SetMeasurementValues_Yt(XmlOneJoint joint)
        {
            measurementValues_Yt.SetElement(0, 0, joint.x);
            measurementValues_Yt.SetElement(1, 0, joint.y);
            measurementValues_Yt.SetElement(2, 0, joint.z);

            double T = GetDeltaT();

            double velocityX = (T == 0) ? 0 : (joint.x - GetPreviousX().GetElement(0, 0)) / T;
            double velocityY = (T == 0) ? 0 : (joint.y - GetPreviousX().GetElement(1, 0)) / T;
            double velocityZ = (T == 0) ? 0 : (joint.z - GetPreviousX().GetElement(2, 0)) / T;

            measurementValues_Yt.SetElement(3, 0, velocityX);
            measurementValues_Yt.SetElement(4, 0, velocityY);
            measurementValues_Yt.SetElement(5, 0, velocityZ);
        }

        public GeneralMatrix GetMeasurementValues_Yt()
        {
            return this.measurementValues_Yt;
        }

        private GeneralMatrix EstimatedResult_Xt()
        {
            estimatedResult_Xt = GetPredictX() + GetKalmanGain_K() * (GetMeasurementValues_Yt() - H_pX());
            return estimatedResult_Xt;
        }

        private GeneralMatrix H_pX()
        {
            return formatMatrix_H * GetPredictX();
        }

        private void UpdataState_Pk()
        {
            estimatedState_Pt = (UnitMatrix_I - K_H()) * GetPredictState_Ptp();
            SetPreviousValues(estimatedResult_Xt, estimatedState_Pt, curTime);
        }

        private GeneralMatrix K_H()
        {
            return GetKalmanGain_K() * formatMatrix_H;
        }
    }
}

