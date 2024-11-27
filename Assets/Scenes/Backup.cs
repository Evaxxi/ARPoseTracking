using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using Mediapipe.Tasks.Vision.PoseLandmarker;
using UnityEngine;
using UnityEngine.Rendering;
using TMPro;
using XCharts.Runtime;
using UnityEngine.XR.ARSubsystems;

namespace Mediapipe.Unity.Sample.PoseLandmarkDetection

{
    public class Backup : VisionTaskApiRunner<PoseLandmarker>
    {
        public TextMeshProUGUI LeftKnee;
        public TextMeshProUGUI RightKnee;
        public TextMeshProUGUI Hip;
        public TextMeshProUGUI LeftAnkle;
        public TextMeshProUGUI RightAnkle;
        public TextMeshProUGUI Stride;
        public LineChart ChartLeftKnee;
        public LineChart ChartRightKnee;
        public LineChart ChartHip;
        public LineChart ChartLeftAnkle;
        public LineChart ChartRightAnkle;
        public LineChart ChartStride;

        //public MeshFilter meshFilter;
        //public Mesh skeletonMesh;
        //public List<Vector3> vertextList = new List<Vector3>();
        //public int meshScale = -5;

        public Animator skeletonAnimator;
        private Transform hip;
        //private Transform rightThumb;
        //private Transform leftThumb;
        //private Transform rightHand;
        //private Transform leftHand;
        //private Transform leftHipShoulder;
        //private Transform leftElbowShoulder;
        private Transform LeftUpperArm;
        private Transform leftWristElbow;


        [SerializeField] private PoseLandmarkerResultAnnotationController _poseLandmarkerResultAnnotationController;

        private Experimental.TextureFramePool _textureFramePool;

        public readonly PoseLandmarkDetectionConfig config = new PoseLandmarkDetectionConfig();

        public PoseLandmarkerResult poseResult;

        double angleLeftKnee;
        double angleRightKnee;
        double angleHip;
        double angleLeftAnkle;
        double angleRightAnkle;
        double lengthStride;



        void Update()
        {
            LeftKnee.text = "Angle Left Knee: " + angleLeftKnee;
            RightKnee.text = "Angle Right Knee: " + angleRightKnee;
            Hip.text = "Angle Hip: " + angleHip;
            LeftAnkle.text = "Angle Left Ankle: " + angleLeftAnkle;
            RightAnkle.text = "Angle Right Ankle: " + angleRightAnkle;
            Stride.text = "Stride Length: " + lengthStride;
            //Debug.Log(poseResult);
            //hip = skeletonAnimator.GetBoneTransform(HumanBodyBones.Hips);
            //leftHipShoulder = skeletonAnimator.GetBoneTransform(HumanBodyBones.LeftShoulder);
            LeftUpperArm = skeletonAnimator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
            leftWristElbow = skeletonAnimator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
            //rightThumb = skeletonAnimator.GetBoneTransform(HumanBodyBones.LeftHand);
            //leftThumb = skeletonAnimator.GetBoneTransform(HumanBodyBones.RightHand);
            //rightHand = skeletonAnimator.GetBoneTransform(HumanBodyBones.LeftHand);
            //leftHand = skeletonAnimator.GetBoneTransform(HumanBodyBones.RightHand);
            //rightLowerArm = skeletonAnimator.GetBoneTransform(HumanBodyBones.leftLowerArm);
            //leftLowerArm = skeletonAnimator.GetBoneTransform(HumanBodyBones.rightLowerArm);

            if (poseResult.poseWorldLandmarks != null)
            {
                //var v_hip = new Vector3(
                //    poseResult.poseWorldLandmarks[0].landmarks[23].x * 8,
                //    poseResult.poseWorldLandmarks[0].landmarks[23].y * 8,
                //    poseResult.poseWorldLandmarks[0].landmarks[23].z * 8
                //);
                //hip.SetPositionAndRotation(v_hip, Quaternion.Euler(0, 180, 0));

                var v_hips = new Vector3(
                    poseResult.poseWorldLandmarks[0].landmarks[23].x - poseResult.poseWorldLandmarks[0].landmarks[24].x,
                    poseResult.poseWorldLandmarks[0].landmarks[23].y - poseResult.poseWorldLandmarks[0].landmarks[24].y,
                    poseResult.poseWorldLandmarks[0].landmarks[23].z - poseResult.poseWorldLandmarks[0].landmarks[24].z
                );

                var c_hip = new Vector3(0, 0, 0);
                hip.SetPositionAndRotation(c_hip, Quaternion.Euler(0, 0, 0));

                //var eulers = Quaternion.FromToRotation(hip.forward, v_hips).eulerAngles;

                //hip.Rotate(eulers.x, eulers.y, eulers.z, Space.World);

                //var v_leftHipShoulder = new Vector3(
                //    poseResult.poseWorldLandmarks[0].landmarks[11].x - poseResult.poseWorldLandmarks[0].landmarks[23].x,
                //    poseResult.poseWorldLandmarks[0].landmarks[11].y - poseResult.poseWorldLandmarks[0].landmarks[23].y,
                //    poseResult.poseWorldLandmarks[0].landmarks[11].z - poseResult.poseWorldLandmarks[0].landmarks[23].z
                //);

                //leftHipShoulder.Rotate(
                //    Quaternion.FromToRotation(leftHipShoulder.forward, v_leftHipShoulder).eulerAngles,
                //    Space.World
                //);

                //eulers = Quaternion.FromToRotation(-LeftUpperArm.right, v_leftHipShoulder).eulerAngles;
                //
                //LeftUpperArm.Rotate(eulers.x, eulers.y, eulers.z, Space.World);

                //var v_leftElbowShoulder = new Vector3(
                //    poseResult.poseWorldLandmarks[0].landmarks[13].x - poseResult.poseWorldLandmarks[0].landmarks[11].x,
                //    poseResult.poseWorldLandmarks[0].landmarks[13].y - poseResult.poseWorldLandmarks[0].landmarks[11].y,
                //    poseResult.poseWorldLandmarks[0].landmarks[13].z - poseResult.poseWorldLandmarks[0].landmarks[11].z
                //);

                //var v_leftWristElbow = new Vector3(
                //    poseResult.poseWorldLandmarks[0].landmarks[15].x - poseResult.poseWorldLandmarks[0].landmarks[13].x,
                //    poseResult.poseWorldLandmarks[0].landmarks[15].y - poseResult.poseWorldLandmarks[0].landmarks[13].y,
                //    poseResult.poseWorldLandmarks[0].landmarks[15].z - poseResult.poseWorldLandmarks[0].landmarks[13].z
                //);

                //eulers = Quaternion.FromToRotation(LeftUpperArm.right, v_leftElbowShoulder).eulerAngles;

                //LeftUpperArm.Rotate(eulers.x, eulers.y, eulers.z, Space.World);

                //LeftUpperArm.Rotate(
                //    Quaternion.FromToRotation(-LeftUpperArm.right, v_leftElbowShoulder).eulerAngles,
                //    Space.World
                //);

                //var norm_x = Vector3.Cross(v_leftElbowShoulder, v_leftWristElbow);
                //LeftUpperArm.Rotate(
                //    Quaternion.FromToRotation(-LeftUpperArm.forward, norm_x).eulerAngles,
                //    Space.World
                //);


                //leftWristElbow.Rotate(
                //    Quaternion.FromToRotation(leftWristElbow.forward, v_leftWristElbow).eulerAngles,
                //    Space.World
                //);

                //var v_lefthand = new Vector3(
                //    
                //    );
                //var v_lefthand = new Vector3(
                //    poseResult.poseWorldLandmarks[0].landmarks[15].x * 8,
                //    poseResult.poseWorldLandmarks[0].landmarks[15].y * 8,
                //    poseResult.poseWorldLandmarks[0].landmarks[15].z * 8
                //);
                //rightHand.SetPositionAndRotation(v_lefthand, Quaternion.Euler(0, 180, 0));
                //var v_righthand = new Vector3(
                //    poseResult.poseWorldLandmarks[0].landmarks[16].x * 8,
                //    poseResult.poseWorldLandmarks[0].landmarks[16].y * 8,
                //    poseResult.poseWorldLandmarks[0].landmarks[16].z * 8
                //);
                //leftHand.SetPositionAndRotation(v_righthand, Quaternion.Euler(0, 180, 0));
            }

            //Debug.Log(hip.position.x);
        }

        public override void Stop()
        {
            base.Stop();
            _textureFramePool?.Dispose();
            _textureFramePool = null;
        }

        protected override IEnumerator Run()
        {
            Debug.Log($"Delegate = {config.Delegate}");
            Debug.Log($"Model = {config.ModelName}");
            Debug.Log($"Running Mode = {config.RunningMode}");
            Debug.Log($"NumPoses = {config.NumPoses}");
            Debug.Log($"MinPoseDetectionConfidence = {config.MinPoseDetectionConfidence}");
            Debug.Log($"MinPosePresenceConfidence = {config.MinPosePresenceConfidence}");
            Debug.Log($"MinTrackingConfidence = {config.MinTrackingConfidence}");
            Debug.Log($"OutputSegmentationMasks = {config.OutputSegmentationMasks}");

            yield return AssetLoader.PrepareAssetAsync(config.ModelPath);

            var options = config.GetPoseLandmarkerOptions(config.RunningMode == Tasks.Vision.Core.RunningMode.LIVE_STREAM ? OnPoseLandmarkDetectionOutput : null);
            taskApi = PoseLandmarker.CreateFromOptions(options, GpuManager.GpuResources);
            var imageSource = ImageSourceProvider.ImageSource;

            yield return imageSource.Play();

            if (!imageSource.isPrepared)
            {
                Logger.LogError(TAG, "Failed to start ImageSource, exiting...");
                yield break;
            }

            // Use RGBA32 as the input format.
            // TODO: When using GpuBuffer, MediaPipe assumes that the input format is BGRA, so maybe the following code needs to be fixed.
            _textureFramePool = new Experimental.TextureFramePool(imageSource.textureWidth, imageSource.textureHeight, TextureFormat.RGBA32, 10);

            // NOTE: The screen will be resized later, keeping the aspect ratio.
            screen.Initialize(imageSource);

            SetupAnnotationController(_poseLandmarkerResultAnnotationController, imageSource);
            _poseLandmarkerResultAnnotationController.InitScreen(imageSource.textureWidth, imageSource.textureHeight);

            //meshFilter = GameObject.Find("Skeleton").GetComponent<MeshFilter>();
            //skeletonMesh = meshFilter.mesh;
            //vertextList.AddRange(skeletonMesh.vertices);

            //Debug.Log(vertextList);

            //skeletonAnimator = GetComponent<Animator>();

            //Debug.Log(skeletonAnimator.bodyPosition);

            var transformationOptions = imageSource.GetTransformationOptions();
            var flipHorizontally = transformationOptions.flipHorizontally;
            var flipVertically = transformationOptions.flipVertically;

            // Always setting rotationDegrees to 0 tod avoid the issue that the detection becomes unstable when the input image is rotated.
            // https://github.com/homuler/MediaPipeUnityPlugin/issues/1196
            var imageProcessingOptions = new Tasks.Vision.Core.ImageProcessingOptions(rotationDegrees: 0);

            AsyncGPUReadbackRequest req = default;
            var waitUntilReqDone = new WaitUntil(() => req.done);
            var result = PoseLandmarkerResult.Alloc(options.numPoses, options.outputSegmentationMasks);

            // NOTE: we can share the GL context of the render thread with MediaPipe (for now, only on Android)
            var canUseGpuImage = options.baseOptions.delegateCase == Tasks.Core.BaseOptions.Delegate.GPU &&
              SystemInfo.graphicsDeviceType == GraphicsDeviceType.OpenGLES3 &&
              GpuManager.GpuResources != null;
            using var glContext = canUseGpuImage ? GpuManager.GetGlContext() : null;

            while (true)
            {
                if (isPaused)
                {
                    yield return new WaitWhile(() => isPaused);
                }

                if (!_textureFramePool.TryGetTextureFrame(out var textureFrame))
                {
                    yield return new WaitForEndOfFrame();
                    continue;
                }

                //TestText();

                // Build the input Image
                Image image;
                if (canUseGpuImage)
                {
                    yield return new WaitForEndOfFrame();
                    textureFrame.ReadTextureOnGPU(imageSource.GetCurrentTexture(), flipHorizontally, flipVertically);
                    image = textureFrame.BuildGpuImage(glContext);
                }
                else
                {
                    req = textureFrame.ReadTextureAsync(imageSource.GetCurrentTexture(), flipHorizontally, flipVertically);
                    yield return waitUntilReqDone;

                    if (req.hasError)
                    {
                        Debug.LogError($"Failed to read texture from the image source, exiting...");
                        break;
                    }
                    image = textureFrame.BuildCPUImage();
                    textureFrame.Release();
                }

                switch (taskApi.runningMode)
                {
                    case Tasks.Vision.Core.RunningMode.IMAGE:
                        if (taskApi.TryDetect(image, imageProcessingOptions, ref result))
                        {
                            _poseLandmarkerResultAnnotationController.DrawNow(result);
                        }
                        else
                        {
                            _poseLandmarkerResultAnnotationController.DrawNow(default);
                        }
                        break;
                    case Tasks.Vision.Core.RunningMode.VIDEO:
                        if (taskApi.TryDetectForVideo(image, GetCurrentTimestampMillisec(), imageProcessingOptions, ref result))
                        {
                            _poseLandmarkerResultAnnotationController.DrawNow(result);
                        }
                        else
                        {
                            _poseLandmarkerResultAnnotationController.DrawNow(default);
                        }
                        break;
                    case Tasks.Vision.Core.RunningMode.LIVE_STREAM:
                        //Debug.Log(vertextList[0]);
                        if (angleLeftKnee != 0)
                        {
                            ChartLeftKnee.AddXAxisData("x" + GetCurrentTimestampMillisec());
                            ChartLeftKnee.AddData(0, angleLeftKnee);
                            ChartRightKnee.AddXAxisData("x" + GetCurrentTimestampMillisec());
                            ChartRightKnee.AddData(0, angleRightKnee);
                            ChartHip.AddXAxisData("x" + GetCurrentTimestampMillisec());
                            ChartHip.AddData(0, angleHip);
                            ChartLeftAnkle.AddXAxisData("x" + GetCurrentTimestampMillisec());
                            ChartLeftAnkle.AddData(0, angleLeftAnkle);
                            ChartRightAnkle.AddXAxisData("x" + GetCurrentTimestampMillisec());
                            ChartRightAnkle.AddData(0, angleRightAnkle);
                            ChartStride.AddXAxisData("x" + GetCurrentTimestampMillisec());
                            ChartStride.AddData(0, lengthStride);
                        }
                        Update();
                        taskApi.DetectAsync(image, GetCurrentTimestampMillisec(), imageProcessingOptions);
                        break;
                }
                //Debug.Log(imageProcessingOptions);
            }
        }

        //public void UpdateFaceMesh(PoseLandmarkerResult result)
        //{
        //   //Debug.Log(result);
        //   // 478 - 10 = 468
        //   for (var i = 0; i < result.poseWorldLandmarks[0].landmarks.Count - 10; i++)
        //   {
        //       var landmark = result.poseWorldLandmarks[0].landmarks[i];
        //       //Landmark Mesh
        //       vertextList[i] = new Vector3(meshScale * landmark.x, meshScale * landmark.y, meshScale * landmark.z);
        //   }
        //   // Mesh
        //   skeletonMesh.SetVertices(vertextList);
        //}

        public void OnPoseLandmarkDetectionOutput(PoseLandmarkerResult result, Image image, long timestamp)
        {
            // 0 - nose
            // 1 - left eye (inner)
            // 2 - left eye
            // 3 - left eye (outer)
            // 4 - right eye (inner)
            // 5 - right eye
            // 6 - right eye (outer)
            // 7 - left ear
            // 8 - right ear
            // 9 - mouth (left)
            // 10 - mouth (right)
            // 11 - left shoulder
            // 12 - right shoulder
            // 13 - left elbow
            // 14 - right elbow
            // 15 - left wrist
            // 16 - right wrist
            // 17 - left pinky
            // 18 - right pinky
            // 19 - left index
            // 20 - right index
            // 21 - left thumb
            // 22 - right thumb
            // 23 - left hip
            // 24 - right hip
            // 25 - left knee
            // 26 - right knee
            // 27 - left ankle
            // 28 - right ankle
            // 29 - left heel
            // 30 - right heel
            // 31 - left foot index
            // 32 - right foot index
            //angleLeftKnee = GetAngle(result, 23, 25, 27);
            // angleRightKnee = GetAngle(result, 24, 26, 28);
            poseResult = result;

            angleLeftKnee = GetAngle(result, 24, 26, 28);
            angleRightKnee = GetAngle(result, 23, 25, 27);
            angleHip = GetAngle(result, 11, 23, 25);
            //angleLeftAnkle = GetAngle(result, 25, 27, 31);
            //angleRightAnkle = GetAngle(result, 26, 28, 32);
            angleLeftAnkle = GetAngle(result, 26, 28, 32);
            angleRightAnkle = GetAngle(result, 25, 27, 31);
            lengthStride = GetDistance(result, 29, 30);

            //poseResult = result;

            _poseLandmarkerResultAnnotationController.DrawLater(result);

            //UpdateFaceMesh(result);
        }

        private static double GetAngle(PoseLandmarkerResult result, int a, int b, int c)
        {
            //Vector Formula
            //v1 = {A.x - B.x, A.y - B.y, A.z - B.z}
            //v2 = {C.x - B.x, C.y - B.y, C.z - B.z}

            double Ax = result.poseWorldLandmarks[0].landmarks[a].x;
            double Ay = result.poseWorldLandmarks[0].landmarks[a].y;
            double Az = result.poseWorldLandmarks[0].landmarks[a].z;
            double Bx = result.poseWorldLandmarks[0].landmarks[b].x;
            double By = result.poseWorldLandmarks[0].landmarks[b].y;
            double Bz = result.poseWorldLandmarks[0].landmarks[b].z;
            double Cx = result.poseWorldLandmarks[0].landmarks[c].x;
            double Cy = result.poseWorldLandmarks[0].landmarks[c].y;
            double Cz = result.poseWorldLandmarks[0].landmarks[c].z;
            double[] v1 = { Ax - Bx, Ay - By, Az - Bz };
            double[] v2 = { Cx - Bx, Cy - By, Cz - Bz };

            //v1mag = sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z)
            //v1norm = { v1.x / v1mag, v1.y / v1mag, v1.z / v1mag}
            double v1mag = Math.Sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
            double[] v1norm = { v1[0] / v1mag, v1[1] / v1mag, v1[2] / v1mag };

            //v2mag = sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z)
            //v2norm = { v2.x / v2mag, v2.y / v2mag, v2.z / v2mag}
            double v2mag = Math.Sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
            double[] v2norm = { v2[0] / v2mag, v2[1] / v2mag, v2[2] / v2mag };

            //dot products of vectors v1 and v2
            //v1norm.x * v2norm.x + v1norm.y * v2norm.y + v1norm.z * v2norm.z
            double dotProduct = v1norm[0] * v2norm[0] + v1norm[1] * v2norm[1] + v1norm[2] * v2norm[2];

            //angle from the dot products
            double angle = (Math.Acos(dotProduct) * 180.0) / Math.PI;

            //Debug.Log(angle);
            return angle;
        }

        private static double GetDistance(PoseLandmarkerResult result, int a, int b)
        {
            double Ax = result.poseWorldLandmarks[0].landmarks[a].x;
            double Ay = result.poseWorldLandmarks[0].landmarks[a].y;
            double Az = result.poseWorldLandmarks[0].landmarks[a].z;
            double Bx = result.poseWorldLandmarks[0].landmarks[b].x;
            double By = result.poseWorldLandmarks[0].landmarks[b].y;
            double Bz = result.poseWorldLandmarks[0].landmarks[b].z;

            double distx = Ax - Bx;
            double disty = Ay - By;
            double distz = Az - Bz;

            double distance = Math.Sqrt(distx * distx + disty * disty + distz * distz);

            return distance;
        }

    }

}
