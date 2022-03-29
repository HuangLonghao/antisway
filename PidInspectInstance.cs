/*----------------------------------------------------------------
// Copyright (C) 2021 JunengVision 版权所有
//
// 文件名：PidInspectInstance.cs
// 文件功能描述：Pid防摇控制实列
//
// 创建标识：haoshuaizheng_20210821.1404
// 审核：
//
// 修改标识：
// 修改描述：
// 审核：
//
//----------------------------------------------------------------*/

using System;
using System.IO;

namespace Jn.RtgLocalSys.Bll
{
    /// <summary>
    /// Pid防摇控制实列
    /// </summary>
    public static class PidInspectInstance
    {
        delegate void PID_INITApiHandler(double rope_length, double v_max, double torque_max, double static_angle, double input_gain_X, double input_Kd_X, double input_gain_theta, double input_Kp_theta, double input_Kd_theta);
        delegate float PID_REALIZEApiHandler(double sampling_time, double angle, double current_x, double target_x);

        private static IntPtr mModuleHandle = IntPtr.Zero;

        /// <summary>
        /// 加载Pid防摇控制算法库
        /// </summary>
        /// <param name="algoLibraryFullPath"></param>
        /// <param name="errMsg"></param>
        /// <returns></returns>
        public static bool LoadLib(string algoLibraryFullPath, out string errMsg)
        {
            bool loadRlt = false;
            errMsg = string.Empty;
            try
            {
                if (!File.Exists(algoLibraryFullPath))
                {
                    throw new Exception("Algorithms library not exist at Jn.AlgoInterface.AlgoMtd.LoadLib");
                }
                mModuleHandle = DLLWrapper.LoadLibrary(algoLibraryFullPath);
                if (IntPtr.Zero != mModuleHandle)
                {
                    loadRlt = true;
                }
            }
            catch (Exception ex)
            {
                errMsg = "PidInspectInstance.LoadLib:" + ex.ToString();
            }
            return loadRlt;
        }

        /// <summary>
        /// 初始化 需要在开始加速前调用
        /// </summary>
        /// <param name="rope_length">绳长，单位为m</param>
        /// <param name="v_max">小车最大速度，单位为m/s</param>
        /// <param name="torque_max">电机最大转矩，单位为N*m</param>
        /// <param name="static_angle">静止状态下角度，单位为rad，默认为0</param>
        /// <param name="input_gain_X">预留接口，一般不需要传入值</param>
        /// <param name="input_Kd_X">预留接口，一般不需要传入值</param>
        /// <param name="input_gain_theta">预留接口，一般不需要传入值</param>
        /// <param name="input_Kp_theta">预留接口，一般不需要传入值</param>
        /// <param name="input_Kd_theta">预留接口，一般不需要传入值</param>
        public static void PID_INIT(double rope_length, double v_max, double torque_max, double static_angle = 0, double input_gain_X = 24, double input_Kd_X = 7.5, double input_gain_theta = 280, double input_Kp_theta = 2500, double input_Kd_theta = 1000)
        {
            try
            {
                PID_INITApiHandler ApiHandler = (PID_INITApiHandler)DLLWrapper.GetFunctionAddress(mModuleHandle, "PID_INIT", typeof(PID_INITApiHandler));
                if (ApiHandler == null)
                {
                    return;
                }
                ApiHandler(rope_length, v_max, torque_max, static_angle, input_gain_X, input_Kd_X, input_gain_theta, input_Kp_theta, input_Kd_theta);
                ApiHandler = null;
                return;
            }
            catch (Exception ex)
            {
                throw (ex);
            }
        }

        /// <summary>
        /// 初始化后每次对角度偏差信号采样后都需要调用
        /// </summary>
        /// <param name="sampling_time">采样时间间隔，单位为s</param>
        /// <param name="angle">当前偏差角（与小车方向相反时为正，反之为负）单位为弧度</param>
        /// <param name="current_x">当前位置，单位为m</param>
        /// <param name="target_x">目标位置，单位为m</param>
        /// <returns>返回值：电机转矩</returns>
        public static double PID_REALIZE(double sampling_time, double angle, double current_x, double target_x)
        {
            try
            {
                float rlt = 0;
                PID_REALIZEApiHandler ApiHandler = (PID_REALIZEApiHandler)DLLWrapper.GetFunctionAddress(mModuleHandle, "PID_REALIZE", typeof(PID_REALIZEApiHandler));
                if (ApiHandler == null)
                {
                    return rlt;
                }
                rlt = ApiHandler(sampling_time, angle, current_x, target_x);
                ApiHandler = null;
                return rlt;
            }
            catch (Exception ex)
            {
                throw (ex);
            }
        }
    }
}
