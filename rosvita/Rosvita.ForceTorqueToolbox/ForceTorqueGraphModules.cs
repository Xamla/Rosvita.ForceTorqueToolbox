using System;
using System.IO;
using System.Collections.Generic;
using System.Threading.Tasks;
using Uml.Robotics.Ros;
using Xamla.Graph;
using Xamla.Graph.MethodModule;

using Messages.ft_sensor_client;
using Rosvita.RosMonitor;

namespace Rosvita.ForceTorqueToolbox
{
    [Module(ModuleType = "Xamla.ForceTorqueToolbox.NullifySensor", Description = "Use the last sampled wrench-datapoints to recalculate the bias.", Flow = true)]
    public class NullifySensor
        : SingleInstanceMethodModule
    {
        const string DEFAULT_NODE_NAME = "/ft_sensor_client";
        IRosClientLibrary rosClient;

        public NullifySensor(IGraphRuntime runtime, IRosClientLibrary rosClient)
            : base(runtime)
        {
            this.rosClient = rosClient;
        }

        [ModuleMethod]
        public async Task Nullify(
            [InputPin(Name = "Node Name", Description = "The name of the toolbox node.", PropertyMode = PropertyMode.Default, DefaultValue = DEFAULT_NODE_NAME)] string nodeName,
            [InputPin(Name = "Sample Size", Description = "The sample size of wrench data points buffered, from which the average is assumed as the bias applied to the next points captured. A value of 0 resets the bias to zero.", PropertyMode = PropertyMode.Default)] uint sampleSize = 200)
        {
            string serviceName = Path.Combine(nodeName, "nullify");

            using (var client = rosClient.GlobalNodeHandle.ServiceClient<Nullify>(serviceName))
            {
                var srv = new Nullify();
                srv.req.sampleSize = sampleSize;
                if (!await client.CallAsync(srv))
                    throw new ServiceCallFailedException(serviceName);

                // Crash early when node does not return success
                if (!srv.resp.success)
                {
                    throw new Exception("Could not nullify data." + srv.resp.message.ToString());
                }
            }
        }
    }

    [Module(ModuleType = "Xamla.ForceTorqueToolbox.BeginCaptureData", Description = "This module begins the capturing process.", Flow = true)]
    public class BeginDataCollection
        : SingleInstanceMethodModule
    {
        const string DEFAULT_NODE_NAME = "/ft_sensor_client";
        const string DEFAULT_REFERENCE_FRAME = "world";
        IRosClientLibrary rosClient;
        string currentNodeName;
        bool running;

        public BeginDataCollection(IGraphRuntime runtime, IRosClientLibrary rosClient)
            : base(runtime)
        {
            this.rosClient = rosClient;
            currentNodeName = "";
            running = false;
        }

        [ModuleMethod]
        public async Task BeginCapture(
            [InputPin(Name = "Node Name", Description = "The name of the toolbox node.", PropertyMode = PropertyMode.Default, DefaultValue = DEFAULT_NODE_NAME)] string nodeName,
            [InputPin(Name = "Reference Frame", Description = "The reference frame in which the data should be in.", PropertyMode = PropertyMode.Default, DefaultValue = DEFAULT_REFERENCE_FRAME)] string referenceFrame,
            [InputPin(Name = "Frequency Hz", Description = "Frequency with which the datapoints are gathered", PropertyMode = PropertyMode.Default)] double frequency = 30,
            [InputPin(Name = "Nullify", Description = "Uses the last n datapoints to zero the output, where n is defined by NullifySampleSize", PropertyMode = PropertyMode.Default)] bool nullify = true,
            [InputPin(Name = "Timeout", Description = "Timeout in milliseconds", PropertyMode = PropertyMode.Default)] Int32 timeout = 0,
            [InputPin(Name = "Max Wrench Count", Description = "Maximum amount of wrench values to be read.", PropertyMode = PropertyMode.Default)] Int32 maxWrenchCount = 0
        )
        {
            currentNodeName = nodeName;

            const string SERVICE_NAME = "beginDataCollection";

            string serviceName = Path.Combine(nodeName, SERVICE_NAME);
            using (var client = rosClient.GlobalNodeHandle.ServiceClient<BeginDataCapture>(serviceName))
            {
                var srv = new BeginDataCapture();
                srv.req.frequency = frequency;
                srv.req.nullifyData = nullify;
                srv.req.referenceFrame = referenceFrame;

                Int32 seconds = timeout / 1000;
                Int32 nanoSeconds = timeout * 1000000;
                TimeData timeoutData = new TimeData(seconds, nanoSeconds);
                srv.req.timeout = new Messages.std_msgs.Duration(timeoutData);
                srv.req.maxWrenchCount = maxWrenchCount;
                if (!await client.CallAsync(srv))
                    throw new ServiceCallFailedException(serviceName);
                // Crash early when node does not return success
                if (!srv.resp.success)
                {
                    throw new Exception("Could not start data collection. " + srv.resp.message.ToString());
                }
                running = true;
            }
        }

        /// <summary>
        /// Makes sure that the node in background stops capturing data 
        /// </summary>
        protected override void OnStopped()
        {
            const string SERVICE_NAME = "abortReadingSensor";

            if (running)
            {
                string serviceName = Path.Combine(currentNodeName, SERVICE_NAME);
                using (var client = rosClient.GlobalNodeHandle.ServiceClient<Messages.std_srvs.Trigger>(serviceName))
                {
                    var srv = new Messages.std_srvs.Trigger();
                    if (!client.Call(srv))
                        throw new ServiceCallFailedException(serviceName);
                }
            }
        }
    }


    [Module(ModuleType = "Xamla.ForceTorqueToolbox.GetCapturedData", Description = "Get the captured wrench data.", Flow = true)]
    public class GetCollectedData
        : SingleInstanceMethodModule
    {
        const string DEFAULT_NODE_NAME = "/ft_sensor_client";
        const string SERVICE_NAME = "getCapturedData";
        IRosClientLibrary rosClient;

        public GetCollectedData(IGraphRuntime runtime, IRosClientLibrary rosClient)
            : base(runtime)
        {
            this.rosClient = rosClient;
        }

        [ModuleMethod]
        [OutputPin(Name = "Wrench Data", Description = "The WrenchStamped Data captured.")]
        public async Task<IEnumerable<Messages.geometry_msgs.WrenchStamped>> GetData(
            [InputPin(Name = "Node Name", Description = "The name of the toolbox node.", PropertyMode = PropertyMode.Default, DefaultValue = DEFAULT_NODE_NAME)] string nodeName)
        {
            string serviceName = Path.Combine(nodeName, SERVICE_NAME);
            using (var client = rosClient.GlobalNodeHandle.ServiceClient<GetCapturedData>(serviceName))
            {
                var srv = new GetCapturedData();
                if (!await client.CallAsync(srv))
                    throw new ServiceCallFailedException(serviceName);

                return srv.resp.wrenchData;
            }
        }
    }
}
