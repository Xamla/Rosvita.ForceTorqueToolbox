using Rosvita.Project;
using System.Collections.Generic;

namespace Rosvita.ForceTorqueToolbox
{
    [RosComponent(
        Name = ConfigurationType,
        DisplayName = "Force Torque Toolbox",
        Manufacturer = "Xamla",
        Version = "0.0.1",
        Description = "Force Torque Toolbox",
        Hidden = false,
        DefaultNodeName = "ft_sensor_client",
        DefaultRespawn = true,
        DefaultRespawnDelay = 1
    )]
    public class ForceTorqueConfiguration
        : RosNodeConfigurationBase
    {
        public const string ConfigurationType = "ForceTorque";
        const string DEFAULT_WRENCH_TOPIC = "/rq_sensor/robotiq_force_torque_wrench";
        const double DEFAULT_RATE = 10;
        const int DEFAULT_BUFFER_SIZE = 200;

        public static List<IconImage> Icons => new List<IconImage>
        {
            new IconImage { Type = "thumb", Path = "/images/part-thumbs/sensor.png" }
        };

        public static ComponentMaintainer Maintainer => new ComponentMaintainer
        {
            Organization = "Xamla",
            Email = "support@xamla.com",
            Url = "http://www.xamla.com/"
        };

        [ComponentProperty(DisplayName = "Topic", DefaultValue = DEFAULT_WRENCH_TOPIC)]
        public string Topic { get; set; } = DEFAULT_WRENCH_TOPIC;

        [ComponentProperty(DisplayName = "Rate (Hz)", DefaultValue = DEFAULT_RATE)]
        public double Rate { get; set; } = DEFAULT_RATE;

        [ComponentProperty(DisplayName = "Nullify Buffer Size", Description = "The maximum amount of stored Wrench Data to zeroise the wrench data.", DefaultValue = DEFAULT_BUFFER_SIZE)]
        public int NullifyBufferSize { get; set; } = DEFAULT_BUFFER_SIZE;
    }
}