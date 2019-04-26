using Rosvita.Project;
using System.Collections.Generic;

namespace Rosvita.ForceTorqueRobotiq
{
    [SensorComponent(
        Name = ConfigurationType,
        DisplayName = "F/T Sensor Robotiq",
        Manufacturer = "Robotiq",
        Version = "0.0.0.1",
        Description = "F/T Sensor Robotiq",
        Hidden = false,
        DefaultNodeName = "rq_sensor",
        DefaultRespawn = true,
        DefaultRespawnDelay = 1
    )]
    public class ForceTorqueRobotiqConfiguration
        : RosNodeConfigurationBase
    {
        public const string ConfigurationType = "ForceTorqueRobotiq";
        const int DEFAULT_MAX_RETRIES = 100;
        const string DEFAULT_SERIAL_ID = "";
        const string DEFAULT_FRAME_ID = "robotiq_ft_frame_id";
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

        public ForceTorqueRobotiqConfiguration()
        {
        }

        [ComponentProperty(DisplayName = "Max Retries", DefaultValue = DEFAULT_MAX_RETRIES)]
        public int MaxRetries { get; set; } = DEFAULT_MAX_RETRIES;

        [ComponentProperty(DisplayName = "Serial ID", Description="The serial id to be used. Searches an active one if empty.", DefaultValue = DEFAULT_SERIAL_ID)]
        public string SerialID { get; set; } = DEFAULT_SERIAL_ID;

        [ComponentProperty(DisplayName = "Frame ID", DefaultValue = DEFAULT_FRAME_ID)]
        public string FrameID { get; set; } = DEFAULT_FRAME_ID;
    }
}