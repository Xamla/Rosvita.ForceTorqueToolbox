using Rosvita.Parsers.Launch;
using Rosvita.Project;
using Rosvita.Project.Generators;
using System.IO;
using System.Linq;
using System.Xml;
using System.Collections.Generic;

using Xamla.Types;

namespace Rosvita.ForceTorqueRobotiq
{
    [ConfigurationGenerator]
    public class ForceTorqueRobotiqGenerator
        : ConfigurationGeneratorBase
    {
        public ForceTorqueRobotiqGenerator()
            : base(nameof(ForceTorqueRobotiqGenerator), GeneratorPriority.LaunchFileGenerator)
        {
        }

        protected override void GenerateInternal(IRosvitaRuntime runtime, IConfiguration configuration, IGeneratorContext context, IGeneratorMessageCollection messages)
        {
            var project = runtime.Project.Current;
            if (project == null)
                throw new XamlaException("Project must not be null.", XamlaError.ArgumentNull);

            // find matching components for this generator
            var robotiqConfigs = configuration.Where(x => x.ConfigurationType == ForceTorqueRobotiqConfiguration.ConfigurationType)
                .OfType<IStandardComponent>()
                .Select(x => (ForceTorqueRobotiqConfiguration)x.GetConfiguration())
                .Where(x => x.Enabled)
                .ToList();

            foreach (var robotiqConf in robotiqConfigs)
            {
                LaunchFileDocument launchFile = GenerateLaunchFile(robotiqConf, context);

                string launchFilename = $"ft_robotiq_{robotiqConf.Name}.launch";
                string launchTempFilePath = Path.Combine(context.TempOutputDirecotry, launchFilename);
                string launchFilePath = Path.Combine(context.FinalOutputDirecotry, launchFilename);
                launchFile.Save(launchTempFilePath);
                context.LaunchFilesToInclude.Add(launchFilePath);

                messages.AddInfo(this, $"Launch file'{project.MakeRelative(launchFilePath)}' written.");
            }
        }


        /*
<launch>
  <group ns="rq_sensor">
    <node name="rq_sensor" pkg="robotiq_force_torque_sensor" type="rq_sensor" output="screen" respawn="true" respawn_delay="1">
      <rosparam command="load" file="/home/xamla/Rosvita.Control/projects/rubiks_cube/.config/current/ft_robotiq_rq_sensor.yaml" />
    </node>
  </group>
</launch>
<launch>
  <node name="ft_sensor_client" pkg="ft_sensor_client" type="ft_sensor_client" output="screen" respawn="true" respawn_delay="1">
    <param name="wrenchTopic" value="/rq_sensor/robotiq_force_torque_wrench" />
    <param name="bufferSize" value="200" />
    <param name="idleRate" value="10" />
  </node>
</launch>
         */
        private LaunchFileDocument GenerateLaunchFile(ForceTorqueRobotiqConfiguration robotiqConf, IGeneratorContext context)
        {
            var launchFile = new LaunchFileDocument();

            var node = new NodeElement
            {
                Name = robotiqConf.Name,
                Pkg = "robotiq_force_torque_sensor",
                Type = "rq_sensor",
                Output = "screen"
            };

            node.SetRespawnSettings(robotiqConf.Respawn, robotiqConf.RespawnDelay);
            Dictionary<string, string> config = new Dictionary<string, string> { };
            config.Add("max_retries", XmlConvert.ToString(robotiqConf.MaxRetries));
            config.Add("serial_id", robotiqConf.SerialID);
            config.Add("frame_id", robotiqConf.FrameID);
            string configFileName = $"ft_robotiq_{robotiqConf.Name}.yaml";
            context.WriteYamlFileToTempOutputDirectory(configFileName, config);
            var parameter = new RosparamElement
            {
                Command = "load",
                File = Path.Combine(context.FinalOutputDirecotry, configFileName)
            };

            node.AddRosparam(parameter);
            // wrap node in a group element to avoid publishing its topic in root namespace
            var groupElement = new GroupElement
            {
                Ns = robotiqConf.Name
            };
            groupElement.AddNode(node);
            launchFile.AddGroup(groupElement);
            return launchFile;
        }
    }
}