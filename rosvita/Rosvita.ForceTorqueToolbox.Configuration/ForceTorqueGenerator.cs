using Rosvita.Parsers.Launch;
using Rosvita.Project;
using Rosvita.Project.Generators;
using System.IO;
using System.Linq;
using System.Xml;
using Xamla.Types;

namespace Rosvita.ForceTorqueToolbox
{
    [ConfigurationGenerator]
    public class ForceTorqueGenerator
        : ConfigurationGeneratorBase
    {
        public ForceTorqueGenerator()
            : base(nameof(ForceTorqueGenerator), GeneratorPriority.LaunchFileGenerator)
        {
        }

        protected override void GenerateInternal(IRosvitaRuntime runtime, IConfiguration configuration, IGeneratorContext context, IGeneratorMessageCollection messages)
        {
            var project = runtime.Project.Current;
            if (project == null)
                throw new XamlaException("Project must not be null.", XamlaError.ArgumentNull);

            // find matching components for this generator
            var toolBoxes = configuration.Where(x => x.ConfigurationType == ForceTorqueConfiguration.ConfigurationType)
                .OfType<IStandardComponent>()
                .Select(x => (ForceTorqueConfiguration)x.GetConfiguration())
                .Where(x => x.Enabled)
                .ToList();

            foreach (var toolBox in toolBoxes)
            {
                LaunchFileDocument launchFile = GenerateLaunchFile(toolBox);

                string launchFilename = $"ft_toolbox_{toolBox.Name}.launch";
                string launchTempFilePath = Path.Combine(context.TempOutputDirecotry, launchFilename);
                string launchFilePath = Path.Combine(context.FinalOutputDirecotry, launchFilename);
                launchFile.Save(launchTempFilePath);
                context.LaunchFilesToInclude.Add(launchFilePath);

                messages.AddInfo(this, $"Launch file'{project.MakeRelative(launchFilePath)}' written.");
            }
        }

        /*
<launch>
  <node name="ft_sensor_client" pkg="ft_sensor_client" type="ft_sensor_client" output="screen" respawn="true" respawn_delay="1">
    <param name="wrenchTopic" value="/rq_sensor/robotiq_force_torque_wrench" />
    <param name="bufferSize" value="200" />
    <param name="idleRate" value="10" />
  </node>
</launch>
         */
        private LaunchFileDocument GenerateLaunchFile(ForceTorqueConfiguration toolBox)
        {
            var launchFile = new LaunchFileDocument();

            var node = new NodeElement
            {
                Name = toolBox.Name,
                Pkg = "ft_sensor_client",
                Type = "ft_sensor_client",
                Output = "screen"
            };

            node.SetRespawnSettings(toolBox.Respawn, toolBox.RespawnDelay);
            node.AddParam("wrenchTopic", value: toolBox.Topic);
            node.AddParam("bufferSize", value: XmlConvert.ToString(toolBox.NullifyBufferSize));
            node.AddParam("idleRate", value: XmlConvert.ToString(toolBox.Rate));
            launchFile.AddNode(node);

            return launchFile;
        }
    }
}