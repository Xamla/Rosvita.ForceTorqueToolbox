using System;
using System.IO;
using Xamla.Graph.MethodModule;
using YamlDotNet.Serialization;

using Rosvita.ForceTorqueToolbox.Types.YamlConverters;

namespace Xamla.Graph.Modules
{
    [Module(ModuleType = "Xamla.IO.YamlWriter", Description = "This Module writes a given object to a Yaml file.", Flow = false)]
    public class YamlWriterModule
        : SingleInstanceMethodModule
    {
        public YamlWriterModule(IGraphRuntime runtime)
            : base(runtime)
        {
        }

        [ModuleMethod]
        public static void YamlWriter(
            [InputPin(Name = "Data", Description = "Data item as array.", PropertyMode = PropertyMode.Allow)] Object Data,
            [InputPin(Name = "Path", Description = "Path to file to be written to.",PropertyMode = PropertyMode.Default, Editor = WellKnownEditors.SingleLineText, ResolvePath=false)]
                string path
        )
        {
            using (StreamWriter writer = File.CreateText(path))
            {
                var serializer = new SerializerBuilder().WithTypeConverter(new WrenchStampedYamlConverter()).Build();
                serializer.Serialize(writer, Data);
            }
        }
    }
}