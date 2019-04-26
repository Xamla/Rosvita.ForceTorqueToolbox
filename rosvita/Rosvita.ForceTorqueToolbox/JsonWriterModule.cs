using System;
using System.IO;
using System.Collections.Generic;
using Xamla.Graph.MethodModule;

using Newtonsoft.Json;

using Xamla.Robotics.Types.JsonConverters;
using Rosvita.ForceTorqueToolbox.Types.JsonConverters;

namespace Xamla.Graph.Modules
{
    [Module(ModuleType = "Xamla.IO.JsonWriter",  Description = "This Module writes a given object to a Json file.", Flow = false)]
    public class JsonWriterModule
        : SingleInstanceMethodModule
    {
        public JsonWriterModule(IGraphRuntime runtime)
            : base(runtime)
        {
        }

        [ModuleMethod]
        public static void CsvWriter(
            [InputPin(Name = "Data", Description = "Data item as array.", PropertyMode = PropertyMode.Allow)] Object Data,
            [InputPin(Name = "Path", Description = "Path to file to be written to.",PropertyMode = PropertyMode.Default, Editor = WellKnownEditors.SingleLineText, ResolvePath=false)]
                string path
        )
        {
            using (StreamWriter writer = File.CreateText(path))
            {
                // TODO: Not sure if this are all the known Converters 
                List<JsonConverter> converters = new List<JsonConverter>(RoboticsJsonConverters.All);
                converters.Add(new WrenchStampedJsonConverter());
                JsonConvert.DefaultSettings = () => new JsonSerializerSettings
                {
                    Converters = converters.ToArray(),
                    Formatting = Newtonsoft.Json.Formatting.Indented
                };
                string dataString = JsonConvert.SerializeObject(Data);
                writer.WriteLine(dataString);
            }

        }
    }
}