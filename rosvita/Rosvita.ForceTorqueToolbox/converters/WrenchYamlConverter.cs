using System;

using YamlDotNet.Core;
using YamlDotNet.Serialization;

using Rosvita.ForceTorqueToolbox.Types.Models;
using Messages.geometry_msgs;
using Rosvita.ForceTorqueToolbox;

namespace Rosvita.ForceTorqueToolbox.Types.YamlConverters
{
    /// <summary>
    /// TODO: This classes could perhaps be generated out of the JsonConverters using Reflection, to avoid duplicate code
    /// or one could parse to json first and then to yaml to serialize, and from yaml to json to deserialize
    /// </summary>
    public class WrenchStampedYamlConverter : IYamlTypeConverter
    {
        public bool Accepts(Type objectType) =>
            objectType == typeof(WrenchStamped);

        public object ReadYaml(IParser parser, Type type) =>
                null;

        public void WriteYaml(IEmitter emitter, object value, Type type)
        {
            WrenchStampedModel model = ((WrenchStamped)value)?.ToModel();
            IValueSerializer valueSerializer = new SerializerBuilder().BuildValueSerializer();
            valueSerializer.SerializeValue(emitter, model, model.GetType());
        }
    }
}