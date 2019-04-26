using System;
using Newtonsoft.Json.Linq;
using Newtonsoft.Json;

using Messages.geometry_msgs;
using Rosvita.ForceTorqueToolbox;
using Rosvita.ForceTorqueToolbox.Types.Models;

namespace Rosvita.ForceTorqueToolbox.Types.JsonConverters
{
    public class WrenchStampedJsonConverter : JsonConverter
    {
        public override bool CanConvert(Type objectType) =>
            objectType == typeof(WrenchStamped);

        public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer) =>
            serializer.Deserialize<WrenchStampedModel>(reader)?.ToWrenchStamped();

        public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer) =>
            serializer.Serialize(writer, ((WrenchStamped)value)?.ToModel());
    }
}

