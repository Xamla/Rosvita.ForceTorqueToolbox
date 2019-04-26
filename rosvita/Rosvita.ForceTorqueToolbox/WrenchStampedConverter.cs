using System;
using System.Linq;
using System.Collections.Generic;
using System.Globalization;

using Newtonsoft.Json.Linq;

using Xamla.Types.Sequence;
using Xamla.Graph;
using Xamla.Types.Converters;

using Xamla.Robotics.Types;

namespace Rosvita.ForceTorqueToolbox
{
    public class WrenchDataConverter
        : ITypeConversionProvider
    {
        ISequence<string[]> WrenchStampedEnumerableToStringArraySequence(IEnumerable<Messages.geometry_msgs.WrenchStamped> wrenches)
        {
            return WrenchStampedSequenceToStringArraySequence(wrenches.ToSequence());
        }

        ISequence<string[]> WrenchStampedSequenceToStringArraySequence(ISequence<Messages.geometry_msgs.WrenchStamped> wrenches)
        {
            string[] header = new string[] { "Seconds", "Nanoseconds", "Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z", "Frame ID" };
            IEnumerable<string[]> data = new string[][] { };
            data = data.Append(header);
            foreach (var wrench in wrenches.ToEnumerable())
            {
                data = data.Append(WrenchStampedToStringArray(wrench));
            }
            return Sequence.ToSequence(data);
        }

        string[] WrenchStampedToStringArray(Messages.geometry_msgs.WrenchStamped wrenchStamped)
        {
            var wrench = wrenchStamped.wrench;
            var header = wrenchStamped.header;

            string forceX = wrench.force.x.ToString("g", CultureInfo.InvariantCulture);
            string forceY = wrench.force.y.ToString("g", CultureInfo.InvariantCulture);
            string forceZ = wrench.force.z.ToString("g", CultureInfo.InvariantCulture);
            string torqueX = wrench.torque.x.ToString("g", CultureInfo.InvariantCulture);
            string torqueY = wrench.torque.y.ToString("g", CultureInfo.InvariantCulture);
            string torqueZ = wrench.torque.z.ToString("g", CultureInfo.InvariantCulture);
            string frameString = header.frame_id;
            string secString = header.stamp.data.sec.ToString();
            string nSecString = header.stamp.data.nsec.ToString();
            return new string[] { secString, nSecString, forceX, forceY, forceZ, torqueX, torqueY, torqueZ, frameString };
        }

        string WrenchStampedToString(Messages.geometry_msgs.WrenchStamped wrenchStamped)
        {
            var wrench = wrenchStamped.wrench;
            var header = wrenchStamped.header;
            string forces = $"<{wrench.force.x.ToString("g", CultureInfo.InvariantCulture)}, {wrench.force.y.ToString("g", CultureInfo.InvariantCulture)}, {wrench.force.z.ToString("g", CultureInfo.InvariantCulture)}>";
            string torque = $"<{wrench.torque.x.ToString("g", CultureInfo.InvariantCulture)}, {wrench.torque.y.ToString("g", CultureInfo.InvariantCulture)}, {wrench.torque.z.ToString("g", CultureInfo.InvariantCulture)}>";
            string frameString = header.frame_id;
            string secString = header.stamp.data.sec.ToString("g", CultureInfo.InvariantCulture);
            string nSecString = header.stamp.data.nsec.ToString("g", CultureInfo.InvariantCulture);
            return $"TimeStamp: {{sec:{secString}, nsec: {nSecString} }}, Forces: {forces}, Forces: {torque}, Frame:'{frameString}'";
        }

        string WrenchStampedEnumerableStampedToString(IEnumerable<Messages.geometry_msgs.WrenchStamped> wrenches)
        {
            return wrenches.Select(wrench => WrenchStampedToString(wrench)).Aggregate("", (acc, x) => $"{acc}\n{x}");
        }

        public IEnumerable<ITypeConverter> GetConverters()
        {
            return new[]            {
                TypeConverter.Create<Messages.geometry_msgs.WrenchStamped, string>(WrenchStampedToString),
                TypeConverter.Create<IEnumerable<Messages.geometry_msgs.WrenchStamped>, string> (WrenchStampedEnumerableStampedToString),
                TypeConverter.Create<IEnumerable<Messages.geometry_msgs.WrenchStamped>, ISequence<string[]> >(WrenchStampedEnumerableToStringArraySequence),
            };
        }

        public IEnumerable<IDynamicTypeConverter> GetDynamicConverters()
        {
            return null;
        }

        public Dictionary<Type, Tuple<Func<object, JToken>, Func<JToken, object>>> GetSerializers()
        {
            return customSerializedTypes;
        }

        static readonly Dictionary<Type, Tuple<Func<object, JToken>, Func<JToken, object>>> customSerializedTypes = new Dictionary<Type, Tuple<Func<object, JToken>, Func<JToken, object>>> { };

        static WrenchDataConverter()
        {
        }
    }
}