
namespace Rosvita.ForceTorqueToolbox.Types.Models
{
    public class WrenchModel
    {
        public string Frame { get; set; }
        public double[] Force { get; set; }
        public double[] Torque { get; set; }
    }
    public class TimeStampModel
    {
        public int seconds { get; set; }
        public int nanoSeconds { get; set; }
    }

    public class WrenchStampedModel
    {
        public WrenchModel Wrench { get; set; }
        public TimeStampModel TimeStamp { get; set; }
    }
}
