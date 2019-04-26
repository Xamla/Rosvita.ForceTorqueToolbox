using Rosvita.ForceTorqueToolbox.Types.Models;

using Messages.geometry_msgs;

namespace Rosvita.ForceTorqueToolbox
{
    public static class WrenchExtensions
    {
        public static WrenchStampedModel ToModel(this WrenchStamped wrenchStamped) =>
            new WrenchStampedModel
            {
                Wrench = new WrenchModel
                {
                    Frame = wrenchStamped.header.frame_id,
                    Force = new double[] { wrenchStamped.wrench.force.x, wrenchStamped.wrench.force.y, wrenchStamped.wrench.force.z },
                    Torque = new double[] { wrenchStamped.wrench.torque.x, wrenchStamped.wrench.torque.y, wrenchStamped.wrench.torque.z }
                },
                TimeStamp = new TimeStampModel
                {
                    seconds = wrenchStamped.header.stamp.data.sec,
                    nanoSeconds = wrenchStamped.header.stamp.data.nsec
                }
            };

        public static WrenchStamped ToWrenchStamped(this WrenchStampedModel model)
        {
            var wrenchStamped = new WrenchStamped();
            wrenchStamped.wrench = new Wrench
            {
                force = new Vector3 { x = model.Wrench.Force[0], y = model.Wrench.Force[1], z = model.Wrench.Force[2] },
                torque = new Vector3 { x = model.Wrench.Torque[0], y = model.Wrench.Torque[1], z = model.Wrench.Torque[2] },
            };
            return wrenchStamped;
        }
    }
}