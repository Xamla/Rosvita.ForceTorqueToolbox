using Microsoft.Extensions.Logging;
using Rosvita.RosMonitor;
using System.Reflection;
using Xamla.Graph;
using Microsoft.Extensions.DependencyInjection;
using Uml.Robotics.Ros;

[assembly: GraphRuntimeInitializer(typeof(Rosvita.ForceTorqueToolbox.Initializer))]
namespace Rosvita.ForceTorqueToolbox
{
    class Initializer
        : IGraphRuntimeInitializer
    {
        public void Initialize(IGraphRuntime runtime)
        {
            ROS.RegisterMessageAssembly(Assembly.GetExecutingAssembly());
            runtime.ModuleFactory.RegisterAllModules(Assembly.GetExecutingAssembly());
            ExtraStaticModules.Init(
                runtime.ServiceLocator.GetService<ILoggerFactory>(),
                runtime.ServiceLocator.GetService<IRosClientLibrary>()
            );

            var converter = new WrenchDataConverter();

            foreach (var convert in converter.GetConverters())
                runtime.TypeConverters.AddConverter(convert);
        }
    }

    public static partial class ExtraStaticModules
    {
        static IRosClientLibrary rosClient;

        internal static void Init(ILoggerFactory loggerFactory, IRosClientLibrary rosClient)
        {
            ExtraStaticModules.rosClient = rosClient;
        }
    }
}
