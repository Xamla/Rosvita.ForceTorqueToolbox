using System.IO;
using Xamla.Graph.MethodModule;

using Xamla.Types.Sequence;
using Rosvita.Utilities.Csv.Export;

namespace Xamla.Graph.Modules
{

    [Module(ModuleType = "Xamla.IO.CsvWriter",  Description = "This Module writes a Sequence of string arrays to a csv file", Flow = false)]
    public class CsvWriterModule
        : SingleInstanceMethodModule
    {
        public CsvWriterModule(IGraphRuntime runtime)
            : base(runtime)
        {

        }

        [ModuleMethod]
        public static void CsvWriter(
            [InputPin(Name = "Data", Description = "Data item as array, with sequence building the rows and the nested string array  building the column entries.", PropertyMode = PropertyMode.Allow)] ISequence<string[]> Data,
            [InputPin(Name = "Path", Description = "Path to file to be written to.",PropertyMode = PropertyMode.Default, Editor = WellKnownEditors.SingleLineText, ResolvePath=false)]
                string path,
            [InputPin(Name = "Delimiter", Description = "Character which is used as delimiter between two data fields", PropertyMode = PropertyMode.Always, Editor = WellKnownEditors.SingleLineText)]
                string delimiter = ";",
            [InputPin(Name = "Padding", Description = "Pading of the entries", PropertyMode = PropertyMode.Always, Editor = WellKnownEditors.DropDown)]
                CsvWriterPadding padding = CsvWriterPadding.NoPadding
        )
        {
            using (StreamWriter writer = File.CreateText(path))
            {
                CsvExport csvWriter = new CsvExport();
                csvWriter.Write(Sequence.ToEnumerable(Data), writer, padding, delimiter);
            }
        }
    }
}