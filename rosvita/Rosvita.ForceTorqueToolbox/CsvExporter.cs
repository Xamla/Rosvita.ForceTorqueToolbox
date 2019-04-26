using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace Rosvita.Utilities.Csv.Export
{
    public enum CsvWriterPadding
    {
        NoPadding,
        RightPadding,
        LeftPadding
    }

    /// <summary>  
    ///  This class uses offers the possibility to write csv-files.  
    /// </summary>  
    public class CsvExport
    {
        TextWriter writer;
        int numColumns;
        int numRows;

        string seperator;
        public CsvExport()
        {
            seperator = ",";
        }

        
        public void Write(IEnumerable<string[]> data, TextWriter output, CsvWriterPadding padding, string seperator = ",")
        {
            this.seperator = seperator;
            numRows = data.Count();
            numColumns = countCols(data);
            writer = output;

            int[] paddingMap = getPaddingMap(data);
            foreach (var row in data)
            {
                string line = concatenateTokens(row, padding, paddingMap);
                writer.WriteLine(line);
            }
        }

        private int countCols(IEnumerable<string[]> data)
        {
            int count = 0;
            foreach (var row in data)
            {
                var currSize = row.Count();
                if (count < currSize)
                    count = currSize;
            }
            return count;
        }

        private int[] getPaddingMap(IEnumerable<string[]> data)
        {
            int[] paddingMap = new int[numColumns];

            foreach (var row in data)
            {
                for (int i = 0; i < row.Count(); ++i)
                {
                    var token = row[i];

                    var currSize = token.Count();

                    if (paddingMap[i] < currSize)
                        paddingMap[i] = currSize;
                }
            }
            return paddingMap;
        }

        private string concatenateTokens(string[] tokens, CsvWriterPadding padding, int[] paddingMap)
        {
            //            StringBuilder output = new StringBuilder();

            string[] paddedTokens = new string[numColumns];
            for (int i = 0; i < tokens.Count(); ++i)
            {
                string token = tokens[i];
                switch (padding)
                {
                    case CsvWriterPadding.LeftPadding:
                        token = token.PadLeft(paddingMap[i]);
                        break;
                    case CsvWriterPadding.RightPadding:
                        token = token.PadRight(paddingMap[i]);
                        break;
                    case CsvWriterPadding.NoPadding:
                        break;
                    default:
                        break;
                }

                paddedTokens[i] = token;
            }
            return string.Join(seperator + " ", paddedTokens);
        }
    }
}
