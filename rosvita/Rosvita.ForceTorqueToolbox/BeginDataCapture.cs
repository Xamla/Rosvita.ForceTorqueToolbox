using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Runtime.InteropServices;
using uint8 = System.Byte;
using Uml.Robotics.Ros;


using Messages.std_msgs;
using String=System.String;
using Messages.geometry_msgs;

namespace Messages.ft_sensor_client
{
    public class BeginDataCapture : RosService
    {
        public override string ServiceType { get { return "ft_sensor_client/BeginDataCapture"; } }
        public override string ServiceDefinition() { return @"float64 frequency
string referenceFrame
duration timeout
int32 maxWrenchCount
bool nullifyData
---
bool success
string message"; }
        public override string MD5Sum() { return "3750640b1365c568027533160f2d83fb"; }

        public BeginDataCapture()
        {
            InitSubtypes(new Request(), new Response());
        }

        public Response Invoke(Func<Request, Response> fn, Request req)
        {
            RosServiceDelegate rsd = (m)=>{
                Request r = m as Request;
                if (r == null)
                    throw new Exception("Invalid Service Request Type");
                return fn(r);
            };
            return (Response)GeneralInvoke(rsd, (RosMessage)req);
        }

        public Request req { get { return (Request)RequestMessage; } set { RequestMessage = (RosMessage)value; } }
        public Response resp { get { return (Response)ResponseMessage; } set { ResponseMessage = (RosMessage)value; } }

        public class Request : RosMessage
        {
				public double frequency;
				public string referenceFrame = "";
				public Duration timeout = new Duration();
				public int maxWrenchCount;
				public bool nullifyData;


            public override string MD5Sum() { return "37f07cad16b25062e1a40d9226fa0871"; }
            public override bool HasHeader() { return false; }
            public override bool IsMetaType() { return false; }
            public override string MessageDefinition() { return @"float64 frequency
string referenceFrame
duration timeout
int32 maxWrenchCount
bool nullifyData"; }
			public override string MessageType { get { return "ft_sensor_client/BeginDataCapture__Request"; } }
            public override bool IsServiceComponent() { return true; }

            public Request()
            {
                
            }

            public Request(byte[] serializedMessage)
            {
                Deserialize(serializedMessage);
            }

            public Request(byte[] serializedMessage, ref int currentIndex)
            {
                Deserialize(serializedMessage, ref currentIndex);
            }

    

            public override void Deserialize(byte[] serializedMessage, ref int currentIndex)
            {
                int arraylength=-1;
                bool hasmetacomponents = false;
                byte[] thischunk, scratch1, scratch2;
                object __thing;
                int piecesize=0;
                IntPtr h;
                
                //frequency
                piecesize = Marshal.SizeOf(typeof(double));
                h = IntPtr.Zero;
                if (serializedMessage.Length - currentIndex != 0)
                {
                    h = Marshal.AllocHGlobal(piecesize);
                    Marshal.Copy(serializedMessage, currentIndex, h, piecesize);
                }
                if (h == IntPtr.Zero) throw new Exception("Memory allocation failed");
                frequency = (double)Marshal.PtrToStructure(h, typeof(double));
                Marshal.FreeHGlobal(h);
                currentIndex+= piecesize;
                //referenceFrame
                referenceFrame = "";
                piecesize = BitConverter.ToInt32(serializedMessage, currentIndex);
                currentIndex += 4;
                referenceFrame = Encoding.ASCII.GetString(serializedMessage, currentIndex, piecesize);
                currentIndex += piecesize;
                //timeout
                timeout = new Duration(new TimeData(
                        BitConverter.ToInt32(serializedMessage, currentIndex),
                        BitConverter.ToInt32(serializedMessage, currentIndex+Marshal.SizeOf(typeof(System.Int32)))));
                currentIndex += 2*Marshal.SizeOf(typeof(System.Int32));
                //maxWrenchCount
                piecesize = Marshal.SizeOf(typeof(int));
                h = IntPtr.Zero;
                if (serializedMessage.Length - currentIndex != 0)
                {
                    h = Marshal.AllocHGlobal(piecesize);
                    Marshal.Copy(serializedMessage, currentIndex, h, piecesize);
                }
                if (h == IntPtr.Zero) throw new Exception("Memory allocation failed");
                maxWrenchCount = (int)Marshal.PtrToStructure(h, typeof(int));
                Marshal.FreeHGlobal(h);
                currentIndex+= piecesize;
                //nullifyData
                nullifyData = serializedMessage[currentIndex++]==1;
            }

            public override byte[] Serialize(bool partofsomethingelse)
            {
                int currentIndex=0, length=0;
                bool hasmetacomponents = false;
                byte[] thischunk, scratch1, scratch2;
                List<byte[]> pieces = new List<byte[]>();
                GCHandle h;
                IntPtr ptr;
                int x__size;
                
                //frequency
                scratch1 = new byte[Marshal.SizeOf(typeof(double))];
                h = GCHandle.Alloc(scratch1, GCHandleType.Pinned);
                Marshal.StructureToPtr(frequency, h.AddrOfPinnedObject(), false);
                h.Free();
                pieces.Add(scratch1);
                //referenceFrame
                if (referenceFrame == null)
                    referenceFrame = "";
                scratch1 = Encoding.ASCII.GetBytes((string)referenceFrame);
                thischunk = new byte[scratch1.Length + 4];
                scratch2 = BitConverter.GetBytes(scratch1.Length);
                Array.Copy(scratch1, 0, thischunk, 4, scratch1.Length);
                Array.Copy(scratch2, thischunk, 4);
                pieces.Add(thischunk);
                //timeout
                pieces.Add(BitConverter.GetBytes(timeout.data.sec));
                pieces.Add(BitConverter.GetBytes(timeout.data.nsec));
                //maxWrenchCount
                scratch1 = new byte[Marshal.SizeOf(typeof(int))];
                h = GCHandle.Alloc(scratch1, GCHandleType.Pinned);
                Marshal.StructureToPtr(maxWrenchCount, h.AddrOfPinnedObject(), false);
                h.Free();
                pieces.Add(scratch1);
                //nullifyData
                thischunk = new byte[1];
                thischunk[0] = (byte) ((bool)nullifyData ? 1 : 0 );
                pieces.Add(thischunk);
                //combine every array in pieces into one array and return it
                int __a_b__f = pieces.Sum((__a_b__c)=>__a_b__c.Length);
                int __a_b__e=0;
                byte[] __a_b__d = new byte[__a_b__f];
                foreach(var __p__ in pieces)
                {
                    Array.Copy(__p__,0,__a_b__d,__a_b__e,__p__.Length);
                    __a_b__e += __p__.Length;
                }
                return __a_b__d;
            }

            public override void Randomize()
            {
                int arraylength=-1;
                Random rand = new Random();
                int strlength;
                byte[] strbuf, myByte;
                
                //frequency
                frequency = (rand.Next() + rand.NextDouble());
                //referenceFrame
                strlength = rand.Next(100) + 1;
                strbuf = new byte[strlength];
                rand.NextBytes(strbuf);  //fill the whole buffer with random bytes
                for (int __x__ = 0; __x__ < strlength; __x__++)
                    if (strbuf[__x__] == 0) //replace null chars with non-null random ones
                        strbuf[__x__] = (byte)(rand.Next(254) + 1);
                strbuf[strlength - 1] = 0; //null terminate
                referenceFrame = Encoding.ASCII.GetString(strbuf);
                //timeout
                timeout = new Duration(new TimeData(
                        Convert.ToInt32(rand.Next()),
                        Convert.ToInt32(rand.Next())));
                //maxWrenchCount
                maxWrenchCount = rand.Next();
                //nullifyData
                nullifyData = rand.Next(2) == 1;
            }

            public override bool Equals(RosMessage ____other)
            {
                if (____other == null)
					return false;

                bool ret = true;
                ft_sensor_client.BeginDataCapture.Request other = (Messages.ft_sensor_client.BeginDataCapture.Request)____other;

                ret &= frequency == other.frequency;
                ret &= referenceFrame == other.referenceFrame;
                ret &= timeout.data.Equals(other.timeout.data);
                ret &= maxWrenchCount == other.maxWrenchCount;
                ret &= nullifyData == other.nullifyData;
                return ret;
            }
        }

        public class Response : RosMessage
        {
				public bool success;
				public string message = "";



            public override string MD5Sum() { return "37f07cad16b25062e1a40d9226fa0871"; }
            public override bool HasHeader() { return false; }
            public override bool IsMetaType() { return false; }
            public override string MessageDefinition() { return @"bool success
string message"; }
			public override string MessageType { get { return "ft_sensor_client/BeginDataCapture__Response"; } }
            public override bool IsServiceComponent() { return true; }

            public Response()
            {
                
            }

            public Response(byte[] serializedMessage)
            {
                Deserialize(serializedMessage);
            }

            public Response(byte[] serializedMessage, ref int currentIndex)
            {
                Deserialize(serializedMessage, ref currentIndex);
            }

	

            public override void Deserialize(byte[] serializedMessage, ref int currentIndex)
            {
                int arraylength = -1;
                bool hasmetacomponents = false;
                int piecesize = 0;
                byte[] thischunk, scratch1, scratch2;
                IntPtr h;
                object __thing;
                
                //success
                success = serializedMessage[currentIndex++]==1;
                //message
                message = "";
                piecesize = BitConverter.ToInt32(serializedMessage, currentIndex);
                currentIndex += 4;
                message = Encoding.ASCII.GetString(serializedMessage, currentIndex, piecesize);
                currentIndex += piecesize;
            }

            public override byte[] Serialize(bool partofsomethingelse)
            {
                int currentIndex=0, length=0;
                bool hasmetacomponents = false;
                byte[] thischunk, scratch1, scratch2;
                List<byte[]> pieces = new List<byte[]>();
                GCHandle h;
                IntPtr ptr;
                int x__size;
                
                //success
                thischunk = new byte[1];
                thischunk[0] = (byte) ((bool)success ? 1 : 0 );
                pieces.Add(thischunk);
                //message
                if (message == null)
                    message = "";
                scratch1 = Encoding.ASCII.GetBytes((string)message);
                thischunk = new byte[scratch1.Length + 4];
                scratch2 = BitConverter.GetBytes(scratch1.Length);
                Array.Copy(scratch1, 0, thischunk, 4, scratch1.Length);
                Array.Copy(scratch2, thischunk, 4);
                pieces.Add(thischunk);
                //combine every array in pieces into one array and return it
                int __a_b__f = pieces.Sum((__a_b__c)=>__a_b__c.Length);
                int __a_b__e=0;
                byte[] __a_b__d = new byte[__a_b__f];
                foreach(var __p__ in pieces)
                {
                    Array.Copy(__p__,0,__a_b__d,__a_b__e,__p__.Length);
                    __a_b__e += __p__.Length;
                }
                return __a_b__d;
            }

            public override void Randomize()
            {
                int arraylength = -1;
                Random rand = new Random();
                int strlength;
                byte[] strbuf, myByte;
                
                //success
                success = rand.Next(2) == 1;
                //message
                strlength = rand.Next(100) + 1;
                strbuf = new byte[strlength];
                rand.NextBytes(strbuf);  //fill the whole buffer with random bytes
                for (int __x__ = 0; __x__ < strlength; __x__++)
                    if (strbuf[__x__] == 0) //replace null chars with non-null random ones
                        strbuf[__x__] = (byte)(rand.Next(254) + 1);
                strbuf[strlength - 1] = 0; //null terminate
                message = Encoding.ASCII.GetString(strbuf);
            }

            public override bool Equals(RosMessage ____other)
            {
                if (____other == null)
					return false;

                bool ret = true;
                ft_sensor_client.BeginDataCapture.Response other = (Messages.ft_sensor_client.BeginDataCapture.Response)____other;

                ret &= success == other.success;
                ret &= message == other.message;
                // for each SingleType st:
                //    ret &= {st.Name} == other.{st.Name};
                return ret;
            }
        }
    }
}
