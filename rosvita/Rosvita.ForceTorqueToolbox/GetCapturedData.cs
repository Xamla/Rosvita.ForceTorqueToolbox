using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using Uml.Robotics.Ros;

namespace Messages.ft_sensor_client
{
    public class GetCapturedData : RosService
    {
        public override string ServiceType { get { return "ft_sensor_client/GetCapturedData"; } }
        public override string ServiceDefinition() { return @"---
geometry_msgs/WrenchStamped[] wrenchData
bool success
string message"; }
        public override string MD5Sum() { return "60b8eaec761021abc95fbc737cce0abb"; }

        public GetCapturedData()
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


            public override string MD5Sum() { return "d41d8cd98f00b204e9800998ecf8427e"; }
            public override bool HasHeader() { return false; }
            public override bool IsMetaType() { return false; }
            public override string MessageDefinition() { return @""; }
			public override string MessageType { get { return "ft_sensor_client/GetCapturedData__Request"; } }
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
                
            }

            public override bool Equals(RosMessage ____other)
            {
                if (____other == null)
					return false;

                bool ret = true;
                ft_sensor_client.GetCapturedData.Request other = (Messages.ft_sensor_client.GetCapturedData.Request)____other;

                return ret;
            }
        }

        public class Response : RosMessage
        {
				public Messages.geometry_msgs.WrenchStamped[] wrenchData;
				public bool success;
				public string message = "";



            public override string MD5Sum() { return "d41d8cd98f00b204e9800998ecf8427e"; }
            public override bool HasHeader() { return false; }
            public override bool IsMetaType() { return true; }
            public override string MessageDefinition() { return @"geometry_msgs/WrenchStamped[] wrenchData
bool success
string message"; }
			public override string MessageType { get { return "ft_sensor_client/GetCapturedData__Response"; } }
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
                
                //wrenchData
                hasmetacomponents |= false;
                arraylength = BitConverter.ToInt32(serializedMessage, currentIndex);
                currentIndex += Marshal.SizeOf(typeof(System.Int32));
                if (wrenchData == null)
                    wrenchData = new Messages.geometry_msgs.WrenchStamped[arraylength];
                else
                    Array.Resize(ref wrenchData, arraylength);
                for (int i=0;i<wrenchData.Length; i++) {
                    //wrenchData[i]
                    wrenchData[i] = new Messages.geometry_msgs.WrenchStamped(serializedMessage, ref currentIndex);
                }
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
                
                //wrenchData
                hasmetacomponents |= false;
                if (wrenchData == null)
                    wrenchData = new Messages.geometry_msgs.WrenchStamped[0];
                pieces.Add(BitConverter.GetBytes(wrenchData.Length));
                for (int i=0;i<wrenchData.Length; i++) {
                    //wrenchData[i]
                    if (wrenchData[i] == null)
                        wrenchData[i] = new Messages.geometry_msgs.WrenchStamped();
                    pieces.Add(wrenchData[i].Serialize(true));
                }
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
                
                //wrenchData
                arraylength = rand.Next(10);
                if (wrenchData == null)
                    wrenchData = new Messages.geometry_msgs.WrenchStamped[arraylength];
                else
                    Array.Resize(ref wrenchData, arraylength);
                for (int i=0;i<wrenchData.Length; i++) {
                    //wrenchData[i]
                    wrenchData[i] = new Messages.geometry_msgs.WrenchStamped();
                    wrenchData[i].Randomize();
                }
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
                ft_sensor_client.GetCapturedData.Response other = (Messages.ft_sensor_client.GetCapturedData.Response)____other;

                if (wrenchData.Length != other.wrenchData.Length)
                    return false;
                for (int __i__=0; __i__ < wrenchData.Length; __i__++)
                {
                    ret &= wrenchData[__i__].Equals(other.wrenchData[__i__]);
                }
                ret &= success == other.success;
                ret &= message == other.message;
                // for each SingleType st:
                //    ret &= {st.Name} == other.{st.Name};
                return ret;
            }
        }
    }
}
