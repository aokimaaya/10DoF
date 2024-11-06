using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor.Animations;
using System.Security.Authentication;
using System.Linq;
using WebSocketSharp;
using System.Text;
using Cysharp.Threading.Tasks;
using Sirenix.OdinInspector;
using Utf8Json;


/// <summary>
/// Read joint angle from BIOIK, then send value to AngleSetter
/// </summary>
public class GetAndSendAngleAIREC : MonoBehaviour
{
    [SerializeField] private BioIK.BioIK bioIK;

    public string ServerURL = "";
    public bool isConnect;
    public string m_signalingKey { get; private set; }

    private float m_timeout;
    private bool keep_connection;

    private bool ready_to_recieve_server = false;

    private WebSocket ws;

    private bool SendingMessage = false;

    public Animator AnimatorForHumanRig;

    private MotionManager motionManager;


    //private AngleSetter angleSetter;
    public AIRECMotorMsgs motorMsgs = new AIRECMotorMsgs();


    private void Start()
    {
        if (AnimatorForHumanRig != null)
            motionManager = new MotionManager();
    }


    [Button]
    public async UniTask TryConnect()
    {
        AccessServer();
        if (!isConnect) await Connect().Timeout(TimeSpan.FromSeconds(3));
    }

    
    public void AccessServer( string signalingKey = "", float timeout = 3.0f)
    {
        //ServerURL = url;
        if (ServerURL == "") 
        {
            Debug.LogError("[GetAndSendAngleAIREC] You must define a Server URL");
            return;
        }
        m_signalingKey = signalingKey;
        m_timeout = timeout;
        keep_connection = false;

        
        if (ws != null)
        {
            ws.Close();
            ws = null;
        }
        ws = new WebSocket(ServerURL);
        ws.WaitTime = System.TimeSpan.FromSeconds(m_timeout);
        if (ServerURL.StartsWith("wss"))
        {
            ws.SslConfiguration.EnabledSslProtocols =
                SslProtocols.Tls | SslProtocols.Tls11 | SslProtocols.Tls12;
        }

        ws.OnOpen += WSConnected;
        ws.OnMessage += WSOnMessage;

        ws.OnError += (sender, e) =>
        {
            Debug.LogError("[GetAndSendAngleAIREC] WebSocket Error Message: " + e.Message);
        };

        ws.OnClose += (sender, e) =>
        {
            Debug.Log("[GetAndSendAngleAIREC] WebSocket Closed. Status Code: "+ e.Code );
            isConnect = false;
            if (keep_connection)
            {
                Connect().Timeout(TimeSpan.FromSeconds(3)).Forget();
            }
        };
    }

    public async UniTask Connect()
    {
        if (isConnect == false)
        {
            //Debug.Log("Connect:" + connecting);
            try
            {
                ws.Connect();
            }
            catch
            {
                
            }
            
            //if (!keep_connection)
            //{
            //    keep_connection = true;
            //    await UniTask.RunOnThreadPool(() => StartPingingServer());
            //}
        }
        UniTask.Yield();
    }


    private void WSConnected(object sender, EventArgs e)
    {
        isConnect = true;
        ready_to_recieve_server = true;
        Debug.Log("[GetAndSendAngleAIREC] WS connected!");
    }


    public void WSSend(object data, bool show_info = true, bool ignore_response = false)
    {
        if (ready_to_recieve_server == false && ignore_response == false)
        {
            return;
        }
        if (ws == null || ws.ReadyState != WebSocketState.Open)
        {
            Debug.LogError("[GetAndSendAngleAIREC] WS is not connected. Unable to send message");
            return;
        }

        if (data is string s)
        {
            if (show_info) Debug.Log("[GetAndSendAngleAIREC] Sending WS data: " + s);
            ws.Send(s);
        }
        else
        {
            string str = JsonUtility.ToJson(data);
            if (show_info) Debug.Log("[GetAndSendAngleAIREC] Sending WS data: " + str);
            ws.Send(str);
        }
        ready_to_recieve_server = false;

    }

    public void SendAIRECMotorMsgs(AIRECMotorMsgs motorMsgs)
    {
        string data =
            motorMsgs.H1 + "," +
            motorMsgs.H2 + "," +
            motorMsgs.H3 + "," +
            motorMsgs.R1 + "," +
            motorMsgs.R2 + "," +
            motorMsgs.R3 + "," +
            motorMsgs.R4 + "," +
            motorMsgs.R5 + "," +
            motorMsgs.R6 + "," +
            motorMsgs.R7 + "," +
            motorMsgs.L1 + "," +
            motorMsgs.L2 + "," +
            motorMsgs.L3 + "," +
            motorMsgs.L4 + "," +
            motorMsgs.L5 + "," +
            motorMsgs.L6 + "," +
            motorMsgs.L7 + "," +
            motorMsgs.T1 + "," +
            motorMsgs.T2 + "," +
            motorMsgs.T3 + "," +
            motorMsgs.Vx + "," +
            motorMsgs.Vz + "," +
            motorMsgs.Vr;

        WSSend(data,false);
    }

    public void SendAIRECMotorMsgsJSON(AIRECMotorMsgs motorMsgs)
    {
        byte[] data = JsonSerializer.Serialize(motorMsgs);
        WSSend(data, false);
    }

    private void WSOnMessage(object sender, MessageEventArgs e)
    {
        //var content = Encoding.UTF8.GetString(e.RawData);
        //Debug.Log("Got Message: " + content);
        ready_to_recieve_server = true;
        Debug.Log("GotData");


        try
        {
            if (e.RawData.Length != 0 && AnimatorForHumanRig == null)
            {
                Debug.LogWarning("No animator found.");
                return;
            }
            else if (AnimatorForHumanRig == null)
            {
                return;
            }

            var content = Encoding.UTF8.GetString(e.RawData);
            if (content.Contains("Execute Motions:"))
            {
                Debug.Log("OnMessage: Execute Motion");
                var motion_command = content.Split(':')[1];
                motionManager.TransitionToAnimation(motion_command);
            }
            else
            {
                switch (content)
                {
                    case "Request Avaliable Motions":
                        Debug.Log("OnMessage: Status Request Motions");
                        WSSend("AvaliableMotion:" + motionManager.GetAnimationListAsString(), false);
                        break;
                    case "Stop Current Motion":
                        Debug.Log("OnMessage: Stop Current Motion");
                        motionManager.PauseAnimation();
                        break;
                    case "Resume Motion":
                        Debug.Log("OnMessage: Resume Motion");
                        motionManager.ResumeAnimation();
                        break;
                    case "Get Current Motion":
                        Debug.Log("OnMessage: Get Current Motion");
                        WSSend("CurrentMotion:" + motionManager.GetCurrentAnimation(), false);
                        break;
                }
            }
        }
        catch (Exception exec)
        {
            Debug.LogError(exec.ToString());
        }
    }

    private void Update()
    {
        UpdateAnglesFromBioIK();
        if (isConnect & SendingMessage)
        {
            SendAIRECMotorMsgs(motorMsgs);
        }
    }

    private void UpdateAnglesFromBioIK()
    {
        motorMsgs.H1 = (float)bioIK.FindSegment("neck.z").Joint.Z.CurrentValue * -1;
        motorMsgs.H2 = (float)bioIK.FindSegment("neck.x").Joint.X.CurrentValue ;
        motorMsgs.H3 = (float)bioIK.FindSegment("Head").Joint.Z.CurrentValue ;

        motorMsgs.R1 = (float)bioIK.FindSegment("Right shoulder").Joint.Z.CurrentValue * -1;
        motorMsgs.R2 = ((float)bioIK.FindSegment("arm.R.y").Joint.X.CurrentValue + 90.0f) *-1; // Offset to actual robot joint // Not moving now
        motorMsgs.R3 = (float)bioIK.FindSegment("uparm.R.x").Joint.Z.CurrentValue * -1;
        motorMsgs.R4 = (float)bioIK.FindSegment("elbow.R.z").Joint.Y.CurrentValue;
        motorMsgs.R5 = (float)bioIK.FindSegment("forearm.R.x").Joint.Z.CurrentValue * -1;
        motorMsgs.R6 = (float)bioIK.FindSegment("hand.R.001").Joint.Z.CurrentValue;
        motorMsgs.R7 = (float)bioIK.FindSegment("hand.R.002").Joint.Z.CurrentValue * -1;

        motorMsgs.L1 = (float)bioIK.FindSegment("Left shoulder").Joint.Z.CurrentValue ;
        motorMsgs.L2 = ((float)bioIK.FindSegment("arm.L.y").Joint.X.CurrentValue + 90.0f) * -1; // Offset to actual robot joint
        motorMsgs.L3 = (float)bioIK.FindSegment("uparm.L.x").Joint.Z.CurrentValue;
        motorMsgs.L4 = (float)bioIK.FindSegment("elbow.L.z").Joint.Y.CurrentValue * -1 ;
        motorMsgs.L5 = (float)bioIK.FindSegment("forearm.L.x").Joint.Z.CurrentValue;
        motorMsgs.L6 = (float)bioIK.FindSegment("hand.L.001").Joint.Z.CurrentValue * -1;
        motorMsgs.L7 = (float)bioIK.FindSegment("hand.L.002").Joint.Z.CurrentValue;

        motorMsgs.T1 = (float)bioIK.FindSegment("Spine.000").Joint.X.CurrentValue;
        motorMsgs.T2 = (float)bioIK.FindSegment("Spine.001").Joint.X.CurrentValue;
        motorMsgs.T3 = (float)bioIK.FindSegment("Spine.002").Joint.Z.CurrentValue * -1;

        motorMsgs.Vx = (float)bioIK.FindSegment("Root").Joint.X.CurrentVelocity;
        motorMsgs.Vz = (float)bioIK.FindSegment("Root").Joint.Z.CurrentVelocity;
        motorMsgs.Vr = (float)bioIK.FindSegment("Hips").Joint.Z.CurrentVelocity;
    }

    [Button]
    private void StartSendingMessage()
    {
        SendingMessage = true;
    }

    [Button]
    private void StopSendingMessage()
    {
        SendingMessage = false;
    }


    public class AIRECMotorMsgs
    {
        public float H1;
        public float H2;
        public float H3;
        public float R1;
        public float R2;
        public float R3;
        public float R4;
        public float R5;
        public float R6;
        public float R7;
        public float L1;
        public float L2;
        public float L3;
        public float L4;
        public float L5;
        public float L6;
        public float L7;
        public float T1;
        public float T2;
        public float T3;
        public float Vx;
        public float Vz;
        public float Vr;
    }

    private void OnDestroy()
    {
        keep_connection = false;
        if(isConnect) ws.Close();
    }


}


