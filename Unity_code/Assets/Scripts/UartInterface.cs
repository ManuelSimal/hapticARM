using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.IO.Ports;
using System.Threading;
using UnityEngine.UI;
using System.Text;
using System.Linq;

public struct Axes
{
    public float x;
    public float y;
    public float z;
    public Axes(float x, float y, float z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }
};

public struct uData
{
    public Quaternion q;
    public Axes p;
    public uData(Quaternion q, Axes p)
    {
        this.q = q;
        this.p = p;
    }
};




public class UartInterface : MonoBehaviour {
    private static SerialPort _serialPort;
    private Thread t_thread;
    private static Queue queue;
    private static Semaphore dataRequestSemaphore;

    //RX thread constants
    const int IMU_PKG_LENGTH = 56;
    const int RX_BUF_LENGTH = 70;

    //IMU variables

    public GameObject target;
    public GameObject target1;

    // Use this for initialization
    void Start () {
        queue = new Queue();
        dataRequestSemaphore = new Semaphore(0, 1);

        /*Initialize the serial port*/
        _serialPort = new SerialPort("COM10", 115200, Parity.None, 8, StopBits.One)
        {
            Handshake = Handshake.None,
            ReadTimeout = 6000,
            WriteTimeout = 500,
            RtsEnable = true,
            ReadBufferSize = RX_BUF_LENGTH,
            Encoding = Encoding.ASCII
        };

        _serialPort.Open();

        t_thread = new Thread(UartReceive);
        dataRequestSemaphore.Release();
        t_thread.Start();
    }
	
	// Update is called once per frame
	void FixedUpdate () {
        bool dataAvailable = (queue.Count > 0);
        uData imuData;
        
        if(dataAvailable)
        {
            imuData = (uData) queue.Dequeue();
            dataRequestSemaphore.Release();

            /*Position of the virtual point*/
            Vector3 direction = new Vector3(imuData.p.x, imuData.p.y, imuData.p.z);
            target1.transform.position = direction;

            /*Change the 3D orientation of an specified element*/
            target.transform.rotation = target.transform.rotation*imuData.q;

        }
    }


    public void UartReceive()
    {
        byte[] recBuf = new byte[RX_BUF_LENGTH];
        uData rxIMU_Data;

        while (true)
        {
            //Debug.Log("Entering semaphore wait...");
            dataRequestSemaphore.WaitOne();
            //Debug.Log("Exiting semaphore wait");
            _serialPort.Write("a");
            int bytesRead = 0;
            int bytesCounter = IMU_PKG_LENGTH;
            int offset = 0;
            double w = 0;
            double x = 0;
            double y = 0;
            double z = 0;
            double px = 0;
            double py = 0;
            double pz = 0;

            while (bytesCounter > 0)
            {
                bytesRead = _serialPort.Read(recBuf, offset, bytesCounter);
                offset += bytesRead;
                bytesCounter -= bytesRead;
            }

            w = BitConverter.ToDouble(recBuf, 0);
            x = BitConverter.ToDouble(recBuf, 8);
            y = BitConverter.ToDouble(recBuf, 16);
            z = BitConverter.ToDouble(recBuf, 24);
            px = BitConverter.ToDouble(recBuf, 32);
            py = BitConverter.ToDouble(recBuf, 40);
            pz = BitConverter.ToDouble(recBuf, 48);

            rxIMU_Data.q.w = Convert.ToSingle(w);
            rxIMU_Data.q.x = Convert.ToSingle(-y);
            rxIMU_Data.q.y = Convert.ToSingle(-z);
            rxIMU_Data.q.z = Convert.ToSingle(x);
            rxIMU_Data.p.x = Convert.ToSingle(-py);
            rxIMU_Data.p.y = Convert.ToSingle(pz);
            rxIMU_Data.p.z = Convert.ToSingle(px);

            Debug.Log("x = " + px);
            Debug.Log("y = " + py);
            Debug.Log("z = " + pz);

            queue.Enqueue(rxIMU_Data);

            Thread.Sleep(2);
        }
    }

public void OnDestroy()
 {
     t_thread.Abort();
     _serialPort.Close();
     Debug.Log("UART closed");
 }
}
