using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using UnityEngine;
using TMPro;
using System;

public class UDPListener : MonoBehaviour
{
    UdpClient clientData;
    int portData = 5005;
    public int receiveBufferSize = 120000;

    public bool showDebug = false;
    IPEndPoint ipEndPointData;
    private object obj = null;
    private System.AsyncCallback AC;
    public byte[] receivedBytes;
    public int[] LidarCalidad = new int[360];
    public int[] LidarDistancias = new int[360];
    public GameObject pilonDistancias;

    public GameObject[] LidarPilonesDistancias = new GameObject[360];

    //Telemetría variables coche
    public long PosicionX = 0;
    public long PosicionY = 0;
    public long PosicionXObjetivo = 0;
    public long PosicionYObjetivo = 0;
    public ulong Encoder = 0;
    public uint Estado = 0;
    public uint Bateria = 5;
    public long angulo = 0;
    public long anguloObjetivo = 0;
    public bool firma1Detectada = false;
    public int firma1PosicionX = 0;
    public int firma1PosicionY = 0;
    public bool firma2Detectada = false;
    public int firma2PosicionX = 0;
    public int firma2PosicionY = 0;

    public TMP_Text txt_PosicionX;
    public TMP_Text txt_PosicionY;
    public TMP_Text txt_PosicionXObjetivo;
    public TMP_Text txt_PosiciónYObjetivo;
    public TMP_Text txt_Encoder;
    public TMP_Text txt_Estado;
    public TMP_Text txt_Bateria;
    public TMP_Text txt_angulo;
    public TMP_Text txt_anguloObjetivo;
    public TMP_Text txt_firma1Detectada;
    public TMP_Text txt_firma1PosicionX;
    public TMP_Text txt_firma1PosicionY;
    public TMP_Text txt_firma2Detectada;
    public TMP_Text txt_firma2PosicionX;
    public TMP_Text txt_firma2PosicionY;

    public MovimientoCoche coche;




    void Start()
    {
        InitializeUDPListener();
    }
    private void Update()
    {
        for(int i = 0; i<=359; i++)
        {
            float d = LidarDistancias[i];
            LidarDistancias[i] = 0;
            float posX = d * Mathf.Sin((Mathf.PI / 180) * i) / 1000;
            float posY = d * Mathf.Cos((Mathf.PI / 180) * i) / 1000;
            Debug.Log("posx = " + posX + " - posy = " + posY );
            Vector3 posicion = new Vector3(transform.position.x + posX, transform.position.y, transform.position.z + posY);
            Debug.Log("posicion del pilon:" + posicion.ToString());
            Instantiate(pilonDistancias, posicion, Quaternion.identity);
        }
        RefrescarEtiquetas();
        coche.posX = PosicionX;
        coche.posY = PosicionY;
    }
    void RefrescarEtiquetas()
    {
        txt_PosicionX.text = "PosX: " +PosicionX.ToString();
        txt_PosicionY.text = "PosY: " +PosicionY.ToString();
        txt_PosicionXObjetivo.text = "PosXObj: " + PosicionXObjetivo.ToString();
        txt_PosiciónYObjetivo.text = "PosYObj: " + PosicionYObjetivo.ToString();
        txt_Encoder.text = "Encoder: " + Encoder.ToString();
        txt_Estado.text = "Estado: " + Estado.ToString();
        txt_Bateria.text = "Bateria: " + Bateria.ToString();
        txt_angulo.text = "Angulo: " + angulo.ToString();
        txt_anguloObjetivo.text = "AnguloObj: " + anguloObjetivo.ToString();
        txt_firma1Detectada.text = "Firma1: " + firma1Detectada.ToString();
        txt_firma1PosicionX.text = "F1X: " + firma1PosicionX.ToString();
        txt_firma1PosicionY.text = "F1Y: " + firma1PosicionY.ToString();
        txt_firma2Detectada.text = "Firma2: " + firma2Detectada.ToString();
        txt_firma2PosicionX.text = "F2X: " + firma2PosicionX.ToString();
        txt_firma2PosicionY.text = "F2Y" + firma2PosicionY.ToString();
}
    public void InitializeUDPListener()
    {
        ipEndPointData = new IPEndPoint(IPAddress.Any, portData);
        clientData = new UdpClient();
        clientData.Client.ReceiveBufferSize = receiveBufferSize;
        clientData.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, optionValue: true);
        clientData.ExclusiveAddressUse = false;
        clientData.EnableBroadcast = true;
        clientData.Client.Bind(ipEndPointData);
        clientData.DontFragment = true;
        if (showDebug) Debug.Log("BufSize: " + clientData.Client.ReceiveBufferSize);
        AC = new System.AsyncCallback(ReceivedUDPPacket);
        clientData.BeginReceive(AC, obj);
        Debug.Log("UDP - Start Receiving..");
    }

    void ReceivedUDPPacket(System.IAsyncResult result)
    {
        //stopwatch.Start();
        receivedBytes = clientData.EndReceive(result, ref ipEndPointData);

        ParsePacket();

        clientData.BeginReceive(AC, obj);

        //stopwatch.Stop();
        //Debug.Log(stopwatch.ElapsedTicks);
        //stopwatch.Reset();
    } // ReceiveCallBack

    void ParsePacket()
    {
        //El primer byte indicará el tipo de paquete.
        //Dependiendo del tipo de paquete, éste contendrá cierta cantidad de datos
        //  -0 -> 5 datos     ->NA
        //  -1 -> 10 datos    ->NA
        //  -2 -> 15 datos    ->NA
        //  -3 -> 360 datos   ->Lidar Quality
        //  -4 -> 720 datos   ->Lidar Distances
        //  -5 -> 50 datos    ->Información general

        // work with receivedBytes
        Debug.Log("receivedBytes len = " + receivedBytes.Length);
        int cabecera = receivedBytes[0];
        if (cabecera == 3)
        {
            Debug.Log("Cabecera 3: Lidar Calidad ------------------");
            for (int i = 1; i <= 360; i++)
            {
                LidarCalidad[i - 1] = receivedBytes[i];
            }
        }
        else if (cabecera == 4)
        {
            Debug.Log("Cabecera 4: Lidar Distancias ------------------");
            for (int i = 1; i<=720; i = i+2)
            {
                int d = receivedBytes[i] << 8 | receivedBytes[i + 1];
                LidarDistancias[(i - 1) / 2] = d;
            }
        }
        else if (cabecera == 5)
        {
            Debug.Log("Cabecera 5: Informacion ------------------");
            Debug.Log("Contenido" + receivedBytes.ToString());
            Debug.Log("Human = " + String.Join(" ",
            new List<byte>(receivedBytes)
            .ConvertAll(i => i.ToString())
            .ToArray()));
            /*
            --Posicion x                        8 bytes
            --Posición y                        8 bytes
            --Posición x Objetivo               8 bytes
            --Posición y Objetivo               8 bytes
            --Encoder 32 uint32                 4 bytes
            --Estado 8bits  uint                1 byte
            --batería 8bits uint                1 byte
            --Ángulo 16 float                   4 bytes     
            --Angulo Objetivo 16 float          4 bytes
            --Cámara firma1 Detectada 1 byte    1 byte
            --Cámara firma1 x 8 bits            1 byte
            --Cámara firma1 y 8 bits            1 byte
            --Cámara firma2 Detectada 1byte     1 byte
            --Cámara firma2 x 8bits             1 byte
            --Cámara firma2 y 8bits             1 byte
            
            |XXXX|YYYY|MMMM|NNNN|QQQQ|W|E|RRRR|TTTT|U|I|O|A|S|D
             0000 0000 0111 1111 1111 2 2 2222 2222 3 3 3 3 3 3
             1234 5678 9012 3456 7890 1 2 3456 7890 1 2 3 4 5 6
            */
            int i = 1;
            PosicionX = (long)(receivedBytes[i] << 24 |
                                 receivedBytes[i + 1] << 16 |
                                 receivedBytes[i + 2] << 8 |
                                 receivedBytes[i + 3]);
            i = 5;
            PosicionY = receivedBytes[i] << 24 |
                                 receivedBytes[i + 1] << 16 |
                                 receivedBytes[i + 2] << 8 |
                                 receivedBytes[i + 3];
            i = 9;
            PosicionXObjetivo = receivedBytes[i] << 24 |
                                 receivedBytes[i + 1] << 16 |
                                 receivedBytes[i + 2] << 8 |
                                 receivedBytes[i + 3];
            i = 13;
            PosicionYObjetivo = receivedBytes[i] << 24 |
                                 receivedBytes[i + 1] << 16 |
                                 receivedBytes[i + 2] << 8 |
                                 receivedBytes[i + 3];
            i = 17;
            Encoder =   (ulong)( receivedBytes[i] << 24 |
                                 receivedBytes[i + 1] << 16 |
                                 receivedBytes[i + 2] << 8 |
                                 receivedBytes[i + 3]);
            i = 21;
            Estado = receivedBytes[i];
            i = 22;
            Bateria = receivedBytes[i];
            i = 23;
            angulo = receivedBytes[i] << 24 |
                                 receivedBytes[i + 1] << 16 |
                                 receivedBytes[i + 2] << 8 |
                                 receivedBytes[i + 3];
            i = 27;
            anguloObjetivo = (long)(receivedBytes[i] << 24 |
                                 receivedBytes[i + 1] << 16 |
                                 receivedBytes[i + 2] << 8 |
                                 receivedBytes[i + 3]);
            i = 31;
            firma1Detectada = receivedBytes[i] == 1;
            i = 32;
            firma1PosicionX = receivedBytes[i];
            i = 33;
            firma1PosicionY = receivedBytes[i];
            i = 34;
            firma2Detectada = receivedBytes[i] == 1;
            i = 35;
            firma2PosicionX = receivedBytes[i];
            i = 36;
            firma2PosicionY = receivedBytes[i];
        }
        else
        {
            Debug.Log("Cabecera Desconocida");
        }
    }

    void OnDestroy()
    {
        if (clientData != null)
        {
            clientData.Close();
        }

    }
}