
using System.Net.Mime;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ArduinoBluetoothAPI;
using UnityEngine.UI;
using System;
using System.Runtime.InteropServices;
using TMPro;
using System.Diagnostics;
using System.Text;
using Unity.VisualScripting;

public class globle : MonoBehaviour
{
	private BluetoothHelper helper;
	private bool isScanning;
	private LinkedList<BluetoothDevice> devices;
	public Text debugLog;
	public Text robotLog;
	public InputField deviceName;
	public GameObject bluetoothUI;
	public Image bluetoothFlag;
	public Sprite[] bluetoothstate;

	public struct ControlMsg
	{
		public bool begin;
		public bool stop;
		public byte robotmode;
	};

	public struct HandleMsg
	{
		public short run;
		public short turn;
		public short height;
		public short tilt;
	};
	public struct MasterMsg
	{
		ControlMsg control;
		HandleMsg handle;
	};
	public struct RobotMsg
	{
		public byte deviceState; // 电机状态
		public float v;
		public float height;
		public float pitch;
		public float yaw;
		public float roll;
	};

	public RobotMsg robotMsg;
	public bool receiveMsg(byte bt)
	{
		if (((dataprocess.getMsgFlag & 0xf0) == 0xf0) && ((dataprocess.getMsgFlag & 0x0f) != 0x0f))    // 获得头
		{
			dataprocess.rxData[98] = dataprocess.rxData[99];
			dataprocess.rxData[99] = bt;
			if (dataprocess.rxData[98] == dataprocess.controlMsgTailChar1 && dataprocess.rxData[99] == dataprocess.controlMsgTailChar2)
				return true;
			else
			{
				dataprocess.rxData[2 + dataprocess.rxDataSize] = bt;
				dataprocess.rxDataSize++;
			}
		}
		else
		{
			dataprocess.rxData[0] = dataprocess.rxData[1];
			dataprocess.rxData[1] = bt;
			if (dataprocess.rxData[0] == dataprocess.controlMsgHeadChar1 && dataprocess.rxData[1] == dataprocess.controlMsgHeadChar2)
				dataprocess.getMsgFlag |= 0xf0;
		}
		return false;
	}
	/// <summary>
	/// Start is called on the frame when a script is enabled just before
	/// any of the Update methods is called the first time.
	/// </summary>
	private void Start()
	{
	}

	private void Awake()
	{
		Log("Switch To BtUI");
		try
		{
			//BluetoothHelper.BLE = true;
			//获取 BluetoothHelper 实例
			helper = BluetoothHelper.GetInstance();
			//打开蓝牙
			helper.EnableBluetooth(true);
			//设置收发字符的长度，这里是重点，不设置则接，发 不了消息，要到数据缓存的一定的量才一次性发送
			helper.setFixedLengthBasedStream(1);

			// 连接成功的回调函数
			helper.OnConnected += () =>
			{
				Log("Connected Successfully");
				helper.StartListening();
				bluetoothFlag.sprite = bluetoothstate[1];
			};

			helper.OnConnectionFailed += () =>
			{
				Log("Connection Failed");
				bluetoothFlag.sprite = bluetoothstate[0];
				Disconnect();
			};

			helper.OnScanEnded += _devices =>
			{
				this.isScanning = false;
				this.devices = _devices;
			};

			// 没有找到设备的回调函数
			helper.OnServiceNotFound += serviceName =>
			{
				Log("Fail to find device: " + serviceName);
				Disconnect();// 断开连接
			};

			// 接收到消息的回调函数
			helper.OnDataReceived += () =>
			{
				byte[] data = helper.ReadBytes();
				// Log(data[0].ToString());
				for (int i = 0; i < data.Length; i++)
					receiveMsg(data[i]);
			};

		}

		catch (Exception e)
		{
			Log("Connection Error: " + e.Message);
			bluetoothFlag.sprite = bluetoothstate[0];
			Disconnect();
		}

	}
	public void Connect()
	{
		if (!string.IsNullOrEmpty(deviceName.text))
		{
			Log("Begin to link Bluetooth: " + deviceName.text);
			// 设置连接的设备名字
			helper.setDeviceName(deviceName.text);
			// 开始连接
			helper.Connect();
		}
		else
			Log("devicename is empty");
	}
	void OnDestroy()
	{
		Disconnect();
	}

	public void Disconnect()
	{
		Log("Disconnect");
		if (helper != null)
			helper.Disconnect();
		bluetoothFlag.sprite = bluetoothstate[0];
	}
	private int log_num = 0;
	public void Log(string s)
	{
		log_num++;
		if (log_num >= 8)
		{
			debugLog.text = "Debug Log";
			log_num = 0;
		}
		debugLog.text = debugLog.text + "\n" + s;
	}
	void send<T>(byte id, T str)
	{
		byte[] msg;
		byte head_id = id;
		msg = dataprocess.dataEncode<T>(str, id);
		helper.SendData(msg);
	}
	private void Update()
	{
		string Roll = "Roll: " + robotMsg.roll.ToString("F3") + "\n";// 保留三位小数
		string Pitch = "Pitch: " + robotMsg.pitch.ToString("F3") + "\n";// 保留三位小数
		string Yaw = "Yaw: " + robotMsg.yaw.ToString("F3") + "\n";// 保留三位小数
		robotLog.text = robotLog.text = "Robot Log\n" + Roll + Yaw + Pitch;
	}

	public void on_SwitchUIDd_changed(int value)
	{
		if (value == 0)
		{
			bluetoothUI.SetActive(true);
			Log("Switch To BtUI");
		}
		else if (value == 1)
		{
			bluetoothUI.SetActive(false);
			Log("Switch To HandleUI");
		}
	}
}


