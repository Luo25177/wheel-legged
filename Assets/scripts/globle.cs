
using System.ComponentModel;
using System.Runtime.CompilerServices;
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
	public GameObject handleUI;
	public Image bluetoothFlag;
	public Sprite[] bluetoothstate;
	public Image[] canstate;

	public struct ControlMsg
	{
		public byte begin;
		public byte stop;
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
		public ControlMsg control;
		public HandleMsg handle;
	};
	public struct RobotMsg
	{
		public byte deviceState; // 电机状态 0000 0000 : 0 0 WL BL FL WR BR FR
		public float v;
		public float height;
		public float pitch;
		public float yaw;
		public float roll;
	};

	public RobotMsg robotMsg;
	public MasterMsg master;
	public void receiveMsg(byte bt)
	{
		if (dataprocess.getHead)    // 获得头
		{
			dataprocess.tail[0] = dataprocess.tail[1];
			dataprocess.tail[1] = bt;
			if (dataprocess.tail[0] == dataprocess.controlMsgTailChar1 && dataprocess.tail[1] == dataprocess.controlMsgTailChar2)
			{
				robotMsg = dataprocess.ByteToStructure<RobotMsg>(dataprocess.rxData, 1);
				dataprocess.getHead = false;
				dataprocess.rxDataSize = 0;
			}
			else
			{
				dataprocess.rxData[dataprocess.rxDataSize] = bt;
				dataprocess.rxDataSize++;
			}
		}
		else
		{
			dataprocess.head[0] = dataprocess.head[1];
			dataprocess.head[1] = bt;
			if (dataprocess.head[0] == dataprocess.controlMsgHeadChar1 && dataprocess.head[1] == dataprocess.controlMsgHeadChar2)
				dataprocess.getHead = true;
		}
		if (dataprocess.rxDataSize >= 50)
		{
			dataprocess.rxDataSize = 0;
			dataprocess.getHead = false;
		}
	}
	private void Start()
	{
		master.handle.height = 20;
		master.handle.run = 0;
		master.handle.turn = 0;
		master.handle.tilt = 0;
		master.control.stop = 1;
		master.control.begin = 0;
		master.control.robotmode = 0;
	}

	private void Awake()
	{
		Log("Switch To BtUI");
		try
		{
			// TODO: 初始高度暂定
			master.handle.height = 20;
			bluetoothUI.SetActive(true);
			handleUI.SetActive(false);
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
				bluetoothButton.image.sprite = bluetoothbuttonstyle[1];
				hasconnected = true;
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
		bluetoothButton.image.sprite = bluetoothbuttonstyle[0];
		hasconnected = false;
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
	private void Update()
	{
		// TODO: 数据更新
		string Roll = "Roll: " + robotMsg.roll.ToString("F3") + "\n";
		string Pitch = "Pitch: " + robotMsg.pitch.ToString("F3") + "\n";
		string Yaw = "Yaw: " + robotMsg.yaw.ToString("F3") + "\n";
		string Height = "Height: " + robotMsg.height.ToString("F3") + "\n";
		string V = "V: " + robotMsg.v.ToString("F3") + "\n";
		robotLog.text = robotLog.text = "Robot Log\n" + Roll + Yaw + Pitch + Height + V;
		// 摇杆数据更新
		runforward();
		swerve();
		canStateUpdate();
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

	void send<T>(T str, byte id)
	{
		byte[] msg = dataprocess.dataEncode<T>(str, id);
		helper.SendData(msg);
	}

	int bluetoothbuttonvalue = 0;
	public Sprite[] bluetoothbuttonstyle;
	public Button bluetoothButton;
	bool hasconnected;
	public void bluetoothButtonFunc()
	{
		if (hasconnected)
			Disconnect();
		else Connect();
	}

	int beginbuttonvalue = 0;
	public Button beginbutton;
	public void beginButtonFunc()
	{
		beginbuttonvalue++;
		if (beginbuttonvalue % 2 == 1)
		{
			beginbutton.image.color = new Color(0, 255, 0);
			master.control.begin = 1;
			beginHandleStream();
		}
		else
		{
			beginbutton.image.color = new Color(255, 0, 0);
			master.control.begin = 0;
			stopHandleStream();
		}
		send<ControlMsg>(master.control, 1);
	}
	private static short limitInRange(short val, short max, short min)
	{
		if (val > max) val = max;
		if (val < min) val = min;
		return val;
	}
	// 左倾
	public void tiltleft()
	{
		// TODO: 具体根据传感器数据来规定
		master.handle.tilt += 1;
		master.handle.tilt = limitInRange(master.handle.tilt, (short)10, (short)-10);
	}
	// 右倾
	public void tiltright()
	{
		master.handle.tilt -= 1;
		master.handle.tilt = limitInRange(master.handle.tilt, (short)10, (short)-10);
	}
	// 底盘上抬
	public void turnup()
	{
		master.handle.height += 1;
		master.handle.height = limitInRange(master.handle.height, (short)35, (short)20);
	}
	// 底盘下沉
	public void turndown()
	{
		master.handle.height -= 1;
		master.handle.height = limitInRange(master.handle.height, (short)35, (short)20);
	}
	public VariableJoystick left;
	public VariableJoystick right;
	public void runforward()
	{
		master.handle.run = (short)(100 * left.Vertical);
	}
	public void swerve()
	{
		master.handle.turn = (short)(100 * right.Horizontal);
	}

	Coroutine beginHandleCoroutine;
	// 开启摇杆输出流
	public void beginHandleStream()
	{
		beginHandleCoroutine = StartCoroutine("handleStream");
	}
	// 关闭摇杆输出流
	public void stopHandleStream()
	{
		StopCoroutine(beginHandleCoroutine);
	}
	// 摇杆数据输出流
	IEnumerator handleStream()
	{
		while (true)
		{
			send<HandleMsg>(master.handle, 2);
			yield return new WaitForSeconds(0.08f);
		}
	}

	public Button changeUIButton;
	public Sprite[] changeUIstyle;
	int changeUIvalue;
	public void changeUIFunc()
	{
		changeUIvalue++;
		if (changeUIvalue % 2 == 1)
		{
			bluetoothUI.SetActive(false);
			handleUI.SetActive(true);
			Log("Switch To HandleUI");
			changeUIButton.image.sprite = changeUIstyle[1];
		}
		else
		{
			bluetoothUI.SetActive(true);
			handleUI.SetActive(false);
			Log("Switch To BtUI");
			changeUIButton.image.sprite = changeUIstyle[0];
		}
	}
	public void canStateUpdate()
	{
		for (int i = 0; i < 6; i++)
		{
			if ((robotMsg.deviceState & (0x01 << i)) != 0)
				canstate[i].color = new Color(0, 255, 0);
			else
				canstate[i].color = new Color(255, 0, 0);
		}
	}
}
