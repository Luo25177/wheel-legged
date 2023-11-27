
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;
using System.Text;

public class dataprocess
{
  static public byte[] rxData = new byte[50];    // 前两个为包头，末端两个为包尾
  static public byte[] head = new byte[2];
  static public byte[] tail = new byte[2];
  static public byte rxDataSize;
  static public bool getHead;
  static public byte controlMsgHeadChar1 = 0xFF;
  static public byte controlMsgHeadChar2 = 0xFE;
  static public byte controlMsgTailChar1 = 0x0A;
  static public byte controlMsgTailChar2 = 0x0D;

  public static byte[] dataEncode<T>(T stru, byte _id)
  {

    byte[] bys = StructToBytes(stru);

    byte[] msg_bytes = new byte[bys.Length + 5];
    msg_bytes[0] = controlMsgHeadChar1;
    msg_bytes[1] = controlMsgHeadChar2;
    msg_bytes[2] = _id;

    for (int i = 0; i < bys.Length; i++)
    {
      msg_bytes[3 + i] = bys[i];
    }

    msg_bytes[3 + bys.Length] = controlMsgTailChar1;
    msg_bytes[3 + bys.Length + 1] = controlMsgTailChar2;

    return msg_bytes;
  }

  public static byte[] StructToBytes(object structObj)
  {
    //得到结构体的大小
    int size = Marshal.SizeOf(structObj);
    //创建byte数组
    byte[] bytes = new byte[size];
    //分配结构体大小的内存空间
    IntPtr structPtr = Marshal.AllocHGlobal(size);
    //将结构体拷到分配好的内存空间
    Marshal.StructureToPtr(structObj, structPtr, false);
    //从内存空间拷到byte数组
    Marshal.Copy(structPtr, bytes, 0, size);
    //释放内存空间
    Marshal.FreeHGlobal(structPtr);
    //返回byte数组
    return bytes;
  }

  /// <summary>  
  /// 由byte数组转换为结构体  
  /// </summary>  
  public static T ByteToStructure<T>(byte[] dataBuffer, int startindex)
  {
    object structure = null;
    int size = Marshal.SizeOf(typeof(T));
    IntPtr allocIntPtr = Marshal.AllocHGlobal(size);
    try
    {
      Marshal.Copy(dataBuffer, startindex, allocIntPtr, size);
      structure = Marshal.PtrToStructure(allocIntPtr, typeof(T));
    }
    finally
    {
      Marshal.FreeHGlobal(allocIntPtr);
    }
    return (T)structure;
  }
}
