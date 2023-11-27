
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;
using System.Text;

public class dataprocess
{
  static public byte[] rxData = new byte[100];    // 前两个为包头，末端两个为包尾
  static public byte rxDataSize;
  static public byte getMsgFlag;
  static public byte controlMsgHeadChar1 = 0xFF;
  static public byte controlMsgHeadChar2 = 0xFE;
  static public byte controlMsgTailChar1 = 0x0A;
  static public byte controlMsgTailChar2 = 0x0D;

  /// <summary>
  /// 编码数据
  /// </summary>
  /// <typeparam name="T"> 数据类型</typeparam>
  /// <param name="stru"></param>
  /// <param name="_head_id"></param>
  /// <returns></returns>
  public static byte[] dataEncode<T>(T stru, byte _head_id)
  {

    byte[] bys = StructToBytes(stru);
    byte sum = 0x00;

    byte[] msg_bytes = new byte[bys.Length + 6];
    msg_bytes[0] = controlMsgHeadChar1;
    msg_bytes[1] = controlMsgHeadChar2;
    msg_bytes[2] = _head_id;

    for (int i = 0; i < bys.Length; i++)
    {
      sum += bys[i];
      msg_bytes[3 + i] = bys[i];
    }

    msg_bytes[3 + bys.Length] = sum;
    msg_bytes[3 + bys.Length + 1] = controlMsgTailChar1;
    msg_bytes[3 + bys.Length + 2] = controlMsgTailChar2;

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
  public T ByteToStructure<T>(byte[] dataBuffer)
  {
    rxDataSize = 0;
    getMsgFlag = 0;
    object structure = null;
    int size = Marshal.SizeOf(typeof(T));
    IntPtr allocIntPtr = Marshal.AllocHGlobal(size);
    try
    {
      Marshal.Copy(dataBuffer, 0, allocIntPtr, size);
      structure = Marshal.PtrToStructure(allocIntPtr, typeof(T));
    }
    finally
    {
      Marshal.FreeHGlobal(allocIntPtr);
    }
    return (T)structure;
  }

}
