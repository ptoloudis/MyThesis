using System;
using UnityEngine;

public class RingBuffer
{
    private readonly ushort[][] buffer;
    private int head;
    private int tail;
    private int count;
    private readonly int arraySize;

    public RingBuffer(int size, int arraySize)
    {
        this.buffer = new ushort[size][];
        this.arraySize = arraySize;
        this.head = 0;
        this.tail = 0;
        this.count = 0;
    }

    public bool IsEmpty => count == 0;

    public bool IsFull => count == buffer.Length;

    public void Enqueue(ushort[] item)
    {
        buffer[tail] = item;
        tail = (tail + 1) % buffer.Length;
        count++;
    }

    public ushort[] Dequeue()
    {
        ushort[] item = buffer[head];
        head = (head + 1) % buffer.Length;
        count--;

        return item;
    }
}
