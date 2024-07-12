using System;
using UnityEngine;

public class RingString
{
    private readonly string[] buffer;
    private int head;
    private int tail;
    private int count;

    public RingString(int size)
    {
        this.buffer = new string[size];
        this.head = 0;
        this.tail = 0;
        this.count = 0;
    }

    public bool IsEmpty => count == 0;

    public bool IsFull => count == buffer.Length;

    public void Enqueue(string item)
    {
        buffer[tail] = item;
        tail = (tail + 1) % buffer.Length;
        count++;
    }

    public string Dequeue()
    {
        string item = buffer[head];
        head = (head + 1) % buffer.Length;
        count--;

        return item;
    }
}
