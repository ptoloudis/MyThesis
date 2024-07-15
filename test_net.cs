using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

class Program
{
    static void Main(string[] args)
    {
        Console.Write("Enter the port number to send the message: ");
        int port;
        while (!int.TryParse(Console.ReadLine(), out port) || port <= 0 || port > 65535)
        {
            Console.WriteLine("Please enter a valid port number between 1 and 65535.");
        }

        UdpClient udpClient = new UdpClient();
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), port);

        string message = "hi";
        byte[] binaryMessage = Encoding.ASCII.GetBytes(message);

        // Frequency in Hertz
        int frequency = 500;
        int interval = 1000 / frequency; // Interval in milliseconds

        Console.WriteLine($"Sending messages to port {port} at {frequency}Hz...");

        while (true)
        {
            udpClient.Send(binaryMessage, binaryMessage.Length, remoteEndPoint);
            Thread.Sleep(interval);
        }
    }
}

