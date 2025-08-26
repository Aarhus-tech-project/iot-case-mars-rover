using System;
using System.IO;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Png;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;

public sealed class GridState
{
    private readonly object _lock = new();
    private byte[] _data = Array.Empty<byte>();
    public int Width { get; private set; }
    public int Height { get; private set; }
    public float CellSizeM { get; private set; }

    public void Update(int w, int h, float cellSizeM, ReadOnlySpan<byte> data)
    {
        if (data.Length != w * h) return; 
        lock (_lock)
        {
            if (_data.Length != data.Length) _data = new byte[data.Length];
            data.CopyTo(_data);
            Width = w; Height = h; CellSizeM = cellSizeM;
        }
    }

    public byte[] EncodePng(bool flipY = true)
    {
        lock (_lock)
        {
            if (_data.Length == 0 || Width == 0 || Height == 0) return Array.Empty<byte>();

            using var img = Image.LoadPixelData<L8>(_data, Width, Height); // 0..255 grayscale
            if (flipY) img.Mutate(c => c.Flip(FlipMode.Vertical));

            using var ms = new MemoryStream();
            img.Save(ms, new PngEncoder { ColorType = PngColorType.Grayscale });
            return ms.ToArray();
        }
    }
}