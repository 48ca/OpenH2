﻿using System;
using OpenH2.Core.Enums.Texture;
using OpenH2.Core.Offsets;
using OpenH2.Core.Parsing;
using OpenH2.Core.Tags.Layout;

namespace OpenH2.Core.Tags
{
    [TagLabel("bitm")]
    public class Bitmap : BaseTag
    {
        public override string Name { get; set; }

        public Bitmap(uint id) : base(id)
        {
        }

        [PrimitiveValue(0)]
        public TextureType TextureType { get; set; }

        [PrimitiveValue(2)]
        public TextureFormat TextureFormat { get; set; }

        [PrimitiveValue(4)]
        public TextureUsage TextureUsage { get; set; }

        [PrimitiveValue(52)]
        public short MipMapCount { get; set; }

        [StringValue(80, 4)]
        public string Tag { get; set; }

        [PrimitiveValue(84)]
        public short Width { get; set; }

        [PrimitiveValue(86)]
        public short Height { get; set; }

        [PrimitiveValue(88)]
        public short Depth { get; set; }

        [PrimitiveValue(90)]
        public short Type { get; set; }

        [PrimitiveValue(92)]
        public short Format { get; set; }

        [PrimitiveValue(94)]
        public TextureProperties Properties { get; set; }

        [PrimitiveValue(96)]
        public short RegX { get; set; }

        [PrimitiveValue(98)]
        public short RegY { get; set; }

        [PrimitiveValue(100)]
        public short MipMapCount2 { get; set; }

        [PrimitiveValue(102)]
        public short PixelOffset { get; set; }
        
        [PrimitiveArray(108, 6)]
        public uint[] LodOffsets { get; set; }

        [PrimitiveArray(132, 6)]
        public uint[] LodSizes { get; set; }

        public BitmapLevelOfDetail[] LevelsOfDetail { get; set; }

        [PrimitiveValue(156)]
        public uint ID { get; set; }

        

        public class BitmapLevelOfDetail
        {
            public NormalOffset Offset { get; set; }

            public uint Size { get; set; }

            public Memory<byte> Data { get; set; } = Memory<byte>.Empty;
        }

        public override void PopulateExternalData(TrackingReader sceneReader)
        {
            LevelsOfDetail = new BitmapLevelOfDetail[6];

            for (int i = 0; i < 6; i++)
            {
                var lod = new BitmapLevelOfDetail();

                lod.Offset = new NormalOffset((int)this.LodOffsets[i]);
                lod.Size = this.LodSizes[i];

                if (lod.Offset.Location == Enums.DataFile.Local && lod.Offset.Value != 0 && lod.Offset.Value != int.MaxValue && lod.Size != 0)
                {
                    lod.Data = sceneReader.Chunk(lod.Offset.Value, (int)lod.Size, "Bitmap").AsMemory();
                }

                LevelsOfDetail[i] = lod;
            }
        }
    }
}
