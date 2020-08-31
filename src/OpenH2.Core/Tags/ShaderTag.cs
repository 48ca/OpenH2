﻿using OpenH2.Core.Representations;
using OpenH2.Core.Tags.Layout;
using OpenH2.Serialization.Layout;
using System.Numerics;

namespace OpenH2.Core.Tags
{
    [TagLabel(TagName.shad)]
    public class ShaderTag : BaseTag
    {
        public override string Name { get; set; }

        public ShaderTag(uint id) : base(id)
        {
        }

        [StringValue(0, 4)]
        public string StemTag { get; set; }

        [PrimitiveValue(4)]
        public TagRef<ShaderTemplateTag> ShaderTemplate { get; set; }

        [ReferenceArray(12)]
        public BitmapInfo[] BitmapInfos { get; set; }

        [ReferenceArray(32)]
        public ShaderArguments[] Arguments { get; set; }

        [ReferenceArray(44)]
        public BitmapReferenceSetting[] BitmapReferenceSettings { get; set; }

        [FixedLength(80)]
        public class BitmapInfo
        {
            [PrimitiveValue(4)]
            public TagRef<BitmapTag> DiffuseBitmap { get; set; }

            [PrimitiveValue(12)]
            public TagRef<BitmapTag> EmissiveBitmap { get; set; }

            [PrimitiveValue(16)]
            public float Param1 { get; set; }

            [PrimitiveValue(20)]
            public float Param2 { get; set; }

            [PrimitiveValue(24)]
            public float Param3 { get; set; }

            [PrimitiveValue(28)]
            public float Param4 { get; set; }

            [PrimitiveValue(48)]
            public TagRef<BitmapTag> AlphaBitmap { get; set; }
        }

        [FixedLength(124)]
        public class ShaderArguments
        {
            [PrimitiveValue(0)]
            public TagRef<ShaderTemplateTag> ShaderTemplate { get; set; }

            [ReferenceArray(4)]
            public ShaderMaps[] ShaderMaps { get; set; }

            [ReferenceArray(12)]
            public BitmapParameter2[] BitmapParamter2s { get; set; }

            [ReferenceArray(20)]
            public Vector4[] ShaderInputs { get; set; }

            [ReferenceArray(28)]
            public BitmapParameter4[] BitmapParamter4s { get; set; }
        }

        [FixedLength(12)]
        public class ShaderMaps
        {
            [PrimitiveValue(0)]
            public TagRef<BitmapTag> Bitmap { get; set; }

            [PrimitiveValue(4)]
            public Vector2 Something { get; set; }
        }

        [FixedLength(4)]
        public class BitmapParameter2
        {
            [PrimitiveValue(0)]
            public ushort ValueA { get; set; }

            [PrimitiveValue(0)]
            public ushort ValueB { get; set; }
        }

        [FixedLength(6)]
        public class BitmapParameter4
        {
            [PrimitiveValue(0)]
            public ushort ValueA { get; set; }

            [PrimitiveValue(0)]
            public ushort ValueB { get; set; }
        }

        [FixedLength(8)]
        public class BitmapReferenceSetting
        {
            [PrimitiveValue(0)]
            public short ValueA { get; set; }

            [PrimitiveValue(2)]
            public short ValueB { get; set; }

            [PrimitiveValue(4)]
            public uint BitmapId { get; set; }
        }
    }
}
