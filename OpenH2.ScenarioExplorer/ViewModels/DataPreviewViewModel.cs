﻿using System;
using System.Collections.Generic;
using System.Text;
using OpenH2.Core.Extensions;
using PropertyChanged;

namespace OpenH2.ScenarioExplorer.ViewModels
{
    [AddINotifyPropertyChangedInterface]
    public class DataPreviewViewModel
    {
        public DataPreviewViewModel(int offset, TagViewModel tag)
        {
            var data = tag.Data.Span;

            this.Byte = data[offset];
            this.Short = data.ReadInt16At(offset);
            this.UShort = data.ReadUInt16At(offset);
            this.Int = data.ReadInt32At(offset);
            this.UInt = data.ReadUInt32At(offset);
            this.Float = data.ReadFloatAt(offset);
            this.String = data.ReadStringFrom(offset, 32);

            this.InternalOffset = (uint)(data.ReadUInt32At(offset) - tag.InternalOffsetStart);
        }

        public byte Byte { get; set; }

        public short Short { get; set; }

        public ushort UShort { get; set; }

        public int Int { get; set; }

        public uint UInt { get; set; }

        public float Float { get; set; }

        public string String { get; set; }

        public uint InternalOffset { get; set; }
    }
}
